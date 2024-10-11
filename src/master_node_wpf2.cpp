#include "fitrobot_interfaces/msg/robot_status.hpp"
#include "fitrobot_interfaces/msg/station.hpp"
#include "fitrobot_interfaces/msg/station_list.hpp"
#include "fitrobot_interfaces/srv/cancel_nav.hpp"
#include "fitrobot_interfaces/srv/list_station.hpp"
#include "fitrobot_interfaces/srv/navigation.hpp"
#include "fitrobot_interfaces/srv/para1.hpp"
#include "fitrobot_interfaces/srv/remote_control.hpp"
#include "fitrobot_interfaces/srv/slam.hpp"
#include "fitrobot_interfaces/srv/subscription_count.hpp"
#include "fitrobot_interfaces/srv/target_station.hpp"
#include "fitrobot_interfaces/srv/terminate_process.hpp"
#include "fitrobot_interfaces/srv/trigger.hpp"
#include "fitrobot_interfaces/srv/waypoint_follower.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <condition_variable>
#include <deque>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp> // 需要添加 JSON 庫
#include <queue>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <tuple> // Include for std::tuple
#include <unordered_map>

using json = nlohmann::json;
using fitrobot_interfaces::msg::RobotStatus;
using fitrobot_interfaces::msg::Station;
using fitrobot_interfaces::msg::StationList;
using fitrobot_interfaces::srv::CancelNav;
using fitrobot_interfaces::srv::ListStation;
using fitrobot_interfaces::srv::WaypointFollower;
using nav2_msgs::action::FollowWaypoints;
using nav2_msgs::action::NavigateThroughPoses;
using nav2_msgs::action::NavigateToPose;
using std::placeholders::_1;
using std::placeholders::_2;

std::tuple<std::string, std::string, std::string> parse_robot_info(const std::string& robot_info) {
    RCLCPP_INFO(rclcpp::get_logger("robot_info"), "%s", robot_info.c_str());
    std::string robot_info_str(robot_info);
    std::stringstream ss(robot_info_str);
    std::string first_robot_info;
    std::getline(ss, first_robot_info, ';'); // 取得第一個機器人的信息

    std::stringstream robot_ss(first_robot_info);
    std::string robot_type, robot_sn;
    std::getline(robot_ss, robot_type, ':'); // Get robot type
    std::getline(robot_ss, robot_sn, ':');   // Get robot serial number

    std::string node_namespace = "/" + robot_type + "_" + robot_sn;
    return std::make_tuple(robot_type, robot_sn, node_namespace);
}
class MasterAsyncService : public rclcpp::Node {
  public:
    MasterAsyncService(const std::string& node_name, const rclcpp::NodeOptions& options)
        : Node(node_name, options) {
        set_robot_info_from_env();
        const char* workspace_ = std::getenv("WORKSPACE");
        if (!workspace_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Environment variable WORKSPACE is not set! Use default workspace=simulations");
            workspace = "simulations";
        } else {
            workspace = workspace_;
            RCLCPP_INFO(this->get_logger(), "ABC WORKSPACE: %s", workspace.c_str());
        }
        this->declare_parameter("waypoint_queue_size", 0);

        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        auto qos_station = rclcpp::QoS(1);
        pub_ = this->create_publisher<RobotStatus>("robot_status", qos_pub);
        station_pub_ = this->create_publisher<Station>("target_station", qos_pub);

        service_cbg_MU = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_cbg_RE = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // services
        nav_srv_ = this->create_service<fitrobot_interfaces::srv::Navigation>(
            "navigation", std::bind(&MasterAsyncService::navigation_callback, this, _1, _2),
            rmw_qos_profile_services_default, service_cbg_MU);
        slam_srv_ = this->create_service<fitrobot_interfaces::srv::Slam>(
            "slam", std::bind(&MasterAsyncService::slam_callback, this, _1, _2),
            rmw_qos_profile_services_default, service_cbg_MU);
        remote_control_srv_ = this->create_service<fitrobot_interfaces::srv::RemoteControl>(
            "remote_control", std::bind(&MasterAsyncService::remote_control_callback, this, _1, _2),
            rmw_qos_profile_services_default, service_cbg_MU);
        terminate_srv_ = this->create_service<fitrobot_interfaces::srv::TerminateProcess>(
            "terminate_slam_or_navigation",
            std::bind(&MasterAsyncService::terminate_slam_or_navigation_callback, this, _1, _2),
            rmw_qos_profile_services_default, service_cbg_MU);
        subscription_count_srv_ = this->create_service<fitrobot_interfaces::srv::SubscriptionCount>(
            "subscription_count",
            std::bind(&MasterAsyncService::subscription_count_callback, this, _1, _2),
            rmw_qos_profile_services_default, service_cbg_MU);
        lifecycle_nav_service_ = this->create_service<fitrobot_interfaces::srv::Trigger>(
            "lifecycle_nav", std::bind(&MasterAsyncService::lifecycle_nav_callback, this, _1, _2),
            rmw_qos_profile_services_default, service_cbg_RE);
        // rmw_qos_profile_services_default, service_cbg_MU);
        target_station_srv = this->create_service<fitrobot_interfaces::srv::TargetStation>(
            "target_station", std::bind(&MasterAsyncService::target_station_callback, this,
                                        std::placeholders::_1, std::placeholders::_2));
        cancel_task_srv = this->create_service<CancelNav>(
            "cancel_task", std::bind(&MasterAsyncService::cancel_task_callback, this, _1, _2),
            rmw_qos_profile_services_default, service_cbg_MU);

        // service clients
        lfm_nav_client = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
            node_namespace + "/lifecycle_manager_navigation/manage_nodes",
            rmw_qos_profile_services_default, service_cbg_MU);
        nav_to_pose_client_ =
            rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        bt_navigator_getstate_client = this->create_client<lifecycle_msgs::srv::GetState>(
            node_namespace + "/bt_navigator/get_state", rmw_qos_profile_services_default,
            service_cbg_MU);
        list_station_client_ = this->create_client<ListStation>(
            "/list_station", rmw_qos_profile_services_default, service_cbg_MU);

        // action clients
        follow_waypoints_client_ =
            rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

        // subscriptions
        sub_options.callback_group = service_cbg_MU;
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goalpose", rclcpp::SystemDefaultsQoS(),
            std::bind(&MasterAsyncService::onGoalPoseReceived, this, std::placeholders::_1));
        goals_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "goalposes", rclcpp::SystemDefaultsQoS(),
            std::bind(&MasterAsyncService::onGoalPoseArrayReceived, this, std::placeholders::_1));
        initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&MasterAsyncService::initialpose_callback, this, _1),
            sub_options);

        // Further initialization...
        std::unordered_map<std::string, std::string> package_names = {
            {"artic", "articubot_one"}, {"lino2", "linorobot2_navigation"}};
        if (package_names.find(robot_type) != package_names.end()) {
            package_name = package_names[robot_type];
        } else {
            throw std::invalid_argument("Unknown robot type: " + robot_type);
        }

        robot_param_srv = this->create_service<fitrobot_interfaces::srv::Para1>(
            "robotparam", std::bind(&MasterAsyncService::robotparam_callback, this,
                                    std::placeholders::_1, std::placeholders::_2));

        waypoint_follower_srv_ = this->create_service<WaypointFollower>(
            "waypointfollower",
            std::bind(&MasterAsyncService::waypointFollowerCallback, this, std::placeholders::_1,
                      std::placeholders::_2),
            rmw_qos_profile_services_default, service_cbg_MU);

        waypoint_queue_thread_ = std::thread(&MasterAsyncService::waypointQueueConsumer, this);
        // waypoint_queue_future_ =
        //     std::async(std::launch::async, &MasterAsyncService::waypointQueueConsumer, this);

        last_feedback_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        target_station = Station();

        // cansleep_ = get_parameter_bool(node_namespace + "/check_robot_status_node", "can_sleep");
        // --> failed. will timeout
    }

    ~MasterAsyncService() {
        // {
        // std::lock_guard<std::mutex> lq(queue_mutex_);
        // std::<std::mutex> lc(canconsume_mutex_);
        // can_consume_queue_ = false;
        // }
        //     if (waypoint_queue_thread_.joinable()) {
        //         waypoint_queue_thread_.join();
        //     }
        if (waypoint_queue_future_.valid()) {
            waypoint_queue_future_.get();
        }
    }

  private:
    std::string workspace;
    std::string robot_type;
    std::string robot_sn;
    std::string node_namespace;
    std::string package_name;
    bool launch_service_active;
    pid_t launch_service_pid; // PID of the launch service process
    Station start_station;
    Station end_station;
    Station target_station;
    rclcpp::SubscriptionOptions sub_options;
    rclcpp::Publisher<fitrobot_interfaces::msg::RobotStatus>::SharedPtr pub_;
    rclcpp::Publisher<Station>::SharedPtr station_pub_;
    rclcpp::CallbackGroup::SharedPtr service_cbg_MU;
    rclcpp::CallbackGroup::SharedPtr service_cbg_RE;
    rclcpp::Service<fitrobot_interfaces::srv::Navigation>::SharedPtr nav_srv_;
    rclcpp::Service<fitrobot_interfaces::srv::Slam>::SharedPtr slam_srv_;
    rclcpp::Service<fitrobot_interfaces::srv::RemoteControl>::SharedPtr remote_control_srv_;
    rclcpp::Service<fitrobot_interfaces::srv::TerminateProcess>::SharedPtr terminate_srv_;
    rclcpp::Service<fitrobot_interfaces::srv::SubscriptionCount>::SharedPtr subscription_count_srv_;
    rclcpp::Service<fitrobot_interfaces::srv::Navigation>::SharedPtr param_set_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr manage_nodes_service_;
    rclcpp::Service<fitrobot_interfaces::srv::Trigger>::SharedPtr lifecycle_nav_service_;
    rclcpp::Service<fitrobot_interfaces::srv::TargetStation>::SharedPtr target_station_srv;
    rclcpp::Service<CancelNav>::SharedPtr cancel_task_srv;

    std::shared_future<rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr>
        goal_handle_future;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goals_sub;

    std::unordered_map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> param_clients_;
    std::shared_ptr<rclcpp::AsyncParametersClient> param_client_robot_status;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr
        lfm_nav_client; // lifecycle_manager_navigation
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr bt_navigator_getstate_client;
    rclcpp::Client<ListStation>::SharedPtr list_station_client_;
    rclcpp::Service<fitrobot_interfaces::srv::Para1>::SharedPtr robot_param_srv;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
    rclcpp::Time last_feedback_time_;
    size_t qsize;
    bool cansleep_;
    std::vector<Station> current_stationlist;
    std::vector<Station> waypoints_;
    // Waypoint queue and synchronization
    std::deque<Station> waypoint_queue_;
    int current_waypoint;
    bool can_consume_queue_ = true;
    std::mutex queue_mutex_;      // protect waypoint_queue_
    std::mutex waypoints_mutex_;  // protect waypoints_
    std::mutex canconsume_mutex_; // protect can_consume_queue_
    std::mutex feedback_mutex_;   // protect current_waypoint
    std::condition_variable canconsume_cv_;
    std::condition_variable queue_cv_;
    std::thread waypoint_queue_thread_;
    std::future<void> waypoint_queue_future_;

    bool is_bt_navigator_active();
    void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    void GoToPose(const geometry_msgs::msg::PoseStamped& pose);
    void onGoalPoseArrayReceived(const geometry_msgs::msg::PoseArray::SharedPtr poses);
    void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void shutdown_launch_service();
    void clean_up(bool ensure_bringup);
    void launch_function_async(const std::string& package_name, const std::string& launch_file,
                               const std::vector<std::string>& args);
    void terminate_process_group(pid_t pid);
    void
    lifecycle_manage_cmd(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client,
                         int cmd);
    void startup_nav(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client);
    void pause_nav(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client);
    void resume_nav(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client);
    void reset_nav(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client);
    void lifecycle_nav_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::Trigger::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::Trigger::Response> response);
    void target_station_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::TargetStation::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::TargetStation::Response> response);
    void cancel_task_callback(const std::shared_ptr<CancelNav::Request> request,
                              std::shared_ptr<CancelNav::Response> response);
    void set_robot_info_from_env();
    int get_robot_status();
    bool get_sim_time(const std::string& node_nmae);
    int get_parameter_int(const std::string& node_name, const std::string& param_name);
    bool get_parameter_bool(const std::string& node_name, const std::string& param_name);
    std::string get_parameter_string(const std::string& node_name, const std::string& param_name);
    bool get_parameter_for_node(const std::string& node_name, const std::string& param_name,
                                rclcpp::Parameter& param_value);
    bool set_parameter_for_node(const std::string& node_name, const rclcpp::Parameter& param_value);
    void set_parameters_map_mask(std::string worldname);
    void run_navigation_async(std::string worldname);
    void navigation_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::Navigation::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::Navigation::Response> response);
    bool check_substring(const std::string& substring);
    void ensure_robotstatus_slam();
    void run_slam_async();
    void slam_callback(const std::shared_ptr<fitrobot_interfaces::srv::Slam::Request> request,
                       std::shared_ptr<fitrobot_interfaces::srv::Slam::Response> response);
    void remote_control_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::RemoteControl::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::RemoteControl::Response> response);
    void terminate_slam_or_navigation_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::TerminateProcess::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::TerminateProcess::Response> response);
    void subscription_count_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::SubscriptionCount::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::SubscriptionCount::Response> response);
    rclcpp::Service<fitrobot_interfaces::srv::WaypointFollower>::SharedPtr waypoint_follower_srv_;
    void waypointQueueConsumer();
    void followWaypoints(const std::vector<Station>& waypoints);
    void goalResponseCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr goal_handle);
    void feedbackCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback);
    void resultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult&
            result);
    void waypointFollowerCallback(
        const std::shared_ptr<fitrobot_interfaces::srv::WaypointFollower::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::WaypointFollower::Response> response);
    void
    robotparam_callback(const std::shared_ptr<fitrobot_interfaces::srv::Para1::Request> request,
                        std::shared_ptr<fitrobot_interfaces::srv::Para1::Response> response);
};

// Implementations of the functions go here...
void MasterAsyncService::shutdown_launch_service() {
    if (launch_service_active && launch_service_pid > 0) {
        RCLCPP_INFO(this->get_logger(), "Shutting down launch service with PID: %d",
                    launch_service_pid);
        terminate_process_group(launch_service_pid);
        launch_service_active = false;
        launch_service_pid = -1;
    } else {
        RCLCPP_WARN(this->get_logger(), "No active launch service to shut down.");
    }
}

void MasterAsyncService::clean_up(bool ensure_bringup = true) {
    RCLCPP_INFO(this->get_logger(), "clean_up: ready to shutdown.");
    shutdown_launch_service();
    if (ensure_bringup) {
        rclcpp::Rate rate(1); // 1Hz 频率
        while (rclcpp::ok()) {
            RCLCPP_INFO(this->get_logger(), "clean_up: ready to shutdown.");
            if (get_robot_status() == RobotStatus::BRINGUP) {
                break;
            }
            rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "clean_up: done. robot_status back to BRINGUP");
    }
}

void MasterAsyncService::launch_function_async(const std::string& package_name,
                                               const std::string& launch_file,
                                               const std::vector<std::string>& args) {
    pid_t pid = fork();
    if (pid == 0) {
        // 子進程：設置新的進程組 ID
        setpgid(0, 0);
        std::ostringstream launch_command;
        std::string source_prefix = "source ~/" + workspace + "/install/setup.bash && ";
        launch_command << source_prefix << "ros2 launch " << package_name << " " << launch_file;
        for (const auto& arg : args) {
            launch_command << " " << arg;
        }
        std::string command_str = launch_command.str();
        const char* command = command_str.c_str();
        RCLCPP_INFO(this->get_logger(), "Launching command: %s", command);
        // 使用 execl 執行 bash 命令
        execl("/bin/bash", "bash", "-c", command, (char*)nullptr);
        // 如果 execl 返回，則表示出現錯誤
        _exit(EXIT_FAILURE);
    } else if (pid < 0) {
        // 父進程：處理 fork 失敗的情況
        RCLCPP_ERROR(this->get_logger(), "Failed to fork a new process for launch command.");
    } else {
        // 父進程：成功創建子進程，設置全局 PID 為子進程的 PID
        launch_service_pid = pid;
        launch_service_active = true;
        // 設置子進程的進程組 ID 與子進程本身相同
        setpgid(pid, pid);
        RCLCPP_DEBUG(this->get_logger(), "Successfully launched %s with launch file %s. PID: %d",
                     package_name.c_str(), launch_file.c_str(), pid);
    }
}

void MasterAsyncService::terminate_process_group(pid_t pid) {
    if (pid <= 0)
        return;
    // 使用負的 PID 來表示進程組 ID
    if (kill(-pid, SIGTERM) == 0) {
        RCLCPP_DEBUG(this->get_logger(), "Successfully sent SIGTERM to process group %d", pid);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send SIGTERM to process group %d: %s", pid,
                     strerror(errno));
    }
    // 可選：使用 SIGKILL 強制終止所有進程
    if (kill(-pid, SIGKILL) == 0) {
        RCLCPP_DEBUG(this->get_logger(), "Successfully sent SIGKILL to process group %d", pid);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send SIGKILL to process group %d: %s", pid,
                     strerror(errno));
    }
}

void MasterAsyncService::lifecycle_manage_cmd(
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client, int cmd) {
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "ManageLifecycleNodes service not available.");
        return;
    }
    auto request_srv = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    request_srv->command = cmd;
    auto future = client->async_send_request(request_srv);
    auto status = future.wait_for(std::chrono::seconds(6));
    if (status == std::future_status::ready) {
        try {
            auto result = future.get();
            RCLCPP_INFO(this->get_logger(), "ManageLifecycleNodes service call succeeded. cmd=%d",
                        cmd);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Timeout while waiting for ManageLifecycleNodes service to complete.");
    }
}

void MasterAsyncService::startup_nav(
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client) {
    lifecycle_manage_cmd(client, 0);
}

void MasterAsyncService::pause_nav(
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client) {
    lifecycle_manage_cmd(client, 1);
}

void MasterAsyncService::resume_nav(
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client) {
    lifecycle_manage_cmd(client, 2);
}

void MasterAsyncService::reset_nav(
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client) {
    lifecycle_manage_cmd(client, 3);
}

void MasterAsyncService::lifecycle_nav_callback(
    const std::shared_ptr<fitrobot_interfaces::srv::Trigger::Request> request,
    std::shared_ptr<fitrobot_interfaces::srv::Trigger::Response> response) {
    if (request->trigger_name == "startup") {
        startup_nav(lfm_nav_client);
    } else if (request->trigger_name == "pause") {
        pause_nav(lfm_nav_client);
    } else if (request->trigger_name == "resume") {
        resume_nav(lfm_nav_client);
    } else if (request->trigger_name == "reset") {
        reset_nav(lfm_nav_client);
    }
}

void MasterAsyncService::target_station_callback(
    const std::shared_ptr<fitrobot_interfaces::srv::TargetStation::Request> request,
    std::shared_ptr<fitrobot_interfaces::srv::TargetStation::Response> response) {
    RCLCPP_INFO(this->get_logger(), "target_station_callback started");
    response->target_station = target_station;
    RCLCPP_INFO(this->get_logger(), "target_station_callback finished");
}

void MasterAsyncService::cancel_task_callback(const std::shared_ptr<CancelNav::Request> request,
                                              std::shared_ptr<CancelNav::Response> response) {
    RCLCPP_INFO(this->get_logger(), "cancel_task_callback started");
    int idx = request->idx;

    {
        std::lock_guard<std::mutex> lg(queue_mutex_);
        if (idx < 0 || idx >= (int)waypoint_queue_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid index: %d", idx);
            response->ack = "cancel_task failed";
            return;
        }
    }
    if (idx == 0) {
        if (goal_handle_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto goal_handle = goal_handle_future.get();
            if (goal_handle) {
                RCLCPP_INFO(this->get_logger(), "Canceling the current task...");
                auto future_cancel = follow_waypoints_client_->async_cancel_goal(goal_handle);
                auto status_cancel = future_cancel.wait_for(std::chrono::seconds(5));
                if (status_cancel == std::future_status::ready) {
                    try {
                        auto result_cancel = future_cancel.get();
                        RCLCPP_INFO(this->get_logger(), "cancel task success");
                        response->ack = "cancel_task success";
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                    }
                } else {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Timeout while waiting for the cancel task operation to complete.");
                }
                return;
            }
        }
        return;
    }
    {
        std::lock_guard<std::mutex> lg(queue_mutex_);
        waypoint_queue_.erase(waypoint_queue_.begin() + idx);
    }
    response->ack = "cancel_task success";
    RCLCPP_INFO(this->get_logger(), "cancel_task_callback finished");
}

void MasterAsyncService::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    GoToPose(*pose);
}

void MasterAsyncService::GoToPose(const geometry_msgs::msg::PoseStamped& pose) {
    // Startup all the lifecycle nodes
    startup_nav(lfm_nav_client);
    if (!is_bt_navigator_active()) {
        RCLCPP_INFO(this->get_logger(), "bt_navigator is not active, proceeding to navigation.");
        return;
    }

    // Send goal to navigate_to_pose action server
    NavigateToPose::Goal goal;
    goal.pose = pose;

    if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available.");
        return;
    }

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](const auto& result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation to pose succeeded.");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation to pose was aborted.");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Navigation to pose was canceled.");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
            break;
        }
    };

    auto goal_handle_future = nav_to_pose_client_->async_send_goal(goal, send_goal_options);

    // Wait for the result
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        auto goal_handle = goal_handle_future.get();
        if (goal_handle) {
            auto result_future = nav_to_pose_client_->async_get_result(goal_handle);
            if (result_future.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
                auto result = result_future.get();
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Successfully reached the goal.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to reach the goal.");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for the result.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server.");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for the goal to be accepted.");
    }
}

// void MasterAsyncService::GoToPose(const geometry_msgs::msg::PoseStamped& pose) {
//     // startup all the lifecycle nodes
//     startup_nav(lfm_nav_client);
//     if (!is_bt_navigator_active()) {
//         RCLCPP_INFO(this->get_logger(), "bt_navigator is not active, proceeding to navigation.");
//         return;
//     }
//     // send goal to navigate_to_pose action server
//     NavigateToPose::Goal goal;
//     goal.pose = pose;
//     nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5));
//     nav_to_pose_client_->async_send_goal(goal);
// }

void MasterAsyncService::onGoalPoseArrayReceived(
    const geometry_msgs::msg::PoseArray::SharedPtr poses) {
    // startup all the lifecycle nodes
    startup_nav(lfm_nav_client);
    if (!is_bt_navigator_active()) {
        RCLCPP_INFO(this->get_logger(), "bt_navigator is not active, proceeding to navigation.");
        return;
    }
    FollowWaypoints::Goal goal;
    goal.poses.clear();
    for (const auto& pose : poses->poses) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = pose;
        pose_stamped.header = poses->header; // 設置 header
        goal.poses.push_back(pose_stamped);
    }
    follow_waypoints_client_->wait_for_action_server(std::chrono::seconds(5));
    follow_waypoints_client_->async_send_goal(goal);
}

void MasterAsyncService::initialpose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    auto sta_msg = RobotStatus();
    sta_msg.status = RobotStatus::NAV_PREPARE_TO_READY;
    pub_->publish(sta_msg);
    set_parameter_for_node(node_namespace + "/check_robot_status_node",
                           rclcpp::Parameter("fitrobot_status", RobotStatus::NAV_PREPARE_TO_READY));
}

bool MasterAsyncService::is_bt_navigator_active() {
    RCLCPP_INFO(this->get_logger(), "is_bt_navigator_active started");
    if (!bt_navigator_getstate_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "GetState service not available for bt_navigator");
        return false;
    }
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = bt_navigator_getstate_client->async_send_request(request);
    auto status = future.wait_for(std::chrono::seconds(5));
    if (status == std::future_status::ready) {
        auto response = future.get();
        if (response->current_state.label == "active") {
            RCLCPP_INFO(this->get_logger(), "bt_navigator is active.");
            return true;
        }
    } else {
        RCLCPP_ERROR(
            this->get_logger(),
            "is_bt_navigator_active: Timeout while waiting for request of GetState service.");
    }
    return false;
}

bool MasterAsyncService::set_parameter_for_node(const std::string& node_name,
                                                const rclcpp::Parameter& param_value) {
    // 如果param_clients_中没有该节点的参数客户端，则创建新的客户端
    if (param_clients_.find(node_name) == param_clients_.end()) {
        param_clients_[node_name] =
            std::make_shared<rclcpp::AsyncParametersClient>(this, node_name);
    }
    // 获取参数客户端
    auto param_client = param_clients_[node_name];
    // 等待服务可用
    if (!param_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Parameter service not available for node %s",
                     node_name.c_str());
        return false;
    }
    // 异步设置参数
    auto future = param_client->set_parameters({param_value});
    auto status = future.wait_for(std::chrono::milliseconds(100));
    if (status == std::future_status::ready) {
        try {
            auto result = future.get();
            if (result[0].successful) {
                RCLCPP_DEBUG(this->get_logger(), "Successfully set parameter %s on node %s",
                             param_value.get_name().c_str(), node_name.c_str());
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s",
                             result[0].reason.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Timeout while waiting for the parameter set operation to complete.");
    }
    return false;
}

void MasterAsyncService::robotparam_callback(
    const std::shared_ptr<fitrobot_interfaces::srv::Para1::Request> request,
    std::shared_ptr<fitrobot_interfaces::srv::Para1::Response> response) {
    RCLCPP_INFO(this->get_logger(), "robotparam_callback started");
    std::string parameter1_name = request->parameter1_name;
    std::string parameter1_value = request->parameter1_value;
    if (parameter1_name == "select_stationlist") {
        auto log_station_info = [](rclcpp::Logger logger, const std::string& station_name,
                                   const std::string& station_type, double x, double y, double z,
                                   double w) {
            RCLCPP_INFO(logger,
                        "station: %-20s (type: %-8s) [x,y,z,w]= [%6.1f, %6.1f, %6.3f, %6.3f]",
                        station_name.c_str(), station_type.c_str(), x, y, z, w);
        };
        auto convertToJsonString = [](const std::string& input) {
            std::string output = input;
            output = std::regex_replace(output, std::regex("'"), "\"");
            output = std::regex_replace(output, std::regex("\\bTrue\\b"), "\"true\"");
            output = std::regex_replace(output, std::regex("\\bFalse\\b"), "\"false\"");
            return output;
        };

        RCLCPP_INFO(this->get_logger(), "parameter1_value: %s", parameter1_value.c_str());
        std::string json_string = convertToJsonString(parameter1_value);
        RCLCPP_INFO(this->get_logger(), "json_string: %s", json_string.c_str());
        json jsoninfo = json::parse(json_string);
        // std::string use_sim = jsoninfo["use_sim"];
        bool use_sim = (jsoninfo["use_sim"] == "true");
        std::string map_key = jsoninfo["map_key"];
        std::string station_key = jsoninfo["station_key"];
        auto request_ = std::make_shared<ListStation::Request>();
        // request_->use_sim = true;
        request_->use_sim = use_sim;
        request_->map_key = map_key;
        request_->station_key = station_key;
        list_station_client_->wait_for_service(std::chrono::seconds(5));
        list_station_client_->async_send_request(request_);

        auto future = list_station_client_->async_send_request(request_);
        auto status = future.wait_for(std::chrono::seconds(5));
        if (status == std::future_status::ready) {
            auto resp = future.get();
            current_stationlist = resp->station_list;
            RCLCPP_INFO(this->get_logger(), "current_stationlist size: %zu",
                        current_stationlist.size());
            for (const auto& station : current_stationlist) {
                auto type = station.type;
                auto name = station.name;
                auto x = station.x;
                auto y = station.y;
                auto z = station.z;
                auto w = station.w;
                log_station_info(this->get_logger(), name, type, x, y, z, w);
                if (type == "start") {
                    start_station = station;
                } else if (type == "end") {
                    end_station = station;
                }
            }
            RCLCPP_INFO(this->get_logger(), "start_station: %s", start_station.name.c_str());
            RCLCPP_INFO(this->get_logger(), "end_station: %s", end_station.name.c_str());
            response->success = "true";
        } else {
            RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for GetState service.");
        }

        RCLCPP_INFO(this->get_logger(), "robotparam_callback finished");
    }
}

void MasterAsyncService::waypointFollowerCallback(
    const std::shared_ptr<fitrobot_interfaces::srv::WaypointFollower::Request> request,
    std::shared_ptr<fitrobot_interfaces::srv::WaypointFollower::Response> response) {
    Station station = request->station;
    cansleep_ = get_parameter_bool(node_namespace + "/check_robot_status_node", "can_sleep");
    RCLCPP_INFO(this->get_logger(), "waypointFollowerCallback started. station: %s",
                station.name.c_str());
    // check validity of station compared with stationlist
    if (current_stationlist.size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "stationlist is empty");
        response->ack = "stationlist is empty. Please set stationlist first.";
        return;
    }
    for (const auto& each : current_stationlist) {
        RCLCPP_INFO(this->get_logger(), "each: %s", each.name.c_str());
        auto sta = Station();
        if (each.name == station.name) {
            if (station.x == 0.0 && station.y == 0.0 && station.z == 0.0 && station.w == 0.0) {
                RCLCPP_INFO(this->get_logger(), "request station (x,y,z,w) is (0,0,0,0). Use "
                                                "station in stationlist instead.");
                sta = each;
            } else {
                sta = station;
            }
            std::lock_guard<std::mutex> lock(queue_mutex_);
            waypoint_queue_.push_back(sta); // use station in request
            char buffer[256];
            snprintf(buffer, sizeof(buffer),
                     "Added waypoint to queue. Queue size: %zu. Added station %s --> "
                     "(x,y,z,w)=(%g, %g, %g, %g)",
                     waypoint_queue_.size(), sta.name.c_str(), sta.x, sta.y, sta.z, sta.w);
            RCLCPP_INFO(this->get_logger(), "%s", buffer);
            response->ack = buffer;
            break;
        } else {
            RCLCPP_INFO(this->get_logger(), "station name not match with curent stationlist");
            response->ack = "station name not match with curent stationlist";
        }
    }
}

void MasterAsyncService::waypointQueueConsumer() {
    rclcpp::Rate rate(1.0); // 1Hz 频率
    qsize = -1;
    bool is_queue_empty;
    while (rclcpp::ok()) {
        {
            std::unique_lock<std::mutex> lc(canconsume_mutex_);
            canconsume_cv_.wait(lc, [this]() { return can_consume_queue_; });
        }
        {
            std::lock_guard<std::mutex> lq(queue_mutex_);
            is_queue_empty = waypoint_queue_.empty();
            if (waypoint_queue_.size() != qsize) {
                qsize = waypoint_queue_.size();
                RCLCPP_INFO(this->get_logger(), "目前排隊任務數:%zu", qsize);
            }
        }
        if (is_queue_empty) {
            RCLCPP_INFO(this->get_logger(), "waypoint_queue_ is empty");
            if (target_station != Station()) {
                rclcpp::Rate rate(10);
                while (rclcpp::ok()) {
                    RCLCPP_INFO(this->get_logger(), "clean_up: ready to shutdown.");
                    int robot_status = get_robot_status();
                    if (robot_status == RobotStatus::NAV_WF_COMPLETED ||
                        robot_status == RobotStatus::NAV_WF_CANCEL ||
                        robot_status == RobotStatus::NAV_WF_FAILED) {
                        break;
                    }
                    rate.sleep();
                }
                // while (rclcpp::ok()) {
                //     RCLCPP_INFO(this->get_logger(), "clean_up: ready to shutdown.");
                //     int robot_status = get_robot_status();
                //     if (robot_status == RobotStatus::NAV_RUNNING ){
                //         break;
                //     }
                //     rate.sleep();
                // }
                auto start_pose = geometry_msgs::msg::PoseStamped();
                start_pose.header.frame_id = "map";
                start_pose.pose.position.x = start_station.x;
                start_pose.pose.position.y = start_station.y;
                start_pose.pose.orientation.z = start_station.z;
                start_pose.pose.orientation.w = start_station.w;
                RCLCPP_INFO(this->get_logger(),
                            "Starting to go back to home. (x,y,z,w)=(%g, %g, %g, %g)",
                            start_station.x, start_station.y, start_station.z, start_station.w);
                station_pub_->publish(start_station);
                GoToPose(start_pose);
                target_station = Station();
                if (cansleep_) {
                    set_parameter_for_node(node_namespace + "/check_robot_status_node",
                                           rclcpp::Parameter("enable_sleep", true));
                    RCLCPP_INFO(this->get_logger(),
                                "Ready to go back home. Set enable_sleep to true again.");
                }
            }
            rate.sleep();
            continue;
        }
        RCLCPP_INFO(this->get_logger(), "Processing waypoint...");
        if (cansleep_) {
            RCLCPP_INFO(this->get_logger(), "disable sleep during wf navigation...");
            set_parameter_for_node(node_namespace + "/check_robot_status_node",
                                   rclcpp::Parameter("enable_sleep", false));
        }
        std::vector<Station> waypoints;
        {
            std::lock_guard<std::mutex> lq(queue_mutex_);
            waypoints.push_back(waypoint_queue_.front());
            waypoints.push_back(end_station);
        }
        RCLCPP_INFO(this->get_logger(), "Starting to follow waypoints...");
        followWaypoints(waypoints);
    }
    rate.sleep();
}

void MasterAsyncService::followWaypoints(const std::vector<Station>& waypoints) {
    RCLCPP_INFO(this->get_logger(), "followWaypoints started");
    {
        std::unique_lock<std::mutex> lc(canconsume_mutex_);
        can_consume_queue_ = false;
        canconsume_cv_.notify_one();
    }
    {
        std::lock_guard<std::mutex> lc(waypoints_mutex_);
        waypoints_ = waypoints;
    }
    startup_nav(lfm_nav_client);
    if (!follow_waypoints_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "FollowWaypoints action server not available.");
        return;
    }
    // tranfer waypoints to poses
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (const auto& waypoint : waypoints) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = waypoint.x;
        pose.pose.position.y = waypoint.y;
        pose.pose.orientation.z = waypoint.z;
        pose.pose.orientation.w = waypoint.w;
        poses.push_back(pose);
    }
    // Create a goal message
    auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
    goal_msg.poses = poses;
    // Define send goal options with callbacks
    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MasterAsyncService::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&MasterAsyncService::feedbackCallback, this,
                                                    std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&MasterAsyncService::resultCallback, this, std::placeholders::_1);
    // Send the goal asynchronously
    goal_handle_future = follow_waypoints_client_->async_send_goal(goal_msg, send_goal_options);
    station_pub_->publish(waypoints_.front());
}

void MasterAsyncService::set_robot_info_from_env() {
    const char* robot_info = std::getenv("ROBOT_INFO");
    if (!robot_info) {
        RCLCPP_WARN(this->get_logger(),
                    "Environment variable ROBOT_INFO is not set! Use default namespace=''");
        node_namespace = "";
        robot_type = "lino2";
        robot_sn = "0001";
    } else {
        std::tie(robot_type, robot_sn, node_namespace) = parse_robot_info(robot_info);
    }
    RCLCPP_INFO(this->get_logger(), "Node name set to: %s", node_namespace.c_str());
}

int MasterAsyncService::get_robot_status() {
    return get_parameter_int(node_namespace + "/check_robot_status_node", "fitrobot_status");
}

bool MasterAsyncService::get_sim_time(const std::string& node_nmae) {
    return get_parameter_bool(node_nmae, "use_sim_time");
}

int MasterAsyncService::get_parameter_int(const std::string& node_name,
                                          const std::string& param_name) {
    rclcpp::Parameter param_value;
    if (get_parameter_for_node(node_name, param_name, param_value)) {
        if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
            return param_value.as_int();
        } else {
            RCLCPP_ERROR(this->get_logger(), "%s parameter is not of type integer.",
                         param_name.c_str());
        }
    }
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameter %s from node %s", param_name.c_str(),
                 node_name.c_str());
    return -1; // Return a default or error value if the parameter is not found or not an
               // integer
}

bool MasterAsyncService::get_parameter_bool(const std::string& node_name,
                                            const std::string& param_name) {
    rclcpp::Parameter param_value;
    if (get_parameter_for_node(node_name, param_name, param_value)) {
        if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            return param_value.as_bool();
        } else {
            RCLCPP_ERROR(this->get_logger(), "%s parameter is not of type bool.",
                         param_name.c_str());
        }
    }
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameter %s from node %s", param_name.c_str(),
                 node_name.c_str());
    return false; // Return a default or error value if the parameter is not found or not an
                  // integer
}

std::string MasterAsyncService::get_parameter_string(const std::string& node_name,
                                                     const std::string& param_name) {
    rclcpp::Parameter param_value;
    if (get_parameter_for_node(node_name, param_name, param_value)) {
        if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            return param_value.as_string();
        } else {
            RCLCPP_ERROR(this->get_logger(), "%s parameter is not of type string.",
                         param_name.c_str());
        }
    }
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameter %s from node %s", param_name.c_str(),
                 node_name.c_str());
    return ""; // Return a default or error value if the parameter is not found or not an
               // integer
}

bool MasterAsyncService::get_parameter_for_node(const std::string& node_name,
                                                const std::string& param_name,
                                                rclcpp::Parameter& param_value) {
    if (param_clients_.find(node_name) == param_clients_.end()) {
        param_clients_[node_name] =
            std::make_shared<rclcpp::AsyncParametersClient>(this, node_name);
    }
    auto param_client = param_clients_[node_name];
    if (!param_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Parameter service not available for node %s",
                     node_name.c_str());
        return false;
    }
    auto future = param_client->get_parameters({param_name});
    auto status = future.wait_for(std::chrono::milliseconds(1000));
    if (status == std::future_status::ready) {
        try {
            auto result = future.get();
            if (!result.empty()) {
                param_value = result[0]; // Save the parameter value
                RCLCPP_DEBUG(this->get_logger(), "Successfully got parameter %s from node %s",
                             param_name.c_str(), node_name.c_str());
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to get parameter %s from node %s",
                             param_name.c_str(), node_name.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(),
                     "Timeout while waiting for the parameter get operation to complete.");
    }
    return false;
}

void MasterAsyncService::set_parameters_map_mask(std::string worldname) {
    std::string param_name = "map_topic";
    std::string param_value = "/" + worldname + "/" + robot_type + "/map";
    std::string param_name2 = "mask_topic";
    std::string param_value2 = "/" + worldname + "/" + robot_type + "/keepout_filter_mask";
    std::vector<std::string> nodes_to_update = {node_namespace + "/amcl",
                                                node_namespace + "/local_costmap/local_costmap",
                                                node_namespace + "/global_costmap/global_costmap",
                                                node_namespace + "/costmap_filter_info_server"};

    rclcpp::Parameter map_param(param_name, param_value);
    rclcpp::Parameter mask_param(param_name2, param_value2);

    bool success = false;
    for (const auto& node : nodes_to_update) {
        if (node != node_namespace + "/costmap_filter_info_server") {
            success = set_parameter_for_node(node, map_param);
        } else {
            success = set_parameter_for_node(node, mask_param);
        }
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set parameter for node %s", node.c_str());
        }
    }
}

void MasterAsyncService::run_navigation_async(std::string worldname) {
    std::string launch_file = "nav.launch.py";
    std::string use_sim = this->get_parameter("use_sim_time").as_bool() ? "true" : "false";
    std::vector<std::string> args = {"worldname:=" + worldname, "sim:=" + use_sim, "rviz:=false"};
    launch_function_async(package_name, launch_file, args);
}

void MasterAsyncService::navigation_callback(
    const std::shared_ptr<fitrobot_interfaces::srv::Navigation::Request> request,
    std::shared_ptr<fitrobot_interfaces::srv::Navigation::Response> response) {
    RCLCPP_INFO(this->get_logger(), "navigation_callback started");
    clean_up();
    run_navigation_async(request->worldname);
    RCLCPP_INFO(this->get_logger(), "navigation_callback finished");
}

bool MasterAsyncService::check_substring(const std::string& substring) {
    auto node_names = this->get_node_names();
    for (const auto& name : node_names) {
        if (name.find(substring) != std::string::npos) {
            return true;
        }
    }
    return false;
}

void MasterAsyncService::ensure_robotstatus_slam() {
    auto nodes = this->get_node_names();
    while (check_substring("slam_toolbox") == false) {
        RCLCPP_INFO(this->get_logger(), "Waiting for check_robot_status_node to be ready...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        nodes = this->get_node_names();
    }
    auto msg = RobotStatus();
    msg.status = RobotStatus::SLAM;
    pub_->publish(msg);
    set_parameter_for_node(node_namespace + "/check_robot_status_node",
                           rclcpp::Parameter("fitrobot_status", RobotStatus::SLAM));
}

void MasterAsyncService::run_slam_async() {
    std::string launch_file = "slam.launch.py";
    std::string use_sim = this->get_parameter("use_sim_time").as_bool() ? "true" : "false";
    std::vector<std::string> args = {"sim:=" + use_sim, "rviz:=false"};
    launch_function_async(package_name, launch_file, args);
}

void MasterAsyncService::slam_callback(
    const std::shared_ptr<fitrobot_interfaces::srv::Slam::Request> request,
    std::shared_ptr<fitrobot_interfaces::srv::Slam::Response> response) {
    RCLCPP_INFO(this->get_logger(), "slam_callback started");
    clean_up();
    run_slam_async();
    ensure_robotstatus_slam();
    RCLCPP_INFO(this->get_logger(), "slam_callback finished");
}

void MasterAsyncService::remote_control_callback(
    const std::shared_ptr<fitrobot_interfaces::srv::RemoteControl::Request> request,
    std::shared_ptr<fitrobot_interfaces::srv::RemoteControl::Response> response) {
    RCLCPP_INFO(this->get_logger(), "remote_control_callback started");
    RCLCPP_INFO(this->get_logger(), "get parameters tested (temp)");
    int robot_status = get_robot_status();
    RCLCPP_INFO(this->get_logger(), "Robot status: %d", robot_status);
    bool sim_time = get_sim_time(node_namespace + "/amcl");
    RCLCPP_INFO(this->get_logger(), "sim_time: %s", sim_time ? "true" : "false");
    std::string frame_id = get_parameter_string("/turtlebot3_world/lino2/map_server", "frame_id");
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());
    bool cansleep = get_parameter_bool(node_namespace + "/check_robot_status_node", "can_sleep");
    RCLCPP_INFO(this->get_logger(), "cansleep: %s", cansleep ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "remote_control_callback finished");
}

void MasterAsyncService::terminate_slam_or_navigation_callback(
    const std::shared_ptr<fitrobot_interfaces::srv::TerminateProcess::Request> request,
    std::shared_ptr<fitrobot_interfaces::srv::TerminateProcess::Response> response) {
    RCLCPP_INFO(this->get_logger(), "terminate_slam_or_navigation_callback started");
    clean_up();
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "terminate_slam_or_navigation_callback finished");
}

void MasterAsyncService::subscription_count_callback(
    const std::shared_ptr<fitrobot_interfaces::srv::SubscriptionCount::Request> request,
    std::shared_ptr<fitrobot_interfaces::srv::SubscriptionCount::Response> response) {
    RCLCPP_INFO(this->get_logger(), "subscription_count_callback started");
    std::string worldname = request->topic_name;
    set_parameters_map_mask(worldname);
    RCLCPP_INFO(this->get_logger(), "subscription_count_callback finished");
}

void MasterAsyncService::goalResponseCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result.");
    }
}

void MasterAsyncService::feedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback) {
    // std::lock_guard<std::mutex> lock(feedback_mutex_);
    rclcpp::Time now = this->now();
    if ((now - last_feedback_time_).seconds() >= 1.0) {
        if (current_waypoint != (int)feedback->current_waypoint) {
            int idx = feedback->current_waypoint;
            RCLCPP_INFO(this->get_logger(),
                        "Heading to waypoint %u. target_station: %s. (x,y,z,w)=(%g, %g, %g, %g)",
                        idx, target_station.name.c_str(), target_station.x, target_station.y,
                        target_station.z, target_station.w);
            {
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                current_waypoint = feedback->current_waypoint;
            }
            {
                std::lock_guard<std::mutex> lc(waypoints_mutex_);
                target_station = waypoints_[idx];
                station_pub_->publish(target_station);
            }
        }
        last_feedback_time_ = now;
    }
}

void MasterAsyncService::resultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult&
        result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Successfully followed all waypoints.");
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal was canceled.");
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
        break;
    }
    RCLCPP_INFO(this->get_logger(), "ResultCallback done");
    {
        std::lock_guard<std::mutex> l1(queue_mutex_);
        waypoint_queue_.pop_front();
    }
    {
        std::lock_guard<std::mutex> l2(feedback_mutex_);
        current_waypoint = -1;
    }
    // rclcpp::Rate rate(1);
    // while (rclcpp::ok()) {
    //     RCLCPP_INFO(this->get_logger(), "clean_up: ready to shutdown.");
    //     int robot_status = get_robot_status();
    //     if (robot_status == RobotStatus::NAV_WF_COMPLETED ||
    //         robot_status == RobotStatus::NAV_WF_CANCEL ||
    //         robot_status == RobotStatus::NAV_WF_FAILED) {
    //         break;
    //     }
    //     rate.sleep();
    // }
    {
        std::unique_lock<std::mutex> l3(canconsume_mutex_);
        can_consume_queue_ = true;
        canconsume_cv_.notify_one();
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    const char* robot_info = std::getenv("ROBOT_INFO");
    std::string node_name = "master_service";
    rclcpp::NodeOptions options;
    if (!robot_info) {
        std::cout << "Environment variable ROBOT_INFO is not set! Use default namespace=/";
    } else {
        auto [robot_type, robot_sn, node_namespace] = parse_robot_info(robot_info);
        options.arguments(
            {"--ros-args", "-r", "__node:=" + node_name, "-r", "__ns:=" + node_namespace});
    }
    auto node = std::make_shared<MasterAsyncService>(node_name, options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();

    return 0;
}
