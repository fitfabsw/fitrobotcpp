#include "fitrobot_interfaces/msg/robot_status.hpp"
#include "fitrobot_interfaces/srv/navigation.hpp"
#include "fitrobot_interfaces/srv/remote_control.hpp"
#include "fitrobot_interfaces/srv/slam.hpp"
#include "fitrobot_interfaces/srv/subscription_count.hpp"
#include "fitrobot_interfaces/srv/terminate_process.hpp"
#include "fitrobot_interfaces/srv/trigger.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <memory>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <sys/wait.h>
#include <tuple> // Include for std::tuple
#include <unordered_map>

using fitrobot_interfaces::msg::RobotStatus;
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
        this->declare_parameter("enable_sleep", false);
        this->declare_parameter("timeout_to_sleep", 10);
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

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        pub_ = this->create_publisher<RobotStatus>("robot_status", qos_pub);

        service_cbg_MU = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_cbg_RE = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

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
            rmw_qos_profile_services_default, service_cbg_MU);

        lfm_nav_client = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
            node_namespace + "/lifecycle_manager_navigation/manage_nodes",
            rmw_qos_profile_services_default, service_cbg_MU);

        nav_to_pose_client_ =
            rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goalpose", rclcpp::SystemDefaultsQoS(),
            std::bind(&MasterAsyncService::onGoalPoseReceived, this, std::placeholders::_1));
        // nav_thru_poses_client_ =
        //     rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
        follow_waypoints_client_ =
            rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
        goals_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "goalposes", rclcpp::SystemDefaultsQoS(),
            std::bind(&MasterAsyncService::onGoalPoseArrayReceived, this, std::placeholders::_1));

        bt_navigator_getstate_client = this->create_client<lifecycle_msgs::srv::GetState>(
            node_namespace + "/bt_navigator/get_state", rmw_qos_profile_services_default,
            service_cbg_MU);

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
    }

  private:
    std::string workspace;
    std::string robot_type;
    std::string robot_sn;
    std::string node_namespace;
    std::string package_name;
    bool launch_service_active;
    pid_t launch_service_pid; // PID of the launch service process
    rclcpp::SubscriptionOptions sub_options;
    rclcpp::Publisher<fitrobot_interfaces::msg::RobotStatus>::SharedPtr pub_;
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

    // rclcpp_action::Server<NavigateToPose>::SharedPtr nav_to_pose_service_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    // rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_thru_poses_client_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goals_sub;

    std::unordered_map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> param_clients_;
    std::shared_ptr<rclcpp::AsyncParametersClient> param_client_robot_status;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr
        lfm_nav_client; // lifecycle_manager_navigation
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr bt_navigator_getstate_client;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

    bool is_bt_navigator_active() {
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
            RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for GetState service.");
        }
        return false;
    }

    void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        // startup all the lifecycle nodes
        startup_nav(lfm_nav_client);
        // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        if (!is_bt_navigator_active()) {
            RCLCPP_INFO(this->get_logger(),
                        "bt_navigator is not active, proceeding to navigation.");
            return;
        }
        // send goal to navigate_to_pose action server
        NavigateToPose::Goal goal;
        goal.pose = *pose;
        nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5));
        nav_to_pose_client_->async_send_goal(goal);
    }

    void onGoalPoseArrayReceived(const geometry_msgs::msg::PoseArray::SharedPtr poses) {
        // startup all the lifecycle nodes
        startup_nav(lfm_nav_client);
        // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        if (!is_bt_navigator_active()) {
            RCLCPP_INFO(this->get_logger(),
                        "bt_navigator is not active, proceeding to navigation.");
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

    void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        auto sta_msg = RobotStatus();
        sta_msg.status = RobotStatus::NAV_PREPARE_TO_READY;
        pub_->publish(sta_msg);
        set_parameter_for_node(
            node_namespace + "/check_robot_status_node",
            rclcpp::Parameter("fitrobot_status", RobotStatus::NAV_PREPARE_TO_READY));
    }

    void shutdown_launch_service() {
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

    void clean_up(bool ensure_bringup = true) {
        RCLCPP_INFO(this->get_logger(), "clean_up: ready to shutdown.");
        shutdown_launch_service();
        if (ensure_bringup) {
            while (1) {
                int robot_status = get_robot_status();
                if (robot_status == 1) {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            RCLCPP_INFO(this->get_logger(), "clean_up: done. robot_status back to BRINGUP");
        }
    }

    void launch_function_async(const std::string& package_name, const std::string& launch_file,
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
            RCLCPP_DEBUG(this->get_logger(),
                         "Successfully launched %s with launch file %s. PID: %d",
                         package_name.c_str(), launch_file.c_str(), pid);
        }
    }

    // 用於終止進程組的函數
    void terminate_process_group(pid_t pid) {
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

    void
    lifecycle_manage_cmd(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client,
                         int cmd) {
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
                RCLCPP_INFO(this->get_logger(),
                            "ManageLifecycleNodes service call succeeded. cmd=%d", cmd);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Timeout while waiting for ManageLifecycleNodes service to complete.");
        }
    }

    void startup_nav(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client) {
        lifecycle_manage_cmd(client, 0);
    }

    void pause_nav(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client) {
        lifecycle_manage_cmd(client, 1);
    }

    void resume_nav(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client) {
        lifecycle_manage_cmd(client, 2);
    }

    void reset_nav(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client) {
        lifecycle_manage_cmd(client, 3);
    }

    void lifecycle_nav_callback(
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

    void set_robot_info_from_env() {
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

    int get_robot_status() {
        return get_parameter_int(node_namespace + "/check_robot_status_node", "fitrobot_status");
    }

    bool get_sim_time(const std::string& node_nmae) {
        return get_parameter_bool(node_nmae, "use_sim_time");
    }

    int get_parameter_int(const std::string& node_name, const std::string& param_name) {
        rclcpp::Parameter param_value;
        if (get_parameter_for_node(node_name, param_name, param_value)) {
            if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                return param_value.as_int();
            } else {
                RCLCPP_ERROR(this->get_logger(), "%s parameter is not of type integer.",
                             param_name.c_str());
            }
        }
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter %s from node %s",
                     param_name.c_str(), node_name.c_str());
        return -1; // Return a default or error value if the parameter is not found or not an
                   // integer
    }

    bool get_parameter_bool(const std::string& node_name, const std::string& param_name) {
        rclcpp::Parameter param_value;
        if (get_parameter_for_node(node_name, param_name, param_value)) {
            if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                return param_value.as_bool();
            } else {
                RCLCPP_ERROR(this->get_logger(), "%s parameter is not of type bool.",
                             param_name.c_str());
            }
        }
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter %s from node %s",
                     param_name.c_str(), node_name.c_str());
        return false; // Return a default or error value if the parameter is not found or not an
                      // integer
    }

    std::string get_parameter_string(const std::string& node_name, const std::string& param_name) {
        rclcpp::Parameter param_value;
        if (get_parameter_for_node(node_name, param_name, param_value)) {
            if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                return param_value.as_string();
            } else {
                RCLCPP_ERROR(this->get_logger(), "%s parameter is not of type string.",
                             param_name.c_str());
            }
        }
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameter %s from node %s",
                     param_name.c_str(), node_name.c_str());
        return ""; // Return a default or error value if the parameter is not found or not an
                   // integer
    }

    bool get_parameter_for_node(const std::string& node_name, const std::string& param_name,
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

    bool set_parameter_for_node(const std::string& node_name,
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

    void set_parameters_map_mask(std::string worldname) {
        std::string param_name = "map_topic";
        std::string param_value = "/" + worldname + "/" + robot_type + "/map";
        std::string param_name2 = "mask_topic";
        std::string param_value2 = "/" + worldname + "/" + robot_type + "/keepout_filter_mask";
        std::vector<std::string> nodes_to_update = {
            node_namespace + "/amcl", node_namespace + "/local_costmap/local_costmap",
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
                RCLCPP_ERROR(this->get_logger(), "Failed to set parameter for node %s",
                             node.c_str());
            }
        }
    }

    void run_navigation_async(std::string worldname) {
        std::string launch_file = "nav.launch.py";
        std::string use_sim = this->get_parameter("use_sim_time").as_bool() ? "true" : "false";
        std::vector<std::string> args = {"worldname:=" + worldname, "sim:=" + use_sim,
                                         "rviz:=false"};
        launch_function_async(package_name, launch_file, args);
    }

    void navigation_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::Navigation::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::Navigation::Response> response) {
        RCLCPP_INFO(this->get_logger(), "navigation_callback started");
        clean_up();
        run_navigation_async(request->worldname);
        RCLCPP_INFO(this->get_logger(), "navigation_callback finished");
    }

    bool check_substring(const std::string& substring) {
        auto node_names = this->get_node_names();
        for (const auto& name : node_names) {
            if (name.find(substring) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    void ensure_robotstatus_slam() {
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

    void run_slam_async() {
        std::string launch_file = "slam.launch.py";
        std::string use_sim = this->get_parameter("use_sim_time").as_bool() ? "true" : "false";
        std::vector<std::string> args = {"sim:=" + use_sim, "rviz:=false"};
        launch_function_async(package_name, launch_file, args);
    }

    void slam_callback(const std::shared_ptr<fitrobot_interfaces::srv::Slam::Request> request,
                       std::shared_ptr<fitrobot_interfaces::srv::Slam::Response> response) {
        RCLCPP_INFO(this->get_logger(), "slam_callback started");
        clean_up();
        run_slam_async();
        ensure_robotstatus_slam();
        RCLCPP_INFO(this->get_logger(), "slam_callback finished");
    }

    void remote_control_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::RemoteControl::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::RemoteControl::Response> response) {
        RCLCPP_INFO(this->get_logger(), "remote_control_callback started");
        RCLCPP_INFO(this->get_logger(), "get parameters tested (temp)");
        int robot_status = get_robot_status();
        RCLCPP_INFO(this->get_logger(), "Robot status: %d", robot_status);
        bool sim_time = get_sim_time(node_namespace + "/amcl");
        RCLCPP_INFO(this->get_logger(), "sim_time: %s", sim_time ? "true" : "false");
        std::string frame_id =
            get_parameter_string("/turtlebot3_world/lino2/map_server", "frame_id");
        RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "remote_control_callback finished");
    }

    void terminate_slam_or_navigation_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::TerminateProcess::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::TerminateProcess::Response> response) {
        RCLCPP_INFO(this->get_logger(), "terminate_slam_or_navigation_callback started");
        clean_up();
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "terminate_slam_or_navigation_callback finished");
    }

    void subscription_count_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::SubscriptionCount::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::SubscriptionCount::Response> response) {
        RCLCPP_INFO(this->get_logger(), "subscription_count_callback started");
        std::string worldname = request->topic_name;
        set_parameters_map_mask(worldname);
        RCLCPP_INFO(this->get_logger(), "subscription_count_callback finished");
    }
};

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

    // auto node = std::make_shared<MasterAsyncService>();
    // rclcpp::spin(node);
    // rclcpp::shutdown();

    return 0;
}
