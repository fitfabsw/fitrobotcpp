#include "fitrobot_interfaces/msg/robot_status.hpp"
#include "fitrobot_interfaces/srv/para1.hpp"
#include "fitrobot_interfaces/srv/trigger.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2_ros/create_timer_ros.h"
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <arpa/inet.h>
#include <chrono>
#include <ifaddrs.h>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <memory>
#include <netdb.h>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tuple> // Include for std::tuple

/*
GoalStatus
-------------------
0 STATUS_UNKNOWN
1 STATUS_ACCEPTED
2 STATUS_EXECUTING
3 STATUS_CANCELING
4 STATUS_SUCCEEDED
5 STATUS_CANCELED
6 STATUS_ABORTED
*/
/*
fitrobot_interfaces/msg/RobotStatus.msg
-----------------------------
int8 STANDBY = 0
int8 BRINGUP = 1
int8 SLAM = 2
int8 NAV_PREPARE = 3

int8 NAV_READY = 10
int8 NAV_RUNNING = 11
int8 NAV_ARRIVED = 12
int8 NAV_CANCEL = 13
int8 NAV_FAILED = 14

int8 NAV_WF_RUNNING = 21
int8 NAV_WF_ARRIVED = 22
int8 NAV_WF_COMPLETED = 23
int8 NAV_WF_CANCEL = 24
int8 NAV_WF_FAILED = 25
*/

using namespace std::chrono_literals;
using action_msgs::msg::GoalStatus;
using action_msgs::msg::GoalStatusArray;
using fitrobot_interfaces::msg::RobotStatus;
using fitrobot_interfaces::srv::Para1;
using fitrobot_interfaces::srv::Trigger;
using lifecycle_msgs::msg::Transition;
using lifecycle_msgs::srv::ChangeState;
using rclcpp::Parameter;
using std::string;
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

class RobotStatusCheckNode : public rclcpp::Node {
  public:
    RobotStatusCheckNode() : Node("check_robot_status_node") {

        set_robot_info_from_env();

        this->declare_parameter("fitrobot_status", RobotStatus::STANDBY);
        this->declare_parameter("enable_sleep", false);
        this->declare_parameter("timeout_to_sleep", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        qos.reliable();
        auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        pub_ = this->create_publisher<RobotStatus>("robot_status", qos_pub);

        service_cbg_MU = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_cbg_RE = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        timer_ = this->create_wall_timer(1s, std::bind(&RobotStatusCheckNode::status_check, this));

        service_ = this->create_service<std_srvs::srv::Trigger>(
            "is_localized", std::bind(&RobotStatusCheckNode::srv_localized_callback, this, _1, _2));
        // rmw_qos_profile_services_default, service_cbg_MU);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = service_cbg_MU;
        sub_nav_to_pose_ = this->create_subscription<GoalStatusArray>(
            "navigate_to_pose/_action/status", 10,
            std::bind(&RobotStatusCheckNode::navigate_to_pose_goal_status_callback, this, _1));
        // sub_options);
        sub_follow_wp_ = this->create_subscription<GoalStatusArray>(
            "follow_waypoints/_action/status", 10,
            std::bind(&RobotStatusCheckNode::follower_waypoints_status_callback, this, _1));
        // sub_options);

        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&RobotStatusCheckNode::parametersCallback, this, _1));

        register_robot_client = this->create_client<Para1>(
            "/fitparam", rmw_qos_profile_services_default, service_cbg_MU);

        lifecycle_nav_client = this->create_client<fitrobot_interfaces::srv::Trigger>(
            "lifecycle_nav", rmw_qos_profile_services_default, service_cbg_MU);

        client_ =
            this->create_client<lifecycle_msgs::srv::ChangeState>("bt_navigator/change_state");

        RCLCPP_INFO(this->get_logger(), "Node initialized: STANDBY");

        nav_statuses = {RobotStatus::NAV_PREPARE_TO_READY, RobotStatus::NAV_READY,
                        RobotStatus::NAV_RUNNING, // Uncomment if needed
                        RobotStatus::NAV_ARRIVED,          RobotStatus::NAV_CANCEL,
                        RobotStatus::NAV_FAILED,
                        RobotStatus::NAV_WF_RUNNING, // Uncomment if needed
                        RobotStatus::NAV_WF_ARRIVED,       RobotStatus::NAV_WF_COMPLETED,
                        RobotStatus::NAV_WF_CANCEL,        RobotStatus::NAV_WF_FAILED};
        cansleep_statuses = {RobotStatus::NAV_READY,      RobotStatus::NAV_ARRIVED,
                             RobotStatus::NAV_CANCEL,     RobotStatus::NAV_FAILED,
                             RobotStatus::NAV_WF_ARRIVED, RobotStatus::NAV_WF_COMPLETED,
                             RobotStatus::NAV_WF_CANCEL,  RobotStatus::NAV_WF_FAILED};
        running_statuses = {RobotStatus::NAV_RUNNING, RobotStatus::NAV_WF_RUNNING};
        is_deactivated_ = false;
    }

  private:
    rcl_interfaces::msg::SetParametersResult
    parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
        int robot_status = this->get_parameter("fitrobot_status").as_int();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto& param : parameters) {
            if (param.get_name() != "fitrobot_status") {
                continue;
            }
            if (robot_status == RobotStatus::BRINGUP && param.as_int() == RobotStatus::SLAM) {
                RCLCPP_INFO(this->get_logger(), "%s", "SLAM");
                update_robot(RobotStatus::SLAM);
            } else if (robot_status == RobotStatus::NAV_PREPARE &&
                       param.as_int() == RobotStatus::NAV_PREPARE_TO_READY) {
                RCLCPP_INFO(this->get_logger(), "%s", "NAV_PREPARE_TO_READY");
            }
        }
        return result;
    }

    void navigate_to_pose_goal_status_callback(const GoalStatusArray::SharedPtr msg) {
        auto status = msg->status_list.back().status;
        int fitrobot_status;
        std::string status_log;
        auto set_and_log_status = [&](int new_status, const std::string& status_log) {
            publish_status(new_status);
            RCLCPP_INFO(this->get_logger(), "%s", status_log.c_str());
            this->set_parameter(Parameter("fitrobot_status", new_status));
            update_robot(new_status);
        };
        switch (status) {
        case GoalStatus::STATUS_EXECUTING:
            // lifecycle_node_cmd(client_, Transition::TRANSITION_DEACTIVATE);
            // lifecycle_nav("startup");
            set_and_log_status(waypoints_following_ ? RobotStatus::NAV_WF_RUNNING
                                                    : RobotStatus::NAV_RUNNING,
                               waypoints_following_ ? "NAV_WF_RUNNING" : "NAV_RUNNING");
            break;
        case GoalStatus::STATUS_SUCCEEDED:
            set_and_log_status(waypoints_following_ ? RobotStatus::NAV_WF_ARRIVED
                                                    : RobotStatus::NAV_ARRIVED,
                               waypoints_following_ ? "NAV_WF_ARRIVED" : "NAV_ARRIVED");
            break;
        case GoalStatus::STATUS_CANCELED:
            if (!waypoints_following_) {
                set_and_log_status(RobotStatus::NAV_CANCEL, "NAV_CANCEL");
            }
            break;
        case GoalStatus::STATUS_ABORTED:
            if (!waypoints_following_) {
                set_and_log_status(RobotStatus::NAV_FAILED, "NAV_FAILED");
            }
            break;
        default:
            break;
        }
    }

    void follower_waypoints_status_callback(const GoalStatusArray::SharedPtr msg) {
        auto status = msg->status_list.back().status;
        auto update_status = [&](int new_status, const std::string& log) {
            waypoints_following_ = (status == GoalStatus::STATUS_EXECUTING);
            publish_status(new_status);
            RCLCPP_INFO(this->get_logger(), "%s", log.c_str());
            this->set_parameter(Parameter("fitrobot_status", new_status));
            update_robot(new_status);
        };
        switch (status) {
        case GoalStatus::STATUS_EXECUTING:
            waypoints_following_ = true; // 更新為執行狀態，但不需做進一步處理
            break;
        case GoalStatus::STATUS_SUCCEEDED:
            update_status(RobotStatus::NAV_WF_COMPLETED, "NAV_WF_COMPLETED");
            break;
        case GoalStatus::STATUS_CANCELED:
            update_status(RobotStatus::NAV_WF_CANCEL, "NAV_WF_CANCEL");
            break;
        case GoalStatus::STATUS_ABORTED:
            update_status(RobotStatus::NAV_WF_FAILED, "NAV_WF_FAILED");
            break;
        default:
            break;
        }
    }

    void srv_localized_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = is_localized_;
        response->message = "is_localized: " + std::to_string(is_localized_);
    }

    bool check_tf(const std::string& to_frame, const std::string& from_frame,
                  double timeout = 1.0) {
        bool canTransform = tf_buffer_->canTransform(to_frame, from_frame, rclcpp::Time(0),
                                                     tf2::durationFromSec(timeout));
        tf_buffer_->clear();
        return canTransform;
    }

    bool is_tf_odom_baselink_existed() { return check_tf("odom", "base_link", 0.9); }

    bool is_tf_odom_map_existed() { return check_tf("map", "odom", 0.1); }

    bool check_substring(const std::string& substring) {
        auto node_names = this->get_node_names();
        for (const auto& name : node_names) {
            if (name.find(substring) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    bool check_nav2_running() { return check_substring(robot_namespace + "/bt_navigator"); }

    bool check_slam_running() { return check_substring(robot_namespace + "/slam_toolbox"); }

    void publish_status(int status) {
        auto msg = RobotStatus();
        msg.status = status;
        pub_->publish(msg);
    }

    std::string get_ip_address() {
        struct ifaddrs *ifaddr, *ifa;
        char host[NI_MAXHOST];
        std::string ip_address;
        if (getifaddrs(&ifaddr) == -1) {
            std::cerr << "Error getting IP addresses" << std::endl;
            return "";
        }
        for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == nullptr)
                continue;
            if (ifa->ifa_addr->sa_family == AF_INET) {
                int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST,
                                    nullptr, 0, NI_NUMERICHOST);
                if (s != 0) {
                    std::cerr << "getnameinfo() failed: " << gai_strerror(s) << std::endl;
                    continue;
                }
                if (std::string(host) != "127.0.0.1") {
                    ip_address = host;
                    break; // 找到一個非回送的 IPv4 地址後退出
                }
            }
        }
        freeifaddrs(ifaddr); // 釋放記憶體
        return ip_address;
    }

    void lifecycle_manage_cmd(std::string cmd) {
        if (!lifecycle_nav_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "lifecycle_manage_cmd service not available");
            return;
        }
        auto request = std::make_shared<fitrobot_interfaces::srv::Trigger::Request>();
        request->trigger_name = cmd;
        auto future = lifecycle_nav_client->async_send_request(request);
        auto status = future.wait_for(std::chrono::milliseconds(5000));
        if (status == std::future_status::ready) {
            try {
                auto result = future.get();
                if (result->success == "true") {
                    RCLCPP_INFO(this->get_logger(), "lifecycle_manage_cmd success!");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "lifecycle_manage_cmd failed!");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Timeout while waiting for the parameter set operation to complete.");
        }
    }

    void lifecycle_node_cmd(rclcpp::Client<ChangeState>::SharedPtr& client, int transition_cmd) {
        auto request = std::make_shared<ChangeState::Request>();
        request->transition.id = transition_cmd;
        // request->transition.label = cmd;
        auto future = client->async_send_request(request);
        auto status = future.wait_for(std::chrono::milliseconds(3000));
        if (status == std::future_status::ready) {
            try {
                auto result = future.get();
                RCLCPP_INFO(this->get_logger(), "lifecycle_node_cmd success!");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Timeout while waiting for the parameter set operation to complete.");
        }
    }

    void lifecycle_nav(std::string cmd) {
        if (!lifecycle_nav_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "lifecycle_nav service not available");
            return;
        }
        auto request = std::make_shared<fitrobot_interfaces::srv::Trigger::Request>();
        request->trigger_name = cmd;
        auto future = lifecycle_nav_client->async_send_request(request);
        auto status = future.wait_for(std::chrono::milliseconds(5000));
        if (status == std::future_status::ready) {
            try {
                auto result = future.get();
                if (result->success == "true") {
                    RCLCPP_INFO(this->get_logger(), "lifecycle_nav success!");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "lifecycle_nav failed!");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Timeout while waiting for the parameter set operation to complete.");
        }
    }

    void register_robot(int robot_status) {
        if (!register_robot_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "register_robot service not available");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Register robot!");
        auto request = std::make_shared<Para1::Request>();
        request->parameter1_name = "register_robot";
        ip = get_ip_address();
        RCLCPP_INFO(this->get_logger(), "IP: %s", ip.c_str());
        request->parameter1_value = "{'robot_namespace': '" + robot_namespace +
                                    "', 'robot_status': " + std::to_string(robot_status) +
                                    ", 'robot_ip': '" + ip + "'}";
        auto future = register_robot_client->async_send_request(request);
        auto status = future.wait_for(std::chrono::milliseconds(5000));
        if (status == std::future_status::ready) {
            try {
                auto result = future.get();
                if (result->success == "true") {
                    RCLCPP_INFO(this->get_logger(), "Register robot success!");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Register robot failed!");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Timeout while waiting for the parameter set operation to complete.");
        }
    }

    void update_robot(int robot_status) {
        if (!register_robot_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "update_robot service not available");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Update robot!");
        auto request = std::make_shared<Para1::Request>();
        request->parameter1_name = "update_robot";
        request->parameter1_value = "{'robot_namespace': '" + robot_namespace +
                                    "', 'robot_status': " + std::to_string(robot_status) + "}";
        auto future = register_robot_client->async_send_request(request);
        auto status = future.wait_for(std::chrono::milliseconds(5000));
        if (status == std::future_status::ready) {
            try {
                auto result = future.get();
                if (result->success == "true") {
                    RCLCPP_INFO(this->get_logger(), "Update robot success!");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Update robot failed!");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Timeout while waiting for the parameter set operation to complete.");
        }
    }

    void status_check() {
        try {
            int robot_status = this->get_parameter("fitrobot_status").as_int();
            if (!is_deactivated_) {
                if (std::find(cansleep_statuses.begin(), cansleep_statuses.end(), robot_status) !=
                    cansleep_statuses.end()) {
                    // pause all the nav2 lifecycle nodes
                    RCLCPP_INFO(this->get_logger(), "watch for sleep");
                    if (this->get_parameter("enable_sleep").as_bool()) {
                        RCLCPP_INFO(this->get_logger(), "Sleeping...");
                        lifecycle_manage_cmd("pause");
                        is_deactivated_ = true;
                    }
                }
            } else if (std::find(running_statuses.begin(), running_statuses.end(), robot_status) !=
                       running_statuses.end()) {
                is_deactivated_ = false;
                RCLCPP_INFO(this->get_logger(), "Back to running. Set is_deactivated_ to false");
            }

            if (robot_status == RobotStatus::STANDBY) {
                if (is_tf_odom_baselink_existed()) {
                    RCLCPP_INFO(this->get_logger(), "BRINGUP");
                    publish_status(RobotStatus::BRINGUP);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::BRINGUP));
                    register_robot(RobotStatus::BRINGUP);
                }
            } else if (robot_status == RobotStatus::BRINGUP) {
                if (check_nav2_running()) {
                    RCLCPP_INFO(this->get_logger(), "NAV_PREPARE");
                    publish_status(RobotStatus::NAV_PREPARE);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::NAV_PREPARE));
                    update_robot(RobotStatus::NAV_PREPARE);
                } else if (!is_tf_odom_baselink_existed()) {
                    RCLCPP_INFO(this->get_logger(), "STANDBY");
                    publish_status(RobotStatus::STANDBY);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::STANDBY));
                }
            } else if (robot_status == RobotStatus::NAV_PREPARE) {
                if (!check_nav2_running()) {
                    RCLCPP_INFO(this->get_logger(), "BRINGUP");
                    publish_status(RobotStatus::BRINGUP);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::BRINGUP));
                    update_robot(RobotStatus::BRINGUP);
                }
            } else if (robot_status == RobotStatus::NAV_PREPARE_TO_READY) {
                if (is_tf_odom_map_existed()) {
                    RCLCPP_INFO(this->get_logger(), "NAV_STANDBY");
                    publish_status(RobotStatus::NAV_READY);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::NAV_READY));
                    update_robot(RobotStatus::NAV_READY);
                    is_localized_ = true;
                } else {
                    RCLCPP_INFO(this->get_logger(), "NAV_PREPARE");
                    publish_status(RobotStatus::NAV_PREPARE);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::NAV_PREPARE));
                    update_robot(RobotStatus::NAV_PREPARE);
                    is_localized_ = false;
                }
            } else if (robot_status == RobotStatus::SLAM) {
                if (!check_slam_running()) {
                    RCLCPP_INFO(this->get_logger(), "BRINGUP");
                    publish_status(RobotStatus::BRINGUP);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::BRINGUP));
                    update_robot(RobotStatus::BRINGUP);
                }
            } else if (std::find(nav_statuses.begin(), nav_statuses.end(), robot_status) !=
                       nav_statuses.end()) {
                if (!is_tf_odom_map_existed()) {
                    RCLCPP_INFO(this->get_logger(), "NAV_PREPARE");
                    publish_status(RobotStatus::NAV_PREPARE);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::NAV_PREPARE));
                    update_robot(RobotStatus::NAV_PREPARE);
                    is_localized_ = false;
                }
            }
        } catch (tf2::LookupException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
        } catch (tf2::ExtrapolationException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform extrapolation failed: %s", ex.what());
        }
    }

    void set_robot_info_from_env() {
        const char* robot_info = std::getenv("ROBOT_INFO");
        if (!robot_info) {
            RCLCPP_WARN(this->get_logger(),
                        "Environment variable ROBOT_INFO is not set! Use default namespace=/");
            robot_namespace = "";
        } else {
            std::tie(robot_type, robot_sn, robot_namespace) = parse_robot_info(robot_info);
        }
        RCLCPP_INFO(this->get_logger(), "Node name set to: %s", robot_namespace.c_str());
    }

    std::string robot_type;
    std::string robot_sn;
    std::string robot_namespace;
    // bool use_sim;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<fitrobot_interfaces::msg::RobotStatus>::SharedPtr pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    rclcpp::Client<Para1>::SharedPtr register_robot_client;
    rclcpp::Client<fitrobot_interfaces::srv::Trigger>::SharedPtr lifecycle_nav_client;
    rclcpp::Client<ChangeState>::SharedPtr client_;

    rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_nav_to_pose_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_follow_wp_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool is_localized_;
    bool waypoints_following_;
    std::unordered_set<int> nav_statuses;
    std::vector<int> cansleep_statuses;
    std::vector<int> running_statuses;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::CallbackGroup::SharedPtr service_cbg_MU;
    rclcpp::CallbackGroup::SharedPtr service_cbg_RE;
    std::string ip;
    bool is_deactivated_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStatusCheckNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();

    // rclcpp::spin(node);
    // rclcpp::shutdown();
    return 0;
}
