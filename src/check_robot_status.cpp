#include "fitrobot_interfaces/msg/robot_status.hpp"
#include "std_msgs/msg/int32.hpp"
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
using rclcpp::Parameter;
using std::string;
using std::placeholders::_1;
using std::placeholders::_2;

class RobotStatusCheckNode : public rclcpp::Node {
  public:
    RobotStatusCheckNode()
        : Node("check_robot_status_node"), is_localized_(false), waypoints_following_(false) {
        auto node_logger = this->get_logger();

        this->declare_parameter("fitrobot_status", RobotStatus::STANDBY);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        qos.reliable();

        pub_ = this->create_publisher<RobotStatus>("robot_status", qos);

        timer_ = this->create_wall_timer(1s, std::bind(&RobotStatusCheckNode::status_check, this));

        service_ = this->create_service<std_srvs::srv::Trigger>(
            "is_localized", std::bind(&RobotStatusCheckNode::srv_localized_callback, this, _1, _2));

        sub_nav_to_pose_ = this->create_subscription<GoalStatusArray>(
            "navigate_to_pose/_action/status", 10,
            std::bind(&RobotStatusCheckNode::navigate_to_pose_goal_status_callback, this, _1));
        sub_follow_wp_ = this->create_subscription<GoalStatusArray>(
            "follow_waypoints/_action/status", 10,
            std::bind(&RobotStatusCheckNode::follower_waypoints_status_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Node initialized: standby");

        nav_statuses = {RobotStatus::NAV_READY,
                        // RobotStatus::NAV_RUNNING, // Uncomment if needed
                        RobotStatus::NAV_ARRIVED, RobotStatus::NAV_CANCEL, RobotStatus::NAV_FAILED,
                        // RobotStatus::NAV_WF_RUNNING, // Uncomment if needed
                        RobotStatus::NAV_WF_ARRIVED, RobotStatus::NAV_WF_COMPLETED,
                        RobotStatus::NAV_WF_CANCEL, RobotStatus::NAV_WF_FAILED};
    }

  private:
    void navigate_to_pose_goal_status_callback(const GoalStatusArray::SharedPtr msg) {
        auto status = msg->status_list.back().status;
        int fitrobot_status;
        string status_log;
        if (status == GoalStatus::STATUS_EXECUTING) {
            if (!waypoints_following_) {
                fitrobot_status = RobotStatus::NAV_RUNNING;
                status_log = "NAV_RUNNING";
            } else {
                fitrobot_status = RobotStatus::NAV_WF_RUNNING;
                status_log = "NAV_WF_RUNNING";
            }
            publish_status(fitrobot_status);
            RCLCPP_INFO(this->get_logger(), "%s", status_log.c_str());
            this->set_parameter(Parameter("fitrobot_status", fitrobot_status));

        } else if (status == GoalStatus::STATUS_SUCCEEDED) {
            if (!waypoints_following_) {
                fitrobot_status = RobotStatus::NAV_ARRIVED;
                status_log = "NAV_ARRIVED";
            } else {
                fitrobot_status = RobotStatus::NAV_WF_ARRIVED;
                status_log = "NAV_WF_ARRIVED";
            }
            publish_status(fitrobot_status);
            RCLCPP_INFO(this->get_logger(), "%s", status_log.c_str());
            this->set_parameter(Parameter("fitrobot_status", fitrobot_status));

        } else if (status == GoalStatus::STATUS_CANCELED) {
            if (!waypoints_following_) {
                fitrobot_status = RobotStatus::NAV_CANCEL;
                publish_status(fitrobot_status);
                RCLCPP_INFO(this->get_logger(), "NAV_CANCEL");
                this->set_parameter(Parameter("fitrobot_status", fitrobot_status));
            }

        } else if (status == GoalStatus::STATUS_ABORTED) {
            if (!waypoints_following_) {
                fitrobot_status = RobotStatus::NAV_FAILED;
                publish_status(fitrobot_status);
                RCLCPP_INFO(this->get_logger(), "NAV_FAILED");
                this->set_parameter(Parameter("fitrobot_status", fitrobot_status));
            }
        }
    }

    void follower_waypoints_status_callback(const GoalStatusArray::SharedPtr msg) {
        auto status = msg->status_list.back().status;
        int fitrobot_status;
        string status_log;
        if (status == GoalStatus::STATUS_EXECUTING) {
            waypoints_following_ = true;

        } else if (status == GoalStatus::STATUS_SUCCEEDED) {
            waypoints_following_ = false;
            fitrobot_status = RobotStatus::NAV_WF_COMPLETED;
            publish_status(fitrobot_status);
            RCLCPP_INFO(this->get_logger(), "NAV_WF_COMPLETED");
            this->set_parameter(Parameter("fitrobot_status", fitrobot_status));

        } else if (status == GoalStatus::STATUS_CANCELED) {
            waypoints_following_ = false;
            fitrobot_status = RobotStatus::NAV_WF_CANCEL;
            publish_status(fitrobot_status);
            RCLCPP_INFO(this->get_logger(), "NAV_WF_CANCEL");
            this->set_parameter(Parameter("fitrobot_status", fitrobot_status));

        } else if (status == GoalStatus::STATUS_ABORTED) {
            waypoints_following_ = false;
            fitrobot_status = RobotStatus::NAV_WF_FAILED;
            publish_status(fitrobot_status);
            RCLCPP_INFO(this->get_logger(), "NAV_WF_FAILED");
            this->set_parameter(Parameter("fitrobot_status", fitrobot_status));
        }
    }

    void srv_localized_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = is_localized_;
        response->message = "is_localized: " + std::to_string(is_localized_);
    }

    bool check_tf(const std::string& to_frame, const std::string& from_frame,
                  double timeout = 1.0) {
        // double timeout = 0.1) {
        // auto duration = tf2::durationFromSec(timeout);
        // bool canTransform = tf_buffer_->canTransform(to_frame, from_frame, tf2::TimePointZero,
        //                                              tf2::durationFromSec(timeout));
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

    bool check_nav2_running() { return check_substring("bt_navigator"); }

    bool check_slam_running() { return check_substring("slam_toolbox"); }

    void publish_status(int status) {
        auto msg = RobotStatus();
        msg.status = status;
        pub_->publish(msg);
    }

    void status_check() {
        try {
            int robot_status = this->get_parameter("fitrobot_status").as_int();

            if (robot_status == RobotStatus::STANDBY) {
                if (is_tf_odom_baselink_existed()) {
                    RCLCPP_INFO(this->get_logger(), "BRINGUP");
                    publish_status(RobotStatus::BRINGUP);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::BRINGUP));
                }
                return;

            } else if (robot_status == RobotStatus::BRINGUP) {
                if (check_nav2_running()) {
                    RCLCPP_INFO(this->get_logger(), "NAV_PREPARE");
                    publish_status(RobotStatus::NAV_PREPARE);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::NAV_PREPARE));
                } else if (!is_tf_odom_baselink_existed()) {
                    RCLCPP_INFO(this->get_logger(), "standby");
                    publish_status(RobotStatus::STANDBY);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::STANDBY));
                }
                return;

            } else if (robot_status == RobotStatus::NAV_PREPARE) {
                if (is_tf_odom_map_existed()) {
                    RCLCPP_INFO(this->get_logger(), "NAV_STANDBY");
                    publish_status(RobotStatus::NAV_READY);
                    is_localized_ = true;
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::NAV_READY));
                } else if (!check_nav2_running()) {
                    RCLCPP_INFO(this->get_logger(), "BRINGUP");
                    publish_status(RobotStatus::BRINGUP);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::BRINGUP));
                }
                return;
            }

            if (robot_status == RobotStatus::SLAM) {
                if (!check_slam_running()) {
                    RCLCPP_INFO(this->get_logger(), "bringup");
                    publish_status(RobotStatus::BRINGUP);
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::BRINGUP));
                }
                return;
            } else if (std::find(nav_statuses.begin(), nav_statuses.end(), robot_status) !=
                       nav_statuses.end()) {
                if (!is_tf_odom_map_existed()) {
                    RCLCPP_INFO(this->get_logger(), "NAV_PREPARE");
                    publish_status(RobotStatus::NAV_PREPARE);
                    is_localized_ = false;
                    this->set_parameter(Parameter("fitrobot_status", RobotStatus::NAV_PREPARE));
                }
                return;
            }

        } catch (tf2::LookupException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
        } catch (tf2::ExtrapolationException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform extrapolation failed: %s", ex.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<fitrobot_interfaces::msg::RobotStatus>::SharedPtr pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_nav_to_pose_;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr sub_follow_wp_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool is_localized_;
    bool waypoints_following_;
    std::vector<int> nav_statuses;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStatusCheckNode>();
    RCLCPP_INFO(node->get_logger(), "Check Robot Status Node started.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
