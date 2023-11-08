#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "fitrobot_interfaces/msg/robot_status.hpp"

using namespace std::chrono_literals;
using fitrobot_interfaces::msg::RobotStatus;
using rclcpp::Parameter;
using std::placeholders::_1;
using std::placeholders::_2;

class RobotStatusCheckNode : public rclcpp::Node {
public:
  RobotStatusCheckNode()
      : Node("check_robot_status_node"), is_localized_(false) {
    auto node_logger = this->get_logger();

    this->declare_parameter("fitrobot_status", RobotStatus::STANDBY);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();

    pub_ = this->create_publisher<RobotStatus>("robot_status", qos);
    timer_ = this->create_wall_timer(
        1s, std::bind(&RobotStatusCheckNode::status_check, this));
    service_ = this->create_service<std_srvs::srv::Trigger>(
        "is_localized",
        std::bind(&RobotStatusCheckNode::srv_localized_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Node initialized: standby");
  }

private:
  void srv_localized_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = is_localized_;
    response->message = "is_localized: " + std::to_string(is_localized_);
  }

  bool check_tf(const std::string &from_frame, const std::string &to_frame,
                double timeout = 0.5) {
    auto duration = tf2::durationFromSec(timeout);
    bool canTransform =
        tf_buffer_->canTransform(from_frame, to_frame, tf2::TimePointZero,
                                 tf2::durationFromSec(timeout));
    tf_buffer_->clear();
    return canTransform;
  }

  bool is_tf_odom_baselink_existed() {
    return check_tf("odom", "base_link", 0.9);
  }

  bool is_tf_odom_map_existed() { return check_tf("map", "odom", 0.1); }

  bool check_nav2_running() {
    auto node_names = this->get_node_names();
    // for (const auto &name : node_names) {
    //   RCLCPP_INFO(this->get_logger(), "Found node: %s", name.c_str());
    // }
    return std::find(node_names.begin(), node_names.end(), "/bt_navigator") !=
           node_names.end();
  }

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
          RCLCPP_INFO(this->get_logger(), "bringup");
          publish_status(RobotStatus::BRINGUP);
          this->set_parameter(
              Parameter("fitrobot_status", RobotStatus::BRINGUP));
        }
        return;

      } else if (robot_status == RobotStatus::BRINGUP) {
        if (check_nav2_running()) {
          RCLCPP_INFO(this->get_logger(), "nav_prepare");
          publish_status(RobotStatus::NAV_PREPARE);
          this->set_parameter(
              Parameter("fitrobot_status", RobotStatus::NAV_PREPARE));
        } else if (!is_tf_odom_baselink_existed()) {
          RCLCPP_INFO(this->get_logger(), "standby");
          publish_status(RobotStatus::STANDBY);
          this->set_parameter(
              Parameter("fitrobot_status", RobotStatus::STANDBY));
        }
        return;

      } else if (robot_status == RobotStatus::NAV_PREPARE) {
        if (is_tf_odom_map_existed()) {
          RCLCPP_INFO(this->get_logger(), "nav_ready");
          publish_status(RobotStatus::NAV_READY);
          is_localized_ = true;
          this->set_parameter(
              Parameter("fitrobot_status", RobotStatus::NAV_READY));
        } else if (!check_nav2_running()) {
          RCLCPP_INFO(this->get_logger(), "bringup");
          publish_status(RobotStatus::BRINGUP);
          this->set_parameter(
              Parameter("fitrobot_status", RobotStatus::BRINGUP));
        }
        return;

      } else if (robot_status == RobotStatus::NAV_READY) {
        if (!is_tf_odom_map_existed()) {
          RCLCPP_INFO(this->get_logger(), "nav_prepare");
          publish_status(RobotStatus::NAV_PREPARE);
          is_localized_ = false;
          this->set_parameter(
              Parameter("fitrobot_status", RobotStatus::NAV_PREPARE));
        }
        return;
      }

    } catch (tf2::LookupException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s",
                   ex.what());
    } catch (tf2::ExtrapolationException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform extrapolation failed: %s",
                   ex.what());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<fitrobot_interfaces::msg::RobotStatus>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool is_localized_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotStatusCheckNode>();
  RCLCPP_INFO(node->get_logger(), "Check Robot Status Node started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
