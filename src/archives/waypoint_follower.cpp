#include "fitrobot_interfaces/srv/waypoint_follower.hpp"
#include "fitrobot_interfaces/msg/station.h"
#include "fitrobot_interfaces/srv/cancel_nav.hpp"
#include "fitrobot_interfaces/srv/target_station.hpp"
#include "fitrobotcpp/robot_navigator.hpp"
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client_goal_handle.hpp>

using std::shared_ptr;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;
using GolaHandleFollowWaypoints =
    rclcpp_action::ClientGoalHandle<FollowWaypoints>;
using namespace std::placeholders;
using WaypointFollowerSrv = fitrobot_interfaces::srv::WaypointFollower;

class WaypointFollowerService : public rclcpp::Node {
public:
  WaypointFollowerService();

  void waypointFollowerCallback(
      const shared_ptr<rmw_request_id_t> /*request_header*/,
      const shared_ptr<WaypointFollowerSrv::Request> request,
      const shared_ptr<WaypointFollowerSrv::Response> /*response*/);

  void targetStationCallback(
      const shared_ptr<rmw_request_id_t> /*request_header*/,
      const shared_ptr<WaypointFollowerSrv::Request> request,
      const shared_ptr<WaypointFollowerSrv::Response> /*response*/);

  void cancelNavCallback(
      const shared_ptr<rmw_request_id_t> /*request_header*/,
      const shared_ptr<WaypointFollowerSrv::Request> request,
      const shared_ptr<WaypointFollowerSrv::Response> /*response*/);

  geometry_msgs::msg::PoseStamped
  convertStationToPose(const fitrobot_interfaces::msg::Station &station);

  void addStation(const fitrobot_interfaces::msg::Station &station);

private:
  rclcpp::Service<fitrobot_interfaces::srv::WaypointFollower>::SharedPtr
      waypoint_srv_;
  rclcpp::Service<fitrobot_interfaces::srv::TargetStation>::SharedPtr
      target_station_srv_;
  rclcpp::Service<fitrobot_interfaces::srv::CancelNav>::SharedPtr
      cancel_nav_srv_;
  rclcpp::Publisher<fitrobot_interfaces::msg::Station>::SharedPtr station_pub_;
  std::shared_ptr<BasicNavigator> navigator_;
  fitrobot_interfaces::msg::Station end_station_;
  fitrobot_interfaces::msg::Station target_station_;
};

WaypointFollowerService::WaypointFollowerService()
    : Node("waypoint_follower_service") {
  waypoint_srv_ = this->create_service<WaypointFollowerSrv>(
      "waypoint_follower",
      std::bind(&WaypointFollowerService::waypointFollowerCallback, this, _1,
                _2, _3));
  // 初始化其他服务和发布者
  navigator_ =
      std::make_shared<BasicNavigator>(); // 假设 BasicNavigator 是适当的 C++ 类
  RCLCPP_INFO(this->get_logger(), "rBBBBBBBBB");

  end_station_.name = "FA Room";
  end_station_.x = 0.5;
  end_station_.y = -1.5;
}

void WaypointFollowerService::waypointFollowerCallback(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<WaypointFollowerSrv::Request> request,
    const shared_ptr<WaypointFollowerSrv::Response> /*response*/) {
  RCLCPP_INFO(this->get_logger(), "waypoint follower服務開始");
  fitrobot_interfaces::msg::Station station = request->station;
  this->addStation(station);
  RCLCPP_INFO(this->get_logger(), "waypoint follower服務結束");
}

void WaypointFollowerService::targetStationCallback(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<WaypointFollowerSrv::Request> request,
    const shared_ptr<WaypointFollowerSrv::Response> response) {
  RCLCPP_INFO(this->get_logger(), "target station服務開始");
  fitrobot_interfaces::msg::Station station;
  // response->target_station = station;
  RCLCPP_INFO(this->get_logger(), "target station服務結束");
}

void WaypointFollowerService::cancelNavCallback(
    const shared_ptr<rmw_request_id_t> /*request_header*/,
    const shared_ptr<WaypointFollowerSrv::Request> request,
    const shared_ptr<WaypointFollowerSrv::Response> /*response*/) {
  // TBD
}

geometry_msgs::msg::PoseStamped WaypointFollowerService::convertStationToPose(
    const fitrobot_interfaces::msg::Station &station) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->get_clock()->now();

  pose.pose.position.x = station.x;
  pose.pose.position.y = station.y;
  pose.pose.orientation.z = station.z;
  pose.pose.orientation.w = station.w;

  return pose;
}

void WaypointFollowerService::addStation(
    const fitrobot_interfaces::msg::Station &station) {
  // Convert stations to poses
  std::vector<geometry_msgs::msg::PoseStamped> goal_poses;
  goal_poses.push_back(convertStationToPose(station));
  goal_poses.push_back(convertStationToPose(end_station_));

  // Send waypoints to the navigator
  navigator_->followWaypoints(goal_poses);

  // Log start information
  RCLCPP_INFO(this->get_logger(),
              "Starting transport task to station (name: %s, x: %f, y: %f)",
              station.name.c_str(), station.x, station.y);

  int i = 0;
  int current_status = -1; // Initialized to an invalid status

  // Monitor feedback and task completion
  while (!navigator_->isFollowWaypointsTaskComplete()) {
    i++;
    auto feedback = navigator_->getFeedbackFollowWaypoints();

    if (feedback && (i % 10 == 0)) {
      RCLCPP_INFO(this->get_logger(), "current_waypoint: %d\n",
                  feedback->current_waypoint);
      if (current_status != feedback->current_waypoint) {
        if (feedback->current_waypoint == 0) {
          target_station_ = station;
          RCLCPP_INFO(this->get_logger(),
                      "Heading to station (name: %s, x: %f, y: %f)",
                      target_station_.name.c_str(), target_station_.x,
                      target_station_.y);
        } else if (feedback->current_waypoint == 1) {
          target_station_ = end_station_;
          RCLCPP_INFO(this->get_logger(), "Transporting to FA Room");
        }
        // Publish current station
        station_pub_->publish(target_station_);
        current_status = feedback->current_waypoint;
      }
    } else {
      // RCLCPP_INFO(this->get_logger(), "no feedback...");
    }
    // Delay to prevent busy waiting
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  // Handle the result of the navigation task

  // rclcpp_action::ResultCode BasicNavigator::getResultFollowWaypoint() {

  // UNKNOWN = action_msgs::msg::GoalStatus::STATUS_UNKNOWN,
  // SUCCEEDED = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
  // CANCELED = action_msgs::msg::GoalStatus::STATUS_CANCELED,
  // ABORTED = action_msgs::msg::GoalStatus::STATUS_ABORTED
  //
  RCLCPP_INFO(this->get_logger(), "CCCCCCCCCCCCCCCCCCCC");

  rclcpp_action::ResultCode result = navigator_->getResultFollowWaypoint();
  switch (result) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Transport task completed!");
    // unfinished_station_count_--; // Assuming you have a similar variable
    // goHomeCheck();               // Replace with your relevant method
    // Publish RobotStatus or set parameters as needed
    break;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_INFO(this->get_logger(), "Transport task canceled!");
    // Publish RobotStatus or set parameters as needed
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_INFO(this->get_logger(), "Transport task failed!");
    // Publish RobotStatus or set parameters as needed
    break;
  default:
    RCLCPP_INFO(this->get_logger(),
                "Invalid return status from transport task!");
    break;
  }

  RCLCPP_INFO(this->get_logger(), "DDDDDDDDDDDDDDDDDDDD");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto waypoint_follower_service = std::make_shared<WaypointFollowerService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(waypoint_follower_service);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
