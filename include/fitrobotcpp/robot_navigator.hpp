// robot_navigator.hpp
#ifndef ROBOT_NAVIGATOR_HPP
#define ROBOT_NAVIGATOR_HPP

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

using GoalHandleFollowWaypoints =
    rclcpp_action::ClientGoalHandle<FollowWaypoints>;
// using WaypointFollowerGoalHandle =
//     rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;

using rclcpp::QoS;
using namespace std::placeholders;

class BasicNavigator : public rclcpp::Node {
public:
  BasicNavigator();
  ~BasicNavigator();

  void setInitialPose(const geometry_msgs::msg::PoseStamped &initial_pose);

  // void goToPose(const geometry_msgs::msg::PoseStamped &pose);
  void
  followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped> &poses);

  // void feedbackCallback(
  //     GoalHandleNavigateToPose::SharedPtr,
  //     const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void feedbackCallback(
      GoalHandleFollowWaypoints::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback);

  // void feedbackCallbackkk(
  //     const nav2_msgs::action::FollowWaypoints::Impl::FeedbackMessage::SharedPtr
  //         msg);

  // void resultCallback(const GoalHandleNavigateToPose::WrappedResult &result);
  void resultCallback(const GoalHandleFollowWaypoints::WrappedResult &result);

  void amclPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  bool isFollowWaypointsTaskComplete();

  // std::shared_ptr<const NavigateToPose::Feedback> getFeedbackNavToPose();
  std::shared_ptr<const FollowWaypoints::Feedback> getFeedbackFollowWaypoints();
  // rclcpp_action::ResultCode getResultNavToPose();
  rclcpp_action::ResultCode getResultFollowWaypoint();

private:
  // GoalHandleFollowWaypoints::SharedPtr waypoint_follower_goal_handle_;
  std::chrono::milliseconds server_timeout_;

  // rclcpp::Subscription<
  //     nav2_msgs::action::FollowWaypoints::Impl::FeedbackMessage>::SharedPtr
  //     navigation_feedback_sub_;

  bool initial_pose_received_;
  geometry_msgs::msg::PoseStamped initial_pose_;

  // std::shared_ptr<const NavigateToPose::Feedback> feedbackNavToPose_;
  std::shared_ptr<const FollowWaypoints::Feedback> feedbackFollowWaypoints_;

  // rclcpp_action::ResultCode resultNavToPose_;
  rclcpp_action::ResultCode resultFollowWaypoints_;

  // std::shared_future<GoalHandleNavigateToPose::WrappedResult>
  //     resultFutureNavToPose_;

  std::shared_future<std::shared_ptr<GoalHandleFollowWaypoints>>
      resultFutureFollowWaypoints_;

  // rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      localization_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_pub_;
};

#endif // ROBOT_NAVIGATOR_HPP
