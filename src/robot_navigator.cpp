#include "fitrobotcpp/robot_navigator.hpp"
#include <functional>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;
using GoalHandleFollowWaypoints =
    rclcpp_action::ClientGoalHandle<FollowWaypoints>;
using rclcpp::QoS;
using namespace std::placeholders;

BasicNavigator::BasicNavigator() : Node("basic_navigator") {
  // Initialize publishers, subscribers, and clients
  initial_pose_ = geometry_msgs::msg::PoseStamped();
  initial_pose_.header.frame_id = "map";
  initial_pose_received_ = false;

  server_timeout_ = std::chrono::milliseconds(100);

  QoS amcl_pose_qos(rclcpp::KeepLast(1));
  amcl_pose_qos.transient_local();
  amcl_pose_qos.reliable();

  // Action Clients
  // nav_to_pose_client_ =
  //     rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  follow_waypoints_client_ =
      rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

  // Subscription
  localization_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "amcl_pose", amcl_pose_qos,
          std::bind(&BasicNavigator::amclPoseCallback, this, _1));

  // Publishers
  initial_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", 10);

  // using FeedbackMessage = nav2_msgs::action::FollowWaypoints_FeedbackMessage;

  RCLCPP_INFO(this->get_logger(), "basic_navigator create feedback_sub");
  // navigation_feedback_sub_ = this->create_subscription<
  //     nav2_msgs::action::FollowWaypoints::Impl::FeedbackMessage>(
  //     "follow_waypoints/_action/feedback", rclcpp::SystemDefaultsQoS(),
  //     [this](const
  //     nav2_msgs::action::FollowWaypoints::Impl::FeedbackMessage::
  //                SharedPtr msg) {
  //       RCLCPP_INFO(this->get_logger(), "FEEDBACK!!!!!");
  //       // feedbackFollowWaypoints_ = msg->feedback;
  //     });

  // ros2 topic echo follow_waypoints/_action/feedback
  // navigation_feedback_sub_ = this->create_subscription<
  //     nav2_msgs::action::FollowWaypoints::Impl::FeedbackMessage>(
  //     "follow_waypoints/_action/feedback", rclcpp::SystemDefaultsQoS(),
  //     std::bind(&BasicNavigator::feedbackCallbackkk, this,
  //               std::placeholders::_1));

  RCLCPP_ERROR(this->get_logger(), "basic_navigator create feedback_sub done");
}

// void BasicNavigator::feedbackCallbackkk(
//     const
//     nav2_msgs::action::FollowWaypoints::Impl::FeedbackMessage::SharedPtr
//         msg) {
//   RCLCPP_INFO(this->get_logger(), "Feedback received");
//   // 在這裡處理反饋
//   // 例如: feedbackFollowWaypoints_ = msg->feedback;
// }

BasicNavigator::~BasicNavigator() {
  // 销毁动作客户端
  // if (nav_to_pose_client_) {
  //   nav_to_pose_client_.reset();
  // }
  if (follow_waypoints_client_) {
    follow_waypoints_client_.reset();
  }
}

void BasicNavigator::setInitialPose(
    const geometry_msgs::msg::PoseStamped &initial_pose) {
  initial_pose_ = initial_pose;

  auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
  initial_pose_msg.header = initial_pose_.header;
  initial_pose_msg.pose.pose = initial_pose_.pose;

  initial_pose_pub_->publish(initial_pose_msg);
  initial_pose_received_ = false;
}

// void BasicNavigator::goToPose(const geometry_msgs::msg::PoseStamped &pose) {
//   if (!nav_to_pose_client_->wait_for_action_server(1s)) {
//     RCLCPP_ERROR(this->get_logger(),
//                  "Action server not available after waiting");
//     return;
//   }
//   auto goal_msg = NavigateToPose::Goal();
//   goal_msg.pose = pose;

//   auto send_goal_options =
//       rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

//   send_goal_options.feedback_callback = std::bind(
//       static_cast<void (BasicNavigator::*)(
//           GoalHandleNavigateToPose::SharedPtr,
//           const std::shared_ptr<const NavigateToPose::Feedback> feedback)>(
//           &BasicNavigator::feedbackCallback),
//       this, _1, _2);

//   send_goal_options.result_callback =
//       std::bind(static_cast<void (BasicNavigator::*)(
//                     const GoalHandleNavigateToPose::WrappedResult &)>(
//                     &BasicNavigator::resultCallback),
//                 this, _1);

//   nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
// }

void BasicNavigator::followWaypoints(
    const std::vector<geometry_msgs::msg::PoseStamped> &poses) {
  if (!follow_waypoints_client_->wait_for_action_server(1s)) {
    RCLCPP_ERROR(this->get_logger(),
                 "FollowWaypoints action server not available after waiting");

    return;
  }
  auto goal_msg = FollowWaypoints::Goal();
  goal_msg.poses = poses;

  auto send_goal_options =
      rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

  // send_goal_options.feedback_callback = std::bind(
  //     static_cast<void (BasicNavigator::*)(
  //         GoalHandleFollowWaypoints::SharedPtr,
  //         const std::shared_ptr<const FollowWaypoints::Feedback> feedback)>(
  //         &BasicNavigator::feedbackCallback),
  //     this, _1, _2);

  send_goal_options.feedback_callback =
      [this](GoalHandleFollowWaypoints::SharedPtr goal_handle,
             const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
        this->feedbackCallback(goal_handle, feedback);
      };

  // send_goal_options.feedback_callback = [this](auto goal_handle, auto
  // feedback) {
  //   this->feedbackCallback(goal_handle, feedback);
  // };

  // send_goal_options.feedback_callback = [this](auto, auto feedback) {
  //   // follow_waypoints_client_.reset();
  //   RCLCPP_INFO(this->get_logger(), "Feedback received: %d",
  //               feedback->current_waypoint);
  //   feedbackFollowWaypoints_ = feedback;
  // };

  // send_goal_options.feedback_callback = [this](auto goal_handle,
  //                                              auto feedback) {
  //   RCLCPP_INFO(this->get_logger(), "Feedback received: %d",
  //               feedback->current_waypoint);
  //   feedbackFollowWaypoints_ = feedback;
  // };

  send_goal_options.result_callback = [this](const auto result) {
    RCLCPP_INFO(this->get_logger(), "Result received");
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
    }
    resultFollowWaypoints_ = result.code;
  };

  // send_goal_options.result_callback =
  //     std::bind(static_cast<void (BasicNavigator::*)(
  //                   const GoalHandleFollowWaypoints::WrappedResult &)>(
  //                   &BasicNavigator::resultCallback),
  //               this, _1);

  // follow_waypoints_client_->async_send_goal(goal_msg, send_goal_options);

  resultFutureFollowWaypoints_ =
      follow_waypoints_client_->async_send_goal(goal_msg, send_goal_options);
}

// void BasicNavigator::feedbackCallback(
//     GoalHandleNavigateToPose::SharedPtr,
//     const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
//   RCLCPP_INFO(this->get_logger(), "Current distance to goal: %f",
//               feedback->distance_remaining);
//   feedbackNavToPose_ = feedback;
// }

void BasicNavigator::feedbackCallback(
    GoalHandleFollowWaypoints::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
  RCLCPP_INFO(this->get_logger(), "followWaypoints feedbackCallback!!!!!");
  feedbackFollowWaypoints_ = feedback;
}

// void BasicNavigator::resultCallback(
//     const GoalHandleNavigateToPose::WrappedResult &result) {
//   resultFutureNavToPose_ =
//       std::async(std::launch::deferred, [result]() { return result; });
//   resultNavToPose_ = result.code;
//   switch (result.code) {
//   case rclcpp_action::ResultCode::SUCCEEDED:
//     RCLCPP_INFO(this->get_logger(), "Goal reached");
//     break;
//   case rclcpp_action::ResultCode::ABORTED:
//     RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//     break;
//   case rclcpp_action::ResultCode::CANCELED:
//     RCLCPP_INFO(this->get_logger(), "Goal was canceled");
//     break;
//   default:
//     RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//     break;
//   }
// }

void BasicNavigator::resultCallback(
    const GoalHandleFollowWaypoints::WrappedResult &result) {
  RCLCPP_INFO(this->get_logger(), "followWaypoints resultCallback!!!!!");
  // resultFutureFollowWaypoints_ =
  //     std::async(std::launch::deferred, [result]() { return result; });
  resultFollowWaypoints_ = result.code;
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
    break;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_INFO(this->get_logger(), "Goal was canceled");
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
    break;
  }
}

void BasicNavigator::amclPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "AMCL pose received");
}

bool BasicNavigator::isFollowWaypointsTaskComplete() {

  // RCLCPP_INFO(this->get_logger(), "isFollowWaypointsTaskComplete starts");
  if (!resultFutureFollowWaypoints_.valid()) {
    // RCLCPP_INFO(this->get_logger(), "...1");
    return true; // Task has been canceled or completed
  }

  if (resultFutureFollowWaypoints_.wait_for(std::chrono::milliseconds(100)) ==
      std::future_status::ready) {
    // RCLCPP_INFO(this->get_logger(), "...2");
    return true; // Task has been completed
  }
  // RCLCPP_INFO(this->get_logger(), "...3");

  return false; // Task is still running
}

// std::shared_ptr<const NavigateToPose::Feedback>
// BasicNavigator::getFeedbackNavToPose() {
//   return feedbackNavToPose_;
// }

std::shared_ptr<const FollowWaypoints::Feedback>
BasicNavigator::getFeedbackFollowWaypoints() {
  return feedbackFollowWaypoints_;
}

// rclcpp_action::ResultCode BasicNavigator::getResultNavToPose() {
//   return resultNavToPose_;
// }

rclcpp_action::ResultCode BasicNavigator::getResultFollowWaypoint() {
  return resultFollowWaypoints_;
}
