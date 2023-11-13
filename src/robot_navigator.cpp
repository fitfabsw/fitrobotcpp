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
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using rclcpp::QoS;
using namespace std::placeholders;

class BasicNavigator : public rclcpp::Node {
  public:
    BasicNavigator() : Node("basic_navigator") {
        // Initialize publishers, subscribers, and clients
        initial_pose_ = geometry_msgs::msg::PoseStamped();
        initial_pose_.header.frame_id = "map";
        initial_pose_received_ = false;

        QoS amcl_pose_qos(rclcpp::KeepLast(1));
        amcl_pose_qos.transient_local();
        amcl_pose_qos.reliable();

        // Action Clients
        nav_to_pose_client_ =
            rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        follow_waypoints_client_ =
            rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

        // Subscription
        localization_pose_sub_ =
            this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "amcl_pose", amcl_pose_qos, std::bind(&BasicNavigator::amclPoseCallback, this, _1));

        // Publishers
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10);
    }

    ~BasicNavigator() {
        // 销毁动作客户端
        if (nav_to_pose_client_) {
            nav_to_pose_client_.reset();
        }
        if (follow_waypoints_client_) {
            follow_waypoints_client_.reset();
        }
        // 可以添加其他清理资源的代码
    }

    void setInitialPose(const geometry_msgs::msg::PoseStamped& initial_pose) {
        initial_pose_ = initial_pose;

        auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        initial_pose_msg.header = initial_pose_.header;
        initial_pose_msg.pose.pose = initial_pose_.pose;

        initial_pose_pub_->publish(initial_pose_msg);
        initial_pose_received_ = false;
    }

    void goToPose(const geometry_msgs::msg::PoseStamped& pose) {
        if (!nav_to_pose_client_->wait_for_action_server(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = pose;

        // Send the goal to the action server
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback =
            std::bind(&BasicNavigator::feedbackCallback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&BasicNavigator::resultCallback, this, _1);

        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
                          const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Current distance to goal: %f",
                    feedback->distance_remaining);
        feedback_ = feedback;
    }

    void resultCallback(const GoalHandleNavigateToPose::WrappedResult& result) {
        result_future_ = std::async(std::launch::deferred, [result]() { return result; });
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

    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "AMCL pose received");
        // 这里可以添加处理位置信息的代码
    }

    // create a function isTaskComplete
    bool isTaskComplete() {
        if (!result_future_.valid()) {
            return true; // 任务已被取消或完成
        }

        if (result_future_.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
            return true; // 任务已完成
        }

        return false; // 任务尚未完成
    }

    // create a function getFeedback
    std::shared_ptr<const NavigateToPose::Feedback> getFeedback() { return feedback_; }

    NavigateToPose::Result getResult() {
        if (result_future_.valid()) {
            auto wrapped_result = result_future_.get();
            if (wrapped_result.result) {
                return *wrapped_result.result; // 返回解引用后的结果对象
            }
        }
        return nav2_msgs::action::NavigateToPose::Result(); // 返回默认构造的结果对象
    }

  private:
    geometry_msgs::msg::PoseStamped initial_pose_;

    std::shared_ptr<const NavigateToPose::Feedback> feedback_;
    std::shared_future<GoalHandleNavigateToPose::WrappedResult> result_future_;

    bool initial_pose_received_;

    // ActionClients
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;

    // Subscription & Callbacks
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        localization_pose_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto navigator = std::make_shared<BasicNavigator>();

    // Set initial pose, go to a pose, etc.

    rclcpp::spin(navigator);
    rclcpp::shutdown();
    return 0;
}
