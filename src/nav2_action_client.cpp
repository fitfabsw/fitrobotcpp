#include "fitrobot_interfaces/srv/cancel_nav.hpp"
#include "fitrobot_interfaces/srv/waypoint_follower.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ref: https://qiita.com/porizou1/items/cb9382bb2955c144d168

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using WaypointFollowerSrv = fitrobot_interfaces::srv::WaypointFollower;
using CancelNavSrv = fitrobot_interfaces::srv::CancelNav;
using FitStation = fitrobot_interfaces::msg::Station;
using std::shared_ptr;

using json = nlohmann::json;
namespace fs = std::filesystem;

struct Station {
    std::string type;
    std::string name;
    double x, y, z, w;
};

std::string get_package_path(const std::string& package_name) {
    try {
        std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
        return package_path;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Package '%s' not found: %s",
                     package_name.c_str(), e.what());
        return "";
    }
}

std::string getMapName() {
    // Execute the command and capture the output
    std::string cmd = "ros2 param get /master_service active_nav_map";
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    // Process the result string as needed
    result = result.substr(17);
    result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());
    return result;
}

json getStationList() {

    std::string my_package_path = get_package_path("fitrobot");
    if (!my_package_path.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Package path found: %s",
                    my_package_path.c_str());
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Package path is empty");
    }

    std::string mapName = getMapName();
    std::cout << "mapName: " << mapName << std::endl;

    fs::path stationListFilePath = fs::path(my_package_path) / "data" / "station_list.json";
    std::cout << "Station List File Path: " << stationListFilePath << std::endl;

    std::ifstream jsonFile(stationListFilePath);
    json completeStationListJson;
    jsonFile >> completeStationListJson;
    return completeStationListJson[mapName]["station_list"];
}

std::pair<Station, Station> getStartAndEndStations() {
    json stationListJson = getStationList();
    Station start, end;

    for (const auto& item : stationListJson) {
        std::cout << "item: " << item << std::endl;
        if (item["type"] == "start") {
            start = {item["type"], item["name"], item["x"], item["y"], item["z"], item["w"]};
        } else if (item["type"] == "end") {
            end = {item["type"], item["name"], item["x"], item["y"], item["z"], item["w"]};
        }
    }
    return std::make_pair(start, end);
}

class Nav2Client : public rclcpp::Node {
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using NavigateToPoseClient = rclcpp_action::Client<NavigateToPose>;
    using FollowWaypointsClient = rclcpp_action::Client<FollowWaypoints>;
    using GoalHandleNavigateToPose = NavigateToPoseClient::GoalHandle;
    using GoalHandleFollowWaypoints = FollowWaypointsClient::GoalHandle;
    using FollowWaypointsFeedback = nav2_msgs::action::FollowWaypoints::Feedback;
    using CancelGoalResponse = action_msgs::srv::CancelGoal_Response;

    // Shared pointers for action clients
    std::shared_ptr<NavigateToPoseClient> client_ptr_;
    std::shared_ptr<FollowWaypointsClient> client_fw_ptr_;

    // Future and shared pointers for handling responses and feedback
    std::shared_future<std::shared_ptr<GoalHandleFollowWaypoints>> resultFutureFollowWaypoints_;
    GoalHandleFollowWaypoints::WrappedResult result_;
    std::shared_ptr<GoalHandleFollowWaypoints> goal_handle_;
    std::shared_future<GoalHandleFollowWaypoints::WrappedResult> future_result_;
    std::shared_future<std::shared_ptr<CancelGoalResponse>> cancel_response_;
    std::shared_ptr<const FollowWaypointsFeedback> feedback_;

    bool isTaskComplete_;
    int current_waypoint_index_ = -1;

    explicit Nav2Client() : Node("nav2_send_goal"), isTaskComplete_(false) {
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        this->client_fw_ptr_ =
            rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
    }

    void follow(std::vector<geometry_msgs::msg::PoseStamped> poses) {
        while (!this->client_fw_ptr_->wait_for_action_server()) {
            RCLCPP_INFO(get_logger(), "Waiting for action server...");
        }
        RCLCPP_INFO(get_logger(), "follow start");
        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses = poses;

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.feedback_callback =
            std::bind(&Nav2Client::feedbackCallbackFollowWaypoints, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&Nav2Client::resultCallbackFollowWaypoints, this, _1);

        resultFutureFollowWaypoints_ = client_fw_ptr_->async_send_goal(goal_msg, send_goal_options);

        if (rclcpp::spin_until_future_complete(shared_from_this()->get_node_base_interface(),
                                               resultFutureFollowWaypoints_) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "get result call failed :(");
        } else {
            RCLCPP_INFO(get_logger(), "get result call success :)");
        }
        std::future_status status =
            resultFutureFollowWaypoints_.wait_for(1000s); // timeout to guarantee a graceful finish
        if (status == std::future_status::ready) {
            if (resultFutureFollowWaypoints_.valid()) {
                goal_handle_ = resultFutureFollowWaypoints_.get();
                if (!goal_handle_) {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                                "Goal accepted by server, waiting for result");
                }
            }
            future_result_ = client_fw_ptr_->async_get_result(goal_handle_);
            RCLCPP_INFO(get_logger(), "future_result async_get_result sent !!!");
        } else {
            RCLCPP_INFO(get_logger(), "goal_handle not ready");
        }
        RCLCPP_INFO(get_logger(), "follow end");
    }

    bool isTaskComplete() {
        if (!future_result_.valid()) {
            return true;
        }
        if (rclcpp::spin_until_future_complete(shared_from_this()->get_node_base_interface(),
                                               future_result_, std::chrono::milliseconds(100)) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            return false;
        } else {
            result_ = future_result_.get();
            if (result_.code != rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(get_logger(), "Task with failed with status code: %d",
                            static_cast<int>(result_.code));
                return true;
            }
            return true;
        }
        return true;
    }

    void cancelTask() {
        RCLCPP_INFO(get_logger(), "cancelTask");
        if (future_result_.valid()) {
            RCLCPP_INFO(get_logger(), "future_result.valid()");
            auto response_received_callback =
                [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> response) {
                    if (response) {
                        RCLCPP_INFO(get_logger(), "Cancel goal response received.");
                    } else {
                        RCLCPP_ERROR(get_logger(), "Failed to receive cancel goal response.");
                    }
                };
            auto future_result =
                client_fw_ptr_->async_cancel_goal(goal_handle_, response_received_callback);
            RCLCPP_INFO(get_logger(), "cancelTask done");
        }
    }

    void feedbackCallbackFollowWaypoints(
        GoalHandleFollowWaypoints::SharedPtr,
        const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
        feedback_ = feedback;
        if (current_waypoint_index_ != feedback->current_waypoint) {
            current_waypoint_index_ = feedback->current_waypoint;
            RCLCPP_INFO(get_logger(), "Passing waypoint %d", current_waypoint_index_);
        }
    }

    void resultCallbackFollowWaypoints(const GoalHandleFollowWaypoints::WrappedResult& result) {
        RCLCPP_INFO(get_logger(), "resultCallbackFollowWaypoints");
        resultFutureFollowWaypoints_.get();
        RCLCPP_INFO(get_logger(), "resultCallbackFollowWaypoints");
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal reached successfully");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            return;
        }
    }

    geometry_msgs::msg::PoseStamped createPose(double x, double y, double theta) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation = toQuaternion(0.0, 0.0, theta);
        return pose;
    }

    geometry_msgs::msg::Quaternion toQuaternion(double roll, double pitch, double yaw) {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        return tf2::toMsg(q);
    }
};

class Nav2Service : public rclcpp::Node {
  public:
    std::shared_ptr<Nav2Client> nav2_client_;
    Station start_station, end_station;
    rclcpp::Service<fitrobot_interfaces::srv::WaypointFollower>::SharedPtr waypoint_srv_;
    rclcpp::Service<fitrobot_interfaces::srv::CancelNav>::SharedPtr cancel_nav_srv_;
    rclcpp::Publisher<FitStation>::SharedPtr target_station_pub_;
    int current_waypoint_index_ = -1;
    rclcpp::CallbackGroup::SharedPtr service_cbg_MU;
    rclcpp::CallbackGroup::SharedPtr service_cbg_RE;
    int count_;

    explicit Nav2Service()
        : Node("nav2_service"), nav2_client_(std::make_shared<Nav2Client>()), count_(0) {
        service_cbg_MU = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_cbg_RE = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        waypoint_srv_ = this->create_service<WaypointFollowerSrv>(
            "waypoint_follower",
            std::bind(&Nav2Service::waypointFollowerCallback, this, _1, _2, _3),
            rmw_qos_profile_services_default, service_cbg_RE);

        cancel_nav_srv_ = this->create_service<CancelNavSrv>(
            "cancel_nav", std::bind(&Nav2Service::cancelNavCallback, this, _1, _2, _3),
            rmw_qos_profile_services_default, service_cbg_MU);

        std::string my_package_path = get_package_path("fitrobot");
        std::tie(start_station, end_station) = getStartAndEndStations();
        RCLCPP_INFO(this->get_logger(), "Start Station: %s, End Station: %s",
                    start_station.name.c_str(), end_station.name.c_str());

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.transient_local();
        qos.reliable();
        target_station_pub_ = this->create_publisher<FitStation>("target_station", qos);
    }

    void cancelNavCallback(const shared_ptr<rmw_request_id_t> /*request_header*/,
                           const shared_ptr<CancelNavSrv::Request> request,
                           const shared_ptr<CancelNavSrv::Response> /*response*/) {
        RCLCPP_INFO(this->get_logger(), "cancel_nav 服務開始");
        nav2_client_->cancelTask();
        RCLCPP_INFO(this->get_logger(), "cancel nav 服務結束");
    }

    void waypointFollowerCallback(const shared_ptr<rmw_request_id_t> /*request_header*/,
                                  const shared_ptr<WaypointFollowerSrv::Request> request,
                                  const shared_ptr<WaypointFollowerSrv::Response> /*response*/) {
        RCLCPP_INFO(this->get_logger(), "waypoint follower服務開始");
        while (count_ > 0) {
        }
        count_++;
        FitStation station = request->station;
        addStation(station);
        count_--;
        RCLCPP_INFO(this->get_logger(), "waypoint follower服務結束");
    }

    void addStation(const FitStation& station) {
        RCLCPP_INFO(get_logger(), "addStation start");
        std::vector<geometry_msgs::msg::PoseStamped> poses =
            this->createWaypointsFromStations(station);
        nav2_client_->follow(poses);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Triggered!!");
        while (!nav2_client_->isTaskComplete()) {
        }
        RCLCPP_INFO(get_logger(), "addStation done");
    }

    std::vector<geometry_msgs::msg::PoseStamped> createWaypointsFromStations(const FitStation& st) {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        Station station = {st.type, st.name, st.x, st.y, st.z, st.w};
        waypoints.push_back(createPose(station.x, station.y, station.z));
        waypoints.push_back(createPose(start_station.x, start_station.y, start_station.z));
        return waypoints;
    }

    geometry_msgs::msg::PoseStamped createPose(double x, double y, double theta) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation = toQuaternion(0.0, 0.0, theta);
        return pose;
    }

    geometry_msgs::msg::Quaternion toQuaternion(double roll, double pitch, double yaw) {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        return tf2::toMsg(q);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2Service>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();
    return 0;
}
