#include "fitrobot_interfaces/srv/waypoint_follower.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ref: https://qiita.com/porizou1/items/cb9382bb2955c144d168

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using WaypointFollowerSrv = fitrobot_interfaces::srv::WaypointFollower;
using FitStation = fitrobot_interfaces::msg::Station;
using std::shared_ptr;

using json = nlohmann::json;
namespace fs = std::filesystem;

struct Station {
  std::string type;
  std::string name;
  double x, y, z, w;
};

std::string get_package_path(const std::string &package_name) {
  try {
    std::string package_path =
        ament_index_cpp::get_package_share_directory(package_name);
    return package_path;
  } catch (const std::exception &e) {
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
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"),
                                                pclose);
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

  fs::path stationListFilePath =
      fs::path(my_package_path) / "data" / "station_list.json";
  std::cout << "Station List File Path: " << stationListFilePath << std::endl;

  std::ifstream jsonFile(stationListFilePath);
  json completeStationListJson;
  jsonFile >> completeStationListJson;
  return completeStationListJson[mapName]["station_list"];
}

std::pair<Station, Station> getStartAndEndStations() {
  json stationListJson = getStationList();
  Station start, end;

  for (const auto &item : stationListJson) {
    std::cout << "item: " << item << std::endl;
    if (item["type"] == "start") {
      start = {item["type"], item["name"], item["x"],
               item["y"],    item["z"],    item["w"]};
    } else if (item["type"] == "end") {
      end = {item["type"], item["name"], item["x"],
             item["y"],    item["z"],    item["w"]};
    }
  }

  return std::make_pair(start, end);
}

class Nav2Client : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints =
      rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_fw_ptr_;

  explicit Nav2Client() : Node("nav2_send_goal") {
    this->client_ptr_ =
        rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    this->client_fw_ptr_ =
        rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
    waypoint_srv_ = this->create_service<WaypointFollowerSrv>(
        "waypoint_follower",
        std::bind(&Nav2Client::waypointFollowerCallback, this, _1, _2, _3));

    std::string my_package_path = get_package_path("fitrobot");
    std::tie(start_station, end_station) = getStartAndEndStations();
    RCLCPP_INFO(this->get_logger(), "Start Station: %s, End Station: %s",
                start_station.name.c_str(), end_station.name.c_str());

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();
    target_station_pub_ =
        this->create_publisher<FitStation>("target_station", qos);
  }

  Station start_station, end_station;
  rclcpp::Service<fitrobot_interfaces::srv::WaypointFollower>::SharedPtr
      waypoint_srv_;
  rclcpp::Publisher<FitStation>::SharedPtr target_station_pub_;
  std::shared_future<std::shared_ptr<GoalHandleFollowWaypoints>>
      resultFutureFollowWaypoints_;
  int current_waypoint_index_ = -1;

  void waypointFollowerCallback(
      const shared_ptr<rmw_request_id_t> /*request_header*/,
      const shared_ptr<WaypointFollowerSrv::Request> request,
      const shared_ptr<WaypointFollowerSrv::Response> /*response*/) {
    RCLCPP_INFO(this->get_logger(), "waypoint follower服務開始");
    FitStation station = request->station;
    addStation(station);
    RCLCPP_INFO(this->get_logger(), "waypoint follower服務結束");
  }

  void addStation(const FitStation &station) {
    RCLCPP_INFO(get_logger(), "addStation start");
    this->follow(station);
    RCLCPP_INFO(get_logger(), "addStation done");
  }

  void follow(const FitStation &station) {
    while (!this->client_fw_ptr_->wait_for_action_server()) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }
    auto goal_msg = FollowWaypoints::Goal();
    goal_msg.poses = createWaypointsFromStations(station);

    auto send_goal_options =
        rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

    send_goal_options.feedback_callback =
        std::bind(&Nav2Client::feedbackCallbackFollowWaypoints, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&Nav2Client::resultCallbackFollowWaypoints, this, _1);

    resultFutureFollowWaypoints_ =
        client_fw_ptr_->async_send_goal(goal_msg, send_goal_options);

    // rclcpp::spin_until_future_complete(shared_from_this(),
    // resultFutureFollowWaypoints_); auto goal_handle =
    // resultFutureFollowWaypoints_.get();
  }

  std::vector<geometry_msgs::msg::PoseStamped>
  createWaypointsFromStations(const FitStation &st) {
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    Station station = {st.type, st.name, st.x, st.y, st.z, st.w};
    // Add station, end_station, start_station as waypoints
    waypoints.push_back(createPose(station.x, station.y, station.z));
    waypoints.push_back(
        createPose(start_station.x, start_station.y, start_station.z));
    waypoints.push_back(
        createPose(end_station.x, end_station.y, end_station.z));
    return waypoints;
  }

  void feedbackCallbackFollowWaypoints(
      GoalHandleFollowWaypoints::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
    if (current_waypoint_index_ != feedback->current_waypoint) {
      current_waypoint_index_ = feedback->current_waypoint;
      RCLCPP_INFO(get_logger(), "Passing waypoint %d", current_waypoint_index_);
    }
  }

  void resultCallbackFollowWaypoints(
      const GoalHandleFollowWaypoints::WrappedResult &result) {
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

  void sendGoal(void) {
    while (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = 2;
    goal_msg.pose.pose.position.y = 0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;
    goal_msg.pose.pose.orientation.z = 0.0;

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
        std::bind(&Nav2Client::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&Nav2Client::resultCallback, this, _1);
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void feedbackCallback(
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO(get_logger(), "Distance remaininf = %f",
                feedback->distance_remaining);
  }

  void resultCallback(const GoalHandleNavigateToPose::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Success!!!");
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

  geometry_msgs::msg::Quaternion toQuaternion(double roll, double pitch,
                                              double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Client>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
