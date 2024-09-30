#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

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

int main() {
  try {
    auto [start, end] = getStartAndEndStations();
    std::cout << "Start station: " << start.name
              << ", End station: " << end.name << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
