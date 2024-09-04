#include "fitrobot_interfaces/msg/robot_status.hpp"
#include "fitrobot_interfaces/srv/navigation.hpp"
#include "fitrobot_interfaces/srv/remote_control.hpp"
#include "fitrobot_interfaces/srv/slam.hpp"
#include "fitrobot_interfaces/srv/subscription_count.hpp"
#include "fitrobot_interfaces/srv/terminate_process.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include <memory>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <string>
#include <sys/wait.h>
#include <unordered_map>

using std::placeholders::_1;
using std::placeholders::_2;
// using std::placeholders::_3;

class MasterAsyncService : public rclcpp::Node {
  public:
    MasterAsyncService() : Node("master_service") {
        // Declare parameters
        this->declare_parameter("active_nav_map", "active_nav_map_not_set");

        // 初始化 robot_type 和 robot_id
        set_robot_info_from_env();

        // QoS settings
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

        // Publishers and services
        // status_pub_ =
        //     this->create_publisher<fitrobot_interfaces::msg::RobotStatus>("robot_status", qos);

        nav_srv_ = this->create_service<fitrobot_interfaces::srv::Navigation>(
            "navigation", std::bind(&MasterAsyncService::navigation_callback, this, _1, _2));

        slam_srv_ = this->create_service<fitrobot_interfaces::srv::Slam>(
            "slam", std::bind(&MasterAsyncService::slam_callback, this, _1, _2));

        remote_control_srv_ = this->create_service<fitrobot_interfaces::srv::RemoteControl>(
            "remote_control",
            std::bind(&MasterAsyncService::remote_control_callback, this, _1, _2));

        terminate_srv_ = this->create_service<fitrobot_interfaces::srv::TerminateProcess>(
            "terminate_slam_or_navigation",
            std::bind(&MasterAsyncService::terminate_slam_or_navigation_callback, this, _1, _2));

        subscription_count_srv_ = this->create_service<fitrobot_interfaces::srv::SubscriptionCount>(
            "subscription_count",
            std::bind(&MasterAsyncService::subscription_count_callback, this, _1, _2));

        service_cbg_MU = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        service_cbg_RE = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        lfm_nav_client = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
            node_name_ + "/lifecycle_manager_navigation/manage_nodes",
            rmw_qos_profile_services_default, service_cbg_MU);

        lfm_costmap_filter_client = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
            node_name_ + "/lifecycle_manager_costmap_filters/manage_nodes",
            rmw_qos_profile_services_default, service_cbg_MU);

        param_set_service_ = this->create_service<fitrobot_interfaces::srv::Navigation>(
            "reconfigure_map_mask",
            std::bind(&MasterAsyncService::reconfigure_map_mask, this, _1, _2));

        manage_nodes_service_ = this->create_service<std_srvs::srv::Empty>(
            "manage_nodes", std::bind(&MasterAsyncService::manage_nodes_callback, this, _1, _2));
        // "manage_nodes", std::bind(&MasterAsyncService::manage_nodes_callback, this, _1, _2),
        // rmw_qos_profile_services_default, service_cbg_MU);
        // rmw_qos_profile_services_default, service_cbg_RE);

        // Handle environment variables
        char* robot_info = std::getenv("ROBOT_INFO");
        if (!robot_info) {
            RCLCPP_ERROR(this->get_logger(), "Environment variable ROBOT_INFO is not set!");
            throw std::runtime_error("Environment variable ROBOT_INFO is required.");
        }

        // Further initialization...
    }

  private:
    std::string robot_type_;
    std::string robot_id_;
    std::string node_name_;
    bool use_sim;
    bool launch_service_active;
    pid_t launch_service_pid; // PID of the launch service process
    rclcpp::CallbackGroup::SharedPtr service_cbg_MU;
    rclcpp::CallbackGroup::SharedPtr service_cbg_RE;
    // rclcpp::Publisher<fitrobot_interfaces::msg::RobotStatus>::SharedPtr status_pub_;
    rclcpp::Service<fitrobot_interfaces::srv::Navigation>::SharedPtr nav_srv_;
    rclcpp::Service<fitrobot_interfaces::srv::Slam>::SharedPtr slam_srv_;
    rclcpp::Service<fitrobot_interfaces::srv::RemoteControl>::SharedPtr remote_control_srv_;
    rclcpp::Service<fitrobot_interfaces::srv::TerminateProcess>::SharedPtr terminate_srv_;
    rclcpp::Service<fitrobot_interfaces::srv::SubscriptionCount>::SharedPtr subscription_count_srv_;
    // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr param_set_service_;
    rclcpp::Service<fitrobot_interfaces::srv::Navigation>::SharedPtr param_set_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr manage_nodes_service_;
    std::unordered_map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> param_clients_;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr
        lfm_nav_client; // lifecycle_manager_navigation
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr
        lfm_costmap_filter_client; // lifecycle_manager_costmap_filter

    void shutdown_launch_service() {
        // std::this_thread::sleep_for(std::chrono::seconds(2)); // 模擬等待時間
        if (launch_service_pid != -1) {
            RCLCPP_INFO(this->get_logger(), "Shutting down launch service with PID: %d",
                        launch_service_pid);

            // Send SIGTERM to gracefully terminate the process
            kill(launch_service_pid, SIGTERM);

            // Optionally wait for the process to terminate
            int status;
            pid_t result = waitpid(launch_service_pid, &status, 0);
            if (result == -1) {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to wait for the launch service process termination.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Launch service process terminated successfully.");
            }

            // Reset the PID since the process has been terminated
            launch_service_pid = -1;
        } else {
            RCLCPP_WARN(this->get_logger(), "No active launch service to shut down.");
        }
        RCLCPP_INFO(this->get_logger(), "Launch service has been shut down.");
    }

    void clean_up(bool ensure_bringup = true) {
        if (launch_service_active) {
            RCLCPP_INFO(this->get_logger(), "clean_up: ready to shutdown.");
            shutdown_launch_service();
            // if (ensure_bringup) {
            //     auto check_status_future = std::async(std::launch::async, [this]() {
            //         // 模擬檢查機器人狀態直到它回到 BRINGUP
            //         while (rclcpp::ok()) {
            //             int robot_status = send_get_parameters_request();
            //             if (robot_status == 1) {
            //                 break;
            //             }
            //             std::this_thread::sleep_for(std::chrono::seconds(1));
            //         }
            //     });
            //     check_status_future.get();
            //     RCLCPP_INFO(this->get_logger(), "clean_up: done. robot_status back to BRINGUP");
            // }
            launch_service_active = false;
        }
    }

    // void launch_function(const std::string& package_name, const std::string& launch_file,
    //                      const std::vector<std::string>& args) {
    //     std::ostringstream launch_command;
    //     std::string source_prefix = "bash -c 'source ~/simulations/install/setup.bash && ";
    //     launch_command << source_prefix << "ros2 launch " << package_name << " " << launch_file;
    //     for (const auto& arg : args) {
    //         launch_command << " " << arg;
    //     }
    //     launch_command << "'"; // 結束 bash -c 的命令
    //     RCLCPP_INFO(this->get_logger(), "Launching command: %s", launch_command.str().c_str());
    //     int result = std::system(launch_command.str().c_str());
    //     if (result != 0) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to launch %s with launch file %s",
    //                      package_name.c_str(), launch_file.c_str());
    //     } else {
    //         RCLCPP_INFO(this->get_logger(), "Successfully launched %s with launch file %s",
    //                     package_name.c_str(), launch_file.c_str());
    //     }
    // }

    void launch_function_async(const std::string& package_name, const std::string& launch_file,
                               const std::vector<std::string>& args) {
        pid_t pid = fork();
        if (pid == 0) {
            std::ostringstream launch_command;
            std::string source_prefix = "source ~/simulations/install/setup.bash && ";
            launch_command << source_prefix << "ros2 launch " << package_name << " " << launch_file;
            for (const auto& arg : args) {
                launch_command << " " << arg;
            }
            std::string command_str = launch_command.str();
            const char* command = command_str.c_str();
            RCLCPP_INFO(this->get_logger(), "Launching command: %s", command);
            execl("/bin/bash", "bash", "-c", command, (char*)nullptr);
            _exit(EXIT_FAILURE);
        } else if (pid < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to fork a new process for launch command.");
        } else {
            launch_service_pid = pid;     // 設置全局PID為子進程的PID
            launch_service_active = true; // 設置為活動狀態
            RCLCPP_INFO(this->get_logger(), "Successfully launched %s with launch file %s. PID: %d",
                        package_name.c_str(), launch_file.c_str(), launch_service_pid);
        }
    }

    void
    lifecycle_manage_cmd(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client,
                         int cmd) {
        // uint8 STARTUP = 0
        // uint8 PAUSE = 1
        // uint8 RESUME = 2
        // uint8 RESET = 3
        // uint8 SHUTDOWN = 4
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "ManageLifecycleNodes service not available.");
            return;
        }
        auto request_srv = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        request_srv->command = cmd;
        auto future = client->async_send_request(request_srv);
        auto status = future.wait_for(std::chrono::seconds(6));
        if (status == std::future_status::ready) {
            try {
                auto result = future.get();
                RCLCPP_INFO(this->get_logger(),
                            "ManageLifecycleNodes service call succeeded. cmd=%d", cmd);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Timeout while waiting for ManageLifecycleNodes service to complete.");
        }
    }

    void
    reset_and_startup(rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr& client) {
        RCLCPP_INFO(this->get_logger(), "manage_nodes_callback started");
        lifecycle_manage_cmd(client, 0); // in order to set parameters
        lifecycle_manage_cmd(
            client,
            3); // after params are set, need to reset to re-configure parameters to take effect
        lifecycle_manage_cmd(client, 0);
    }

    void manage_nodes_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                               std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        reset_and_startup(lfm_nav_client);
        reset_and_startup(lfm_costmap_filter_client);
    }

    // 解析環境變數並設置 robot_type_ 和 robot_id_
    void set_robot_info_from_env() {
        const char* robot_info = std::getenv("ROBOT_INFO");
        if (!robot_info) {
            RCLCPP_ERROR(this->get_logger(), "Environment variable ROBOT_INFO is not set!");
            throw std::runtime_error("Environment variable ROBOT_INFO is required.");
        }

        std::string robot_info_str(robot_info);
        std::stringstream ss(robot_info_str);
        std::string first_robot_info;
        std::getline(ss, first_robot_info, ';'); // 取得第一個機器人的信息

        std::stringstream robot_ss(first_robot_info);
        std::getline(robot_ss, robot_type_, ':'); // 取得機器人類型
        std::getline(robot_ss, robot_id_, ':');   // 取得機器人序號

        use_sim = atoi(std::getenv("USE_SIM")) == 1 ? true : false;
        RCLCPP_INFO(this->get_logger(), "use_sim: %s", use_sim ? "true" : "false");

        node_name_ = "/" + robot_type_ + "_" + robot_id_;
        RCLCPP_INFO(this->get_logger(), "Node name set to: %s", node_name_.c_str());
    }

    bool set_parameter_for_node(const std::string& node_name, const std::string& param_name,
                                const std::string& param_value) {
        if (param_clients_.find(node_name) == param_clients_.end()) {
            param_clients_[node_name] =
                std::make_shared<rclcpp::AsyncParametersClient>(this, node_name);
        }

        auto param_client = param_clients_[node_name];
        if (!param_client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Parameter service not available for node %s",
                         node_name.c_str());
            return false;
        }

        auto future = param_client->set_parameters({rclcpp::Parameter(param_name, param_value)});
        auto status = future.wait_for(std::chrono::milliseconds(100));
        if (status == std::future_status::ready) {
            try {
                auto result = future.get();
                if (result[0].successful) {
                    RCLCPP_INFO(this->get_logger(),
                                "Successfully set parameter %s to %s on node %s",
                                param_name.c_str(), param_value.c_str(), node_name.c_str());
                    return true;
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s",
                                 result[0].reason.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Timeout while waiting for the parameter set operation to complete.");
        }
        return false;
    }

    void reconfigure_map_mask(
        const std::shared_ptr<fitrobot_interfaces::srv::Navigation::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::Navigation::Response> response) {
        std::string worldname = request->worldname;
        std::string param_name = "map_topic";
        std::string param_value = "/" + worldname + "/" + robot_type_ + "/map";
        std::string param_name2 = "mask_topic";
        std::string param_value2 = "/" + worldname + "/" + robot_type_ + "/keepout_filter_mask";
        std::vector<std::string> nodes_to_update = {node_name_ + "/amcl",
                                                    node_name_ + "/local_costmap/local_costmap",
                                                    node_name_ + "/global_costmap/global_costmap",
                                                    node_name_ + "/costmap_filter_info_server"};

        bool success = false;
        for (const auto& node : nodes_to_update) {
            if (node != node_name_ + "/costmap_filter_info_server") {
                success = set_parameter_for_node(node, param_name, param_value);
            } else {
                success = set_parameter_for_node(node, param_name2, param_value2);
            }
            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set parameter for node %s",
                             node.c_str());
            }
        }
        reset_and_startup(lfm_nav_client);
        reset_and_startup(lfm_costmap_filter_client);
    }

    // void run_navigation(std::string worldname) {
    //     std::string package_name = "linorobot2_navigation";
    //     std::string launch_file = "nav.launch.py";
    //     std::vector<std::string> args = {
    //         "worldname:=" + worldname, "sim:=" + std::string(use_sim ? "true" : "false"),
    //         "rviz:=false", std::string("namespace:=") + "/" + robot_type_ + "_" + robot_id_};
    //     launch_function(package_name, launch_file, args);
    // }

    void run_navigation_async(std::string worldname) {
        std::string package_name = "linorobot2_navigation";
        std::string launch_file = "nav.launch.py";
        std::vector<std::string> args = {
            "worldname:=" + worldname, "sim:=" + std::string(use_sim ? "true" : "false"),
            "rviz:=false", std::string("namespace:=") + "/" + robot_type_ + "_" + robot_id_};
        launch_function_async(package_name, launch_file, args);
    }

    void navigation_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::Navigation::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::Navigation::Response> response) {
        RCLCPP_INFO(this->get_logger(), "navigation_callback started");
        // Implementation of navigation callback
        // run_navigation(request->worldname);
        run_navigation_async(request->worldname);
        RCLCPP_INFO(this->get_logger(), "navigation_callback finished");
    }

    void slam_callback(const std::shared_ptr<fitrobot_interfaces::srv::Slam::Request> request,
                       std::shared_ptr<fitrobot_interfaces::srv::Slam::Response> response) {
        RCLCPP_INFO(this->get_logger(), "slam_callback started");
        // Implementation of SLAM callback
        clean_up();
        RCLCPP_INFO(this->get_logger(), "slam_callback finished");
    }

    void remote_control_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::RemoteControl::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::RemoteControl::Response> response) {
        RCLCPP_INFO(this->get_logger(), "remote_control_callback started");
        // Implementation of remote control callback
        RCLCPP_INFO(this->get_logger(), "remote_control_callback finished");
    }

    void terminate_slam_or_navigation_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::TerminateProcess::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::TerminateProcess::Response> response) {
        RCLCPP_INFO(this->get_logger(), "terminate_slam_or_navigation_callback started");
        // Implementation to terminate processes
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "terminate_slam_or_navigation_callback finished");
    }

    void subscription_count_callback(
        const std::shared_ptr<fitrobot_interfaces::srv::SubscriptionCount::Request> request,
        std::shared_ptr<fitrobot_interfaces::srv::SubscriptionCount::Response> response) {
        RCLCPP_INFO(this->get_logger(), "subscription_count_callback started");
        // Implementation of subscription count callback
        RCLCPP_INFO(this->get_logger(), "subscription_count_callback finished");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MasterAsyncService>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    rclcpp::shutdown();

    // auto node = std::make_shared<MasterAsyncService>();
    // rclcpp::spin(node);
    // rclcpp::shutdown();

    return 0;
}
