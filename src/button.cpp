#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include <pigpiod_if2.h>

using namespace std::chrono_literals;

class DigitalReader : public rclcpp::Node {
  private:
    int pi_;
    int pin_;
    bool is_pull_up_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
    std_msgs::msg::Empty empty_message;
    size_t count_;
    int pressed;
    int clicked;
    void timer_callback() {
        if (gpio_read(pi_, pin_) == 1) {
            pressed = 1;
            clicked = 0;
        } else {
            if (pressed == 1) {
                clicked = 1;
                pressed = 0;
                // RCLCPP_INFO(this->get_logger(), "clicked");
                publisher_->publish(empty_message);
            }
        }
    }

  public:
    DigitalReader() : Node("gpio_publisher"), count_(0), pressed(0) {
        this->declare_parameter<int>("pin", 5);
        this->declare_parameter<bool>("is_pull_up", true);
        this->get_parameter("pin", pin_);
        this->get_parameter("is_pull_up", is_pull_up_);
        if (is_pull_up_) {
            RCLCPP_INFO(this->get_logger(), "Read GPIO-%02d (PULL_UP)", pin_);
        } else {
            RCLCPP_INFO(this->get_logger(), "Read GPIO-%02d (PULL_DOWN)", pin_);
        }
        pi_ = pigpio_start(NULL, NULL); /* Connect to Pi. */
        if (pi_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "cannot connect pigpiod");
            rclcpp::shutdown();
            exit(1);
        } else {
            set_mode(pi_, pin_, PI_INPUT);
            if (is_pull_up_) {
                set_pull_up_down(pi_, pin_, PI_PUD_UP);
            } else {
                set_pull_up_down(pi_, pin_, PI_PUD_OFF);
            }
            publisher_ = this->create_publisher<std_msgs::msg::Empty>("ok_to_go", 10);
            timer_ = this->create_wall_timer(50ms, std::bind(&DigitalReader::timer_callback, this));
        }
    }

    ~DigitalReader() { pigpio_stop(pi_); }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DigitalReader>());
    rclcpp::shutdown();

    return 0;
}
