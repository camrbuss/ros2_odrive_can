#include <cstdio>
#include <chrono>
#include <memory>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_odrive_can/msg/odrive_status.hpp"
#include "ros2_odrive_can/socketcan_interface.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(): Node("minimal_publisher"), count_(0), si_(0x21)
  {
    publisher_ = this->create_publisher<ros2_odrive_can::msg::OdriveStatus>("topic", 10);
    timer_ = this->create_wall_timer(
        50ms, std::bind(&MinimalPublisher::timer_callback, this));

    send_frame_.can_dlc = 1;
    send_frame_.can_id = 0b00000000001;
    send_frame_.data[0] = 0xff;
  }

private:
  void timer_callback()
  {
    ros2_odrive_can::msg::OdriveStatus message;
    message.axis_error = 1;
    message.axis_state = 2;
    can_frame recv_frame = si_.readFrame();
    RCLCPP_INFO(this->get_logger(), "this->now time:  %zu %u", this->now().nanoseconds(), recv_frame.can_id);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%u'", );

    // si_.writeFrame(send_frame_);

    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ros2_odrive_can::msg::OdriveStatus>::SharedPtr publisher_;
  size_t count_;


  canid_t c_id = 0x33;
  can_frame send_frame_;

  SocketcanInterface si_;

};

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  auto node1 = std::make_shared<MinimalPublisher>();
  exec.add_node(node1);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
