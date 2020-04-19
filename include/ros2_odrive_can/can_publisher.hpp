#include <linux/can/raw.h>
#include <linux/can.h>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "ros2_odrive_can/socketcan_interface.hpp"
#include "ros2_odrive_can/odrive_can.hpp"

#include "ros2_odrive_can/msg/odrive_status.hpp"

class CanPublisher : public rclcpp::Node
{
public:
    CanPublisher(/* args */);
    ~CanPublisher();

private:
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    SocketcanInterface socket_axis0_read_;
    SocketcanInterface socket_axis1_read_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros2_odrive_can::msg::OdriveStatus>::SharedPtr publisher_;
    rclcpp::Clock ros_clock_;

    void timerCallback();
};