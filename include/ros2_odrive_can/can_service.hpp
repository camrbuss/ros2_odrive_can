#include <linux/can/raw.h>


#include "rclcpp/rclcpp.hpp"
#include "ros2_odrive_can/msg/odrive_status.hpp"
#include "ros2_odrive_can/srv/encoder_estimates.hpp"
#include "ros2_odrive_can/socketcan_interface.hpp"

class CanService : public rclcpp::Node
{
public:
    CanService(/* args */);
    ~CanService();

private:

    void callback(const std::shared_ptr<ros2_odrive_can::srv::EncoderEstimates::Request> request, std::shared_ptr<ros2_odrive_can::srv::EncoderEstimates::Response> response);

    rclcpp::Service<ros2_odrive_can::srv::EncoderEstimates>::SharedPtr service_;

    canid_t c_id = 0x33;
    can_frame send_frame_;

    SocketcanInterface si_;
};