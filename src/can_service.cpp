#include "ros2_odrive_can/can_service.hpp"


CanService::CanService(/* args */) : Node("can_service"), si_(0x21)
{
    service_ = this->create_service<ros2_odrive_can::srv::EncoderEstimates>("odrive/encoder_estimates", std::bind(&CanService::callback, this, std::placeholders::_1, std::placeholders::_2));

    send_frame_.can_dlc = 1;
    send_frame_.can_id = 0b00000000001;
    send_frame_.data[0] = 0xff;
}

CanService::~CanService()
{
}

void CanService::callback(const std::shared_ptr<ros2_odrive_can::srv::EncoderEstimates::Request> request, std::shared_ptr<ros2_odrive_can::srv::EncoderEstimates::Response> response)
{
    if (request->axis == 0)
    {
        response->pos_estimate = 0.1;
        response->vel_estimate = 0.2;
    }
    else if (request->axis == 1)
    {
        response->pos_estimate = 1.1;
        response->vel_estimate = 2.2;
    } else
    {
        response->pos_estimate = 0.0;
        response->vel_estimate = 0.0;

    }
    
}