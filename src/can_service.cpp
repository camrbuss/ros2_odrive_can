#include "ros2_odrive_can/can_service.hpp"

CanService::CanService(/* args */) : Node("can_service"),
                                     socket_odrive_estop_(odrive_can::Msg::MSG_ODRIVE_ESTOP),
                                     socket_get_motor_error_(odrive_can::Msg::MSG_GET_MOTOR_ERROR),
                                     socket_get_encoder_error_(odrive_can::Msg::MSG_GET_ENCODER_ERROR),
                                     socket_get_sensorless_error_(odrive_can::Msg::MSG_GET_SENSORLESS_ERROR),
                                     socket_set_axis_node_id_(odrive_can::Msg::MSG_SET_AXIS_NODE_ID),
                                     socket_set_axis_requested_state_(odrive_can::Msg::MSG_SET_AXIS_REQUESTED_STATE),
                                     socket_set_axis_startup_config_(odrive_can::Msg::MSG_SET_AXIS_STARTUP_CONFIG),
                                     socket_get_encoder_estimates_(odrive_can::Msg::MSG_GET_ENCODER_ESTIMATES),
                                     socket_get_encoder_count_(odrive_can::Msg::MSG_GET_ENCODER_COUNT),
                                     socket_set_controller_modes_(odrive_can::Msg::MSG_SET_CONTROLLER_MODES),
                                     socket_set_input_pos_(odrive_can::Msg::MSG_SET_INPUT_POS),
                                     socket_set_input_vel_(odrive_can::Msg::MSG_SET_INPUT_VEL),
                                     socket_set_input_current_(odrive_can::Msg::MSG_SET_INPUT_CURRENT),
                                     socket_set_vel_limit_(odrive_can::Msg::MSG_SET_VEL_LIMIT),
                                     socket_start_anticogging_(odrive_can::Msg::MSG_START_ANTICOGGING),
                                     socket_set_traj_vel_limit_(odrive_can::Msg::MSG_SET_TRAJ_VEL_LIMIT),
                                     socket_set_traj_accel_limits_(odrive_can::Msg::MSG_SET_TRAJ_ACCEL_LIMITS),
                                     socket_set_traj_a_per_css_(odrive_can::Msg::MSG_SET_TRAJ_A_PER_CSS),
                                     socket_get_iq_(odrive_can::Msg::MSG_GET_IQ),
                                     socket_get_sensorless_estimates_(odrive_can::Msg::MSG_GET_SENSORLESS_ESTIMATES),
                                     socket_reset_odrive_(odrive_can::Msg::MSG_RESET_ODRIVE),
                                     socket_get_vbus_voltage_(odrive_can::Msg::MSG_GET_VBUS_VOLTAGE),
                                     socket_clear_errors_(odrive_can::Msg::MSG_CLEAR_ERRORS)
{
    service_odrive_estop_ = this->create_service<ros2_odrive_can::srv::OdriveEstop>("odrive/odrive_estop", std::bind(&CanService::odrive_estop_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_motor_error_ = this->create_service<ros2_odrive_can::srv::GetMotorError>("odrive/get_motor_error", std::bind(&CanService::get_motor_error_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_encoder_error_ = this->create_service<ros2_odrive_can::srv::GetEncoderError>("odrive/get_encoder_error", std::bind(&CanService::get_encoder_error_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_sensorless_error_ = this->create_service<ros2_odrive_can::srv::GetSensorlessError>("odrive/get_sensorless_error", std::bind(&CanService::get_sensorless_error_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_axis_node_id_ = this->create_service<ros2_odrive_can::srv::SetAxisNodeId>("odrive/set_axis_node_id", std::bind(&CanService::set_axis_node_id_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_axis_requested_state_ = this->create_service<ros2_odrive_can::srv::SetAxisRequestedState>("odrive/set_axis_requested_state", std::bind(&CanService::set_axis_requested_state_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_axis_startup_config_ = this->create_service<ros2_odrive_can::srv::SetAxisStartupConfig>("odrive/set_axis_startup_config", std::bind(&CanService::set_axis_startup_config_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_encoder_estimates_ = this->create_service<ros2_odrive_can::srv::GetEncoderEstimates>("odrive/get_encoder_estimates", std::bind(&CanService::get_encoder_estimates_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_encoder_count_ = this->create_service<ros2_odrive_can::srv::GetEncoderCount>("odrive/get_encoder_count", std::bind(&CanService::get_encoder_count_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_controller_modes_ = this->create_service<ros2_odrive_can::srv::SetControllerModes>("odrive/set_controller_modes", std::bind(&CanService::set_controller_modes_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_input_pos_ = this->create_service<ros2_odrive_can::srv::SetInputPos>("odrive/set_input_pos", std::bind(&CanService::set_input_pos_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_input_vel_ = this->create_service<ros2_odrive_can::srv::SetInputVel>("odrive/set_input_vel", std::bind(&CanService::set_input_vel_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_input_current_ = this->create_service<ros2_odrive_can::srv::SetInputCurrent>("odrive/set_input_current", std::bind(&CanService::set_input_current_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_vel_limit_ = this->create_service<ros2_odrive_can::srv::SetVelLimit>("odrive/set_vel_limit", std::bind(&CanService::set_vel_limit_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_start_anticogging_ = this->create_service<ros2_odrive_can::srv::StartAnticogging>("odrive/start_anticogging", std::bind(&CanService::start_anticogging_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_traj_vel_limit_ = this->create_service<ros2_odrive_can::srv::SetTrajVelLimit>("odrive/set_traj_vel_limit", std::bind(&CanService::set_traj_vel_limit_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_traj_accel_limits_ = this->create_service<ros2_odrive_can::srv::SetTrajAccelLimits>("odrive/set_traj_accel_limits", std::bind(&CanService::set_traj_accel_limits_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_set_traj_a_per_css_ = this->create_service<ros2_odrive_can::srv::SetTrajAPerCss>("odrive/set_traj_a_per_css", std::bind(&CanService::set_traj_a_per_css_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_iq_ = this->create_service<ros2_odrive_can::srv::GetIq>("odrive/get_iq", std::bind(&CanService::get_iq_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_sensorless_estimates_ = this->create_service<ros2_odrive_can::srv::GetSensorlessEstimates>("odrive/get_sensorless_estimates", std::bind(&CanService::get_sensorless_estimates_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_reset_odrive_ = this->create_service<ros2_odrive_can::srv::ResetOdrive>("odrive/reset_odrive", std::bind(&CanService::reset_odrive_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_get_vbus_voltage_ = this->create_service<ros2_odrive_can::srv::GetVbusVoltage>("odrive/get_vbus_voltage", std::bind(&CanService::get_vbus_voltage_callback, this, std::placeholders::_1, std::placeholders::_2));
    service_clear_errors_ = this->create_service<ros2_odrive_can::srv::ClearErrors>("odrive/clear_errors", std::bind(&CanService::clear_errors_callback, this, std::placeholders::_1, std::placeholders::_2));
}

CanService::~CanService()
{
}

void CanService::odrive_estop_callback(const std::shared_ptr<ros2_odrive_can::srv::OdriveEstop::Request> request, std::shared_ptr<ros2_odrive_can::srv::OdriveEstop::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    send_frame.can_id = odrive_can::Msg::MSG_ODRIVE_ESTOP;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }
    socket_get_vbus_voltage_.writeFrame(send_frame);
    response->success = true;
}
void CanService::get_motor_error_callback(const std::shared_ptr<ros2_odrive_can::srv::GetMotorError::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetMotorError::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::get_encoder_error_callback(const std::shared_ptr<ros2_odrive_can::srv::GetEncoderError::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetEncoderError::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::get_sensorless_error_callback(const std::shared_ptr<ros2_odrive_can::srv::GetSensorlessError::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetSensorlessError::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::set_axis_node_id_callback(const std::shared_ptr<ros2_odrive_can::srv::SetAxisNodeId::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetAxisNodeId::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::set_axis_requested_state_callback(const std::shared_ptr<ros2_odrive_can::srv::SetAxisRequestedState::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetAxisRequestedState::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 8;
    send_frame.data[0] = request->requested_state;
    send_frame.data[1] = request->requested_state >> 8;
    send_frame.data[2] = request->requested_state >> 16;
    send_frame.data[3] = request->requested_state >> 24;
    send_frame.can_id = odrive_can::Msg::MSG_SET_AXIS_REQUESTED_STATE;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        response->success = false;
        return;
    }

    socket_set_axis_requested_state_.writeFrame(send_frame);
    response->success = true;
}
void CanService::set_axis_startup_config_callback(const std::shared_ptr<ros2_odrive_can::srv::SetAxisStartupConfig::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetAxisStartupConfig::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::get_encoder_estimates_callback(const std::shared_ptr<ros2_odrive_can::srv::GetEncoderEstimates::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetEncoderEstimates::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    send_frame.can_id = 1 << 30; // Set ID to have RTR bit set.
    send_frame.can_id += odrive_can::Msg::MSG_GET_ENCODER_ESTIMATES;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        return;
    }
    socket_get_encoder_estimates_.writeFrame(send_frame);
    can_frame recv_frame = socket_get_encoder_estimates_.readFrame();
    RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3], recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);
    response->pos_estimate = odrive_can::can_getSignal<float>(recv_frame, 0, 32, true);
    response->vel_estimate = odrive_can::can_getSignal<float>(recv_frame, 32, 32, true);
}
void CanService::get_encoder_count_callback(const std::shared_ptr<ros2_odrive_can::srv::GetEncoderCount::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetEncoderCount::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::set_controller_modes_callback(const std::shared_ptr<ros2_odrive_can::srv::SetControllerModes::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetControllerModes::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::set_input_pos_callback(const std::shared_ptr<ros2_odrive_can::srv::SetInputPos::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetInputPos::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::set_input_vel_callback(const std::shared_ptr<ros2_odrive_can::srv::SetInputVel::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetInputVel::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::set_input_current_callback(const std::shared_ptr<ros2_odrive_can::srv::SetInputCurrent::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetInputCurrent::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::set_vel_limit_callback(const std::shared_ptr<ros2_odrive_can::srv::SetVelLimit::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetVelLimit::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::start_anticogging_callback(const std::shared_ptr<ros2_odrive_can::srv::StartAnticogging::Request> request, std::shared_ptr<ros2_odrive_can::srv::StartAnticogging::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::set_traj_vel_limit_callback(const std::shared_ptr<ros2_odrive_can::srv::SetTrajVelLimit::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetTrajVelLimit::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::set_traj_accel_limits_callback(const std::shared_ptr<ros2_odrive_can::srv::SetTrajAccelLimits::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetTrajAccelLimits::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::set_traj_a_per_css_callback(const std::shared_ptr<ros2_odrive_can::srv::SetTrajAPerCss::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetTrajAPerCss::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::get_iq_callback(const std::shared_ptr<ros2_odrive_can::srv::GetIq::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetIq::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::get_sensorless_estimates_callback(const std::shared_ptr<ros2_odrive_can::srv::GetSensorlessEstimates::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetSensorlessEstimates::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        return;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        return;
    }
    else
    {
        return;
    }
}
void CanService::reset_odrive_callback(const std::shared_ptr<ros2_odrive_can::srv::ResetOdrive::Request> request, std::shared_ptr<ros2_odrive_can::srv::ResetOdrive::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0 || odrive_can::AXIS::AXIS_1)
    {
        can_frame send_frame;
        send_frame.can_dlc = 0;
        send_frame.can_id = odrive_can::AXIS::AXIS_0_ID;
        send_frame.can_id += odrive_can::Msg::MSG_RESET_ODRIVE;
        socket_get_vbus_voltage_.writeFrame(send_frame);
        response->success = true;
    }
    else
    {
        response->success = false;
        return;
    }
}
void CanService::get_vbus_voltage_callback(const std::shared_ptr<ros2_odrive_can::srv::GetVbusVoltage::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetVbusVoltage::Response> response)
{
    if (request->axis == odrive_can::AXIS::AXIS_0 || odrive_can::AXIS::AXIS_1)
    {
        can_frame send_frame;
        send_frame.can_dlc = 0;
        send_frame.can_id = 1 << 30; // Set ID to have RTR bit set.
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
        send_frame.can_id += odrive_can::Msg::MSG_GET_VBUS_VOLTAGE;
        socket_get_vbus_voltage_.writeFrame(send_frame);
        can_frame recv_frame = socket_get_vbus_voltage_.readFrame();
        RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", recv_frame.data[0], recv_frame.data[1], recv_frame.data[2], recv_frame.data[3], recv_frame.data[4], recv_frame.data[5], recv_frame.data[6], recv_frame.data[7]);
        response->vbus_voltage = odrive_can::can_getSignal<float>(recv_frame, 0, 32, true);
    }
    else
    {
        return;
    }
}
void CanService::clear_errors_callback(const std::shared_ptr<ros2_odrive_can::srv::ClearErrors::Request> request, std::shared_ptr<ros2_odrive_can::srv::ClearErrors::Response> response)
{
    can_frame send_frame;
    send_frame.can_dlc = 0;
    send_frame.can_id = odrive_can::Msg::MSG_CLEAR_ERRORS;
    if (request->axis == odrive_can::AXIS::AXIS_0)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_0_ID;
    }
    else if (request->axis == odrive_can::AXIS::AXIS_1)
    {
        send_frame.can_id += odrive_can::AXIS::AXIS_1_ID;
    }
    else
    {
        return;
    }
    socket_get_vbus_voltage_.writeFrame(send_frame);
    response->success = true;
}