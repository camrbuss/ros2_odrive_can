#include <linux/can/raw.h>
#include <linux/can.h>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "ros2_odrive_can/socketcan_interface.hpp"
#include "ros2_odrive_can/odrive_can.hpp"

// #include "ros2_odrive_can/msg/odrive_status.hpp"

#include "ros2_odrive_can/srv/odrive_estop.hpp"
#include "ros2_odrive_can/srv/get_motor_error.hpp"
#include "ros2_odrive_can/srv/get_encoder_error.hpp"
#include "ros2_odrive_can/srv/get_sensorless_error.hpp"
#include "ros2_odrive_can/srv/set_axis_node_id.hpp"
#include "ros2_odrive_can/srv/set_axis_requested_state.hpp"
#include "ros2_odrive_can/srv/set_axis_startup_config.hpp"
#include "ros2_odrive_can/srv/get_encoder_estimates.hpp"
#include "ros2_odrive_can/srv/get_encoder_count.hpp"
#include "ros2_odrive_can/srv/set_controller_modes.hpp"
#include "ros2_odrive_can/srv/set_input_pos.hpp"
#include "ros2_odrive_can/srv/set_input_vel.hpp"
#include "ros2_odrive_can/srv/set_input_current.hpp"
#include "ros2_odrive_can/srv/set_vel_limit.hpp"
#include "ros2_odrive_can/srv/start_anticogging.hpp"
#include "ros2_odrive_can/srv/set_traj_vel_limit.hpp"
#include "ros2_odrive_can/srv/set_traj_accel_limits.hpp"
#include "ros2_odrive_can/srv/set_traj_a_per_css.hpp"
#include "ros2_odrive_can/srv/get_iq.hpp"
#include "ros2_odrive_can/srv/get_sensorless_estimates.hpp"
#include "ros2_odrive_can/srv/reset_odrive.hpp"
#include "ros2_odrive_can/srv/get_vbus_voltage.hpp"
#include "ros2_odrive_can/srv/clear_errors.hpp"

class CanService : public rclcpp::Node
{
public:
    CanService(/* args */);
    ~CanService();

private:
    rclcpp::Service<ros2_odrive_can::srv::OdriveEstop>::SharedPtr service_odrive_estop_;
    rclcpp::Service<ros2_odrive_can::srv::GetMotorError>::SharedPtr service_get_motor_error_;
    rclcpp::Service<ros2_odrive_can::srv::GetEncoderError>::SharedPtr service_get_encoder_error_;
    rclcpp::Service<ros2_odrive_can::srv::GetSensorlessError>::SharedPtr service_get_sensorless_error_;
    rclcpp::Service<ros2_odrive_can::srv::SetAxisNodeId>::SharedPtr service_set_axis_node_id_;
    rclcpp::Service<ros2_odrive_can::srv::SetAxisRequestedState>::SharedPtr service_set_axis_requested_state_;
    rclcpp::Service<ros2_odrive_can::srv::SetAxisStartupConfig>::SharedPtr service_set_axis_startup_config_;
    rclcpp::Service<ros2_odrive_can::srv::GetEncoderEstimates>::SharedPtr service_get_encoder_estimates_;
    rclcpp::Service<ros2_odrive_can::srv::GetEncoderCount>::SharedPtr service_get_encoder_count_;
    rclcpp::Service<ros2_odrive_can::srv::SetControllerModes>::SharedPtr service_set_controller_modes_;
    rclcpp::Service<ros2_odrive_can::srv::SetInputPos>::SharedPtr service_set_input_pos_;
    rclcpp::Service<ros2_odrive_can::srv::SetInputVel>::SharedPtr service_set_input_vel_;
    rclcpp::Service<ros2_odrive_can::srv::SetInputCurrent>::SharedPtr service_set_input_current_;
    rclcpp::Service<ros2_odrive_can::srv::SetVelLimit>::SharedPtr service_set_vel_limit_;
    rclcpp::Service<ros2_odrive_can::srv::StartAnticogging>::SharedPtr service_start_anticogging_;
    rclcpp::Service<ros2_odrive_can::srv::SetTrajVelLimit>::SharedPtr service_set_traj_vel_limit_;
    rclcpp::Service<ros2_odrive_can::srv::SetTrajAccelLimits>::SharedPtr service_set_traj_accel_limits_;
    rclcpp::Service<ros2_odrive_can::srv::SetTrajAPerCss>::SharedPtr service_set_traj_a_per_css_;
    rclcpp::Service<ros2_odrive_can::srv::GetIq>::SharedPtr service_get_iq_;
    rclcpp::Service<ros2_odrive_can::srv::GetSensorlessEstimates>::SharedPtr service_get_sensorless_estimates_;
    rclcpp::Service<ros2_odrive_can::srv::ResetOdrive>::SharedPtr service_reset_odrive_;
    rclcpp::Service<ros2_odrive_can::srv::GetVbusVoltage>::SharedPtr service_get_vbus_voltage_;
    rclcpp::Service<ros2_odrive_can::srv::ClearErrors>::SharedPtr service_clear_errors_;

    SocketcanInterface socket_get_motor_error_;
    SocketcanInterface socket_get_encoder_error_;
    SocketcanInterface socket_get_sensorless_error_;
    SocketcanInterface socket_get_encoder_estimates_;
    SocketcanInterface socket_get_encoder_count_;
    SocketcanInterface socket_get_iq_;
    SocketcanInterface socket_get_sensorless_estimates_;
    SocketcanInterface socket_get_vbus_voltage_;
    SocketcanInterface socket_generic_write_;

    void odrive_estop_callback(const std::shared_ptr<ros2_odrive_can::srv::OdriveEstop::Request> request, std::shared_ptr<ros2_odrive_can::srv::OdriveEstop::Response> response);
    void get_motor_error_callback(const std::shared_ptr<ros2_odrive_can::srv::GetMotorError::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetMotorError::Response> response);
    void get_encoder_error_callback(const std::shared_ptr<ros2_odrive_can::srv::GetEncoderError::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetEncoderError::Response> response);
    void get_sensorless_error_callback(const std::shared_ptr<ros2_odrive_can::srv::GetSensorlessError::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetSensorlessError::Response> response);
    void set_axis_node_id_callback(const std::shared_ptr<ros2_odrive_can::srv::SetAxisNodeId::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetAxisNodeId::Response> response);
    void set_axis_requested_state_callback(const std::shared_ptr<ros2_odrive_can::srv::SetAxisRequestedState::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetAxisRequestedState::Response> response);
    void set_axis_startup_config_callback(const std::shared_ptr<ros2_odrive_can::srv::SetAxisStartupConfig::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetAxisStartupConfig::Response> response);
    void get_encoder_estimates_callback(const std::shared_ptr<ros2_odrive_can::srv::GetEncoderEstimates::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetEncoderEstimates::Response> response);
    void get_encoder_count_callback(const std::shared_ptr<ros2_odrive_can::srv::GetEncoderCount::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetEncoderCount::Response> response);
    void set_controller_modes_callback(const std::shared_ptr<ros2_odrive_can::srv::SetControllerModes::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetControllerModes::Response> response);
    void set_input_pos_callback(const std::shared_ptr<ros2_odrive_can::srv::SetInputPos::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetInputPos::Response> response);
    void set_input_vel_callback(const std::shared_ptr<ros2_odrive_can::srv::SetInputVel::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetInputVel::Response> response);
    void set_input_current_callback(const std::shared_ptr<ros2_odrive_can::srv::SetInputCurrent::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetInputCurrent::Response> response);
    void set_vel_limit_callback(const std::shared_ptr<ros2_odrive_can::srv::SetVelLimit::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetVelLimit::Response> response);
    void start_anticogging_callback(const std::shared_ptr<ros2_odrive_can::srv::StartAnticogging::Request> request, std::shared_ptr<ros2_odrive_can::srv::StartAnticogging::Response> response);
    void set_traj_vel_limit_callback(const std::shared_ptr<ros2_odrive_can::srv::SetTrajVelLimit::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetTrajVelLimit::Response> response);
    void set_traj_accel_limits_callback(const std::shared_ptr<ros2_odrive_can::srv::SetTrajAccelLimits::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetTrajAccelLimits::Response> response);
    void set_traj_a_per_css_callback(const std::shared_ptr<ros2_odrive_can::srv::SetTrajAPerCss::Request> request, std::shared_ptr<ros2_odrive_can::srv::SetTrajAPerCss::Response> response);
    void get_iq_callback(const std::shared_ptr<ros2_odrive_can::srv::GetIq::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetIq::Response> response);
    void get_sensorless_estimates_callback(const std::shared_ptr<ros2_odrive_can::srv::GetSensorlessEstimates::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetSensorlessEstimates::Response> response);
    void reset_odrive_callback(const std::shared_ptr<ros2_odrive_can::srv::ResetOdrive::Request> request, std::shared_ptr<ros2_odrive_can::srv::ResetOdrive::Response> response);
    void get_vbus_voltage_callback(const std::shared_ptr<ros2_odrive_can::srv::GetVbusVoltage::Request> request, std::shared_ptr<ros2_odrive_can::srv::GetVbusVoltage::Response> response);
    void clear_errors_callback(const std::shared_ptr<ros2_odrive_can::srv::ClearErrors::Request> request, std::shared_ptr<ros2_odrive_can::srv::ClearErrors::Response> response);
};