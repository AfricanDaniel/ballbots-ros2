/// @file
/// @brief Forward kinematics implementation for a mecanum wheel robot with robust ODrive startup.

#include <chrono>
#include <thread>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "minimec_msgs/msg/wheel_commands.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "minimeclib/kinematics.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class MinimecDriver : public rclcpp::Node
{
public:
  MinimecDriver()
  : Node("minimec_driver")
  {
    // ==========================================
    // 1. PARAMETER DECLARATION
    // ==========================================
    declare_parameter("rate", 50.0);
    declare_parameter("fl_joint_name", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fr_joint_name", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rr_joint_name", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rl_joint_name", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fl_feedback_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fr_feedback_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rr_feedback_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rl_feedback_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fl_control_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fr_control_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rr_control_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rl_control_topic", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fl_control_service", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("fr_control_service", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rr_control_service", rclcpp::ParameterType::PARAMETER_STRING);
    declare_parameter("rl_control_service", rclcpp::ParameterType::PARAMETER_STRING);

    // Load Parameters
    fl_joint_name = get_parameter("fl_joint_name").as_string();
    fr_joint_name = get_parameter("fr_joint_name").as_string();
    rr_joint_name = get_parameter("rr_joint_name").as_string();
    rl_joint_name = get_parameter("rl_joint_name").as_string();

    // Calculate Loop Rate
    std::chrono::milliseconds timer_step =
      (std::chrono::milliseconds)(int)(1000.0 / get_parameter("rate").as_double());

    // ==========================================
    // 2. SETUP PUBS / SUBS
    // ==========================================
    sub_wheel_cmd_ = create_subscription<minimec_msgs::msg::WheelCommands>(
      "wheel_cmd", 10, std::bind(&MinimecDriver::wheelCmdCallback, this, std::placeholders::_1));

    pub_fl_ = create_publisher<odrive_can::msg::ControlMessage>(get_parameter("fl_control_topic").as_string(), 10);
    pub_fr_ = create_publisher<odrive_can::msg::ControlMessage>(get_parameter("fr_control_topic").as_string(), 10);
    pub_rr_ = create_publisher<odrive_can::msg::ControlMessage>(get_parameter("rr_control_topic").as_string(), 10);
    pub_rl_ = create_publisher<odrive_can::msg::ControlMessage>(get_parameter("rl_control_topic").as_string(), 10);

    sub_fl_ = create_subscription<odrive_can::msg::ControllerStatus>(
      get_parameter("fl_feedback_topic").as_string(), 10,
      std::bind(&MinimecDriver::controllerCallbackFL, this, std::placeholders::_1));
    sub_fr_ = create_subscription<odrive_can::msg::ControllerStatus>(
      get_parameter("fr_feedback_topic").as_string(), 10,
      std::bind(&MinimecDriver::controllerCallbackFR, this, std::placeholders::_1));
    sub_rr_ = create_subscription<odrive_can::msg::ControllerStatus>(
      get_parameter("rr_feedback_topic").as_string(), 10,
      std::bind(&MinimecDriver::controllerCallbackRR, this, std::placeholders::_1));
    sub_rl_ = create_subscription<odrive_can::msg::ControllerStatus>(
      get_parameter("rl_feedback_topic").as_string(), 10,
      std::bind(&MinimecDriver::controllerCallbackRL, this, std::placeholders::_1));

    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // ==========================================
    // 3. SETUP SERVICE CLIENTS
    // ==========================================
    client_fl_ = create_client<odrive_can::srv::AxisState>(get_parameter("fl_control_service").as_string());
    client_fr_ = create_client<odrive_can::srv::AxisState>(get_parameter("fr_control_service").as_string());
    client_rr_ = create_client<odrive_can::srv::AxisState>(get_parameter("rr_control_service").as_string());
    client_rl_ = create_client<odrive_can::srv::AxisState>(get_parameter("rl_control_service").as_string());

    client_clear_fl_ = create_client<std_srvs::srv::Trigger>("/odrive_axis0/clear_errors");
    client_clear_fr_ = create_client<std_srvs::srv::Trigger>("/odrive_axis1/clear_errors");
    client_clear_rr_ = create_client<std_srvs::srv::Trigger>("/odrive_axis2/clear_errors");
    client_clear_rl_ = create_client<std_srvs::srv::Trigger>("/odrive_axis3/clear_errors");

    // Wait for services to be available (Blocking check at startup)
    waitForServices();

    // ==========================================
    // 4. START TIMER
    // ==========================================
    timer_ = create_wall_timer(
      timer_step, std::bind(&MinimecDriver::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Driver Initialized. Waiting for ODrives...");
  }

private:
  void waitForServices() {
      // Helper to wait for all 4 clients
      auto wait_for = [this](auto client) {
          while (!client->wait_for_service(1s)) {
              if (!rclcpp::ok()) {
                  RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                  return;
              }
              RCLCPP_INFO(this->get_logger(), "Waiting for ODrive services...");
          }
      };
      wait_for(client_fl_);
      wait_for(client_fr_);
      wait_for(client_rr_);
      wait_for(client_rl_);
  }

  void timerCallback()
  {
    // ==========================================
    // MODE A: RETRY LOGIC (Startup)
    // ==========================================
    if (odrives_enabled < 4) {
      // Run retry logic every 100 ticks (approx 2.0 seconds at 50Hz)
      if (retry_counter % 100 == 0) {
          RCLCPP_INFO(this->get_logger(), "Attempting to Enable ODrives (Current: %d/4)...", odrives_enabled);
          enableOdrives();
      }
      retry_counter++;
      return; // Skip normal control until motors are ready
    }

    // ==========================================
    // MODE B: NORMAL CONTROL (Running)
    // ==========================================
    else {
      auto msg = odrive_can::msg::ControlMessage{};
      msg.control_mode = 2; // VELOCITY_CONTROL
      msg.input_mode = 1;   // PASSTHROUGH
      msg.input_pos = 0.0;
      msg.input_torque = 0.0;

      // 1. Send Commands
      msg.input_vel = -wheel_speeds.fl;
      pub_fl_->publish(msg);

      msg.input_vel = wheel_speeds.fr;
      pub_fr_->publish(msg);

      msg.input_vel = wheel_speeds.rr;
      pub_rr_->publish(msg);

      msg.input_vel = -wheel_speeds.rl;
      pub_rl_->publish(msg);

      // 2. Publish Joint States
      sensor_msgs::msg::JointState joint_msg;
      joint_msg.header.stamp = get_clock()->now();
      joint_msg.name = {fl_joint_name, fr_joint_name, rr_joint_name, rl_joint_name};
      joint_msg.position = {-wheel_pos.fl, -wheel_pos.fr, -wheel_pos.rr, -wheel_pos.rl};

      pub_joint_states_->publish(joint_msg);
    }
  }

  void enableOdrives()
  {
    // ← ADD: Clear errors first
    auto clear_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_clear_fl_->async_send_request(clear_req);
    client_clear_fr_->async_send_request(clear_req);
    client_clear_rr_->async_send_request(clear_req);
    client_clear_rl_->async_send_request(clear_req);

    // ← ADD: Wait 500ms for clear to complete before sending state 1
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
    request->axis_requested_state = 8; // AXIS_STATE_CLOSED_LOOP_CONTROL

    // Robust Callback: Only increments if we aren't full yet
    auto callback = [this](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future) {
        try {
            future.get(); // Ensure no exceptions
            if (this->odrives_enabled < 4) {
                this->odrives_enabled++;
                RCLCPP_INFO(this->get_logger(), "ODrive confirmed Closed-Loop (%d/4)", this->odrives_enabled);
            }
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Failed to receive service response.");
        }
    };

    // Send requests
    client_fl_->async_send_request(request, callback);
    client_fr_->async_send_request(request, callback);
    client_rr_->async_send_request(request, callback);
    client_rl_->async_send_request(request, callback);
  }

  void wheelCmdCallback(const minimec_msgs::msg::WheelCommands & msg)
  {
    // translate wheel speeds to turns/s
    // 2.0 comes from gear ratio? 3.1415... is PI
    wheel_speeds = minimeclib::WheelSpeeds{
      msg.fl / 2.0 / 3.14159265358979323846,
      msg.fr / 2.0 / 3.14159265358979323846,
      msg.rr / 2.0 / 3.14159265358979323846,
      msg.rl / 2.0 / 3.14159265358979323846
    };
  }

  void controllerCallbackFL(const odrive_can::msg::ControllerStatus & msg) {
    wheel_pos.fl = msg.pos_estimate * 2.0 * 3.14159265358979323846;
  }
  void controllerCallbackFR(const odrive_can::msg::ControllerStatus & msg) {
    wheel_pos.fr = msg.pos_estimate * 2.0 * 3.14159265358979323846;
  }
  void controllerCallbackRR(const odrive_can::msg::ControllerStatus & msg) {
    wheel_pos.rr = msg.pos_estimate * 2.0 * 3.14159265358979323846;
  }
  void controllerCallbackRL(const odrive_can::msg::ControllerStatus & msg) {
    wheel_pos.rl = msg.pos_estimate * 2.0 * 3.14159265358979323846;
  }

  // --- MEMBER VARIABLES ---

  // State Tracking
  int retry_counter = 0;
  int odrives_enabled{0};

  // Robot State
  minimeclib::WheelPositions wheel_pos;
  minimeclib::WheelSpeeds wheel_speeds;

  // ROS Interfaces
  rclcpp::Subscription<minimec_msgs::msg::WheelCommands>::SharedPtr sub_wheel_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;

  // ODrive Control Publishers
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_fl_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_fr_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_rr_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr pub_rl_;

  // ODrive Feedback Subscribers
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub_fl_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub_fr_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub_rr_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr sub_rl_;

  // ODrive Service Clients
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_fl_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_fr_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_rr_;
  rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr client_rl_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_clear_fl_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_clear_fr_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_clear_rr_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_clear_rl_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Names
  std::string fl_joint_name;
  std::string fr_joint_name;
  std::string rr_joint_name;
  std::string rl_joint_name;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimecDriver>());
  rclcpp::shutdown();
  return 0;
}