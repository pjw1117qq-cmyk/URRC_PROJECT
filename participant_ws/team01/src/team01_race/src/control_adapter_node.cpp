#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "team01_race/control_math.hpp"

using SteadyClock = std::chrono::steady_clock;

class ControlAdapter : public rclcpp::Node
{
public:
  ControlAdapter() : Node("team01_control_adapter")
  {
    const auto requested_speed_topic = declare_parameter<std::string>(
      "requested_speed_topic", "/team01/internal/requested_speed");
    const auto requested_steering_topic = declare_parameter<std::string>(
      "requested_steering_topic", "/team01/internal/requested_steering");
    const auto start_topic = declare_parameter<std::string>("race_start_topic", "/race/start/team01");
    const auto speed_topic = declare_parameter<std::string>("speed_topic", "/team01/speed");
    const auto steering_topic = declare_parameter<std::string>("steering_topic", "/team01/steering");
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/team01/scan");
    const auto controller_ready_topic = declare_parameter<std::string>(
      "controller_ready_topic", "/team01/internal/controller_ready");
    wheelbase_ = declare_parameter<double>("control.wheelbase_m", 0.34);
    track_ = declare_parameter<double>("control.steering_track_m", 0.18);
    radius_ = declare_parameter<double>("control.wheel_radius_m", 0.04);
    rate_ = declare_parameter<double>("control.command_rate_hz", 50.0);
    command_timeout_ = declare_parameter<double>("safety.command_timeout_sec", 0.25);
    sensor_timeout_ = declare_parameter<double>("safety.sensor_timeout_sec", 0.25);
    max_speed_ = declare_parameter<double>("control.max_speed_mps", 6.0);
    max_steering_ = declare_parameter<double>("control.max_steering_angle_rad", 0.523599);
    acceleration_ = declare_parameter<double>("control.max_accel_mps2", 1.0);
    deceleration_ = declare_parameter<double>("control.max_decel_mps2", 2.5);
    steering_rate_ = declare_parameter<double>("control.max_steering_rate_radps", 1.5);
    if (wheelbase_ <= 0.0 || track_ <= 0.0 || radius_ <= 0.0 || rate_ <= 0.0 ||
      command_timeout_ <= 0.0 || sensor_timeout_ <= 0.0 || max_speed_ <= 0.0 ||
      max_steering_ <= 0.0 || max_steering_ > 0.523599 || acceleration_ <= 0.0 ||
      deceleration_ <= 0.0 || steering_rate_ <= 0.0) {
      throw std::invalid_argument("invalid control configuration");
    }

    speed_pub_ = create_publisher<std_msgs::msg::Float32>(speed_topic, 10);
    steering_pub_ = create_publisher<std_msgs::msg::Float32>(steering_topic, 10);
    final_pub_ = create_publisher<std_msgs::msg::Float32>("/team01/debug/final_speed", 10);
    requested_steering_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team01/debug/requested_steering", 10);
    reason_pub_ = create_publisher<std_msgs::msg::String>("/team01/debug/control_stop_reason", 10);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(controller_ready_topic, 10);
    rear_left_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/team01/actuator/rear_left_velocity", 10);
    rear_right_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/team01/actuator/rear_right_velocity", 10);
    front_left_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/team01/actuator/front_left_position", 10);
    front_right_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/team01/actuator/front_right_position", 10);

    requested_speed_sub_ = create_subscription<std_msgs::msg::Float32>(
      requested_speed_topic, 10, [this](std_msgs::msg::Float32::ConstSharedPtr message) {
        requested_speed_ = message->data;
        speed_received_ = true;
        last_speed_command_ = SteadyClock::now();
        publish();
      });
    requested_steering_sub_ = create_subscription<std_msgs::msg::Float32>(
      requested_steering_topic, 10, [this](std_msgs::msg::Float32::ConstSharedPtr message) {
        requested_steering_ = message->data;
        steering_received_ = true;
        last_steering_command_ = SteadyClock::now();
        publish();
      });
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      start_topic, rclcpp::QoS(10).reliable().durability_volatile(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        started_ = message->data;
        publish();
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr) {
        scan_received_ = true;
        last_scan_ = SteadyClock::now();
      });
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/team01/emergency_stop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        emergency_ = message->data;
        publish();
      });
    last_publish_ = SteadyClock::now();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_)),
      [this] {publish();});
  }

private:
  static double move_toward(double current, double target, double limit)
  {
    return current + std::clamp(target - current, -limit, limit);
  }

  static void send(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & publisher, double value)
  {
    std_msgs::msg::Float64 message;
    message.data = value;
    publisher->publish(message);
  }

  void publish()
  {
    const auto now = SteadyClock::now();
    const double dt = std::max(0.0, std::chrono::duration<double>(now - last_publish_).count());
    const bool command_stale = !speed_received_ || !steering_received_ ||
      std::chrono::duration<double>(now - last_speed_command_).count() > command_timeout_ ||
      std::chrono::duration<double>(now - last_steering_command_).count() > command_timeout_;
    const bool scan_stale = !scan_received_ ||
      std::chrono::duration<double>(now - last_scan_).count() > sensor_timeout_;
    const bool controller_ready = started_ && !emergency_ && !scan_stale;
    std_msgs::msg::Bool ready_message;
    ready_message.data = controller_ready;
    ready_pub_->publish(ready_message);
    std::string reason;
    if (emergency_) reason = "emergency_stop";
    else if (!started_) reason = "waiting_for_race_start";
    else if (scan_stale) reason = "lidar_timeout";
    else if (command_stale) reason = "command_timeout";
    else if (!std::isfinite(requested_speed_) || !std::isfinite(requested_steering_)) {
      reason = "invalid_command";
    }

    if (!reason.empty() || requested_speed_ <= 0.0) {
      speed_ = 0.0;
      steering_ = 0.0;
    } else {
      const double target_speed = std::clamp(requested_speed_, 0.0, max_speed_);
      const double target_steering = std::clamp(
        requested_steering_, -max_steering_, max_steering_);
      speed_ = move_toward(
        speed_, target_speed, (target_speed >= speed_ ? acceleration_ : deceleration_) * dt);
      steering_ = move_toward(steering_, target_steering, steering_rate_ * dt);
    }

    std_msgs::msg::Float32 output;
    output.data = static_cast<float>(speed_); speed_pub_->publish(output); final_pub_->publish(output);
    output.data = static_cast<float>(steering_); steering_pub_->publish(output);
    output.data = static_cast<float>(requested_steering_); requested_steering_pub_->publish(output);
    std_msgs::msg::String text;
    text.data = reason; reason_pub_->publish(text);

    const auto angles = team01_race::ackermann_joint_angles(
      steering_, wheelbase_, track_, max_steering_);
    const double wheel_velocity = speed_ / radius_;
    send(rear_left_pub_, wheel_velocity);
    send(rear_right_pub_, wheel_velocity);
    send(front_left_pub_, angles.first);
    send(front_right_pub_, angles.second);
    last_publish_ = now;
  }

  double wheelbase_, track_, radius_, rate_, command_timeout_, sensor_timeout_;
  double max_speed_, max_steering_, acceleration_, deceleration_, steering_rate_;
  double requested_speed_{0.0}, requested_steering_{0.0}, speed_{0.0}, steering_{0.0};
  bool speed_received_{false}, steering_received_{false}, scan_received_{false};
  bool started_{false}, emergency_{false};
  SteadyClock::time_point last_speed_command_{}, last_steering_command_{}, last_scan_{}, last_publish_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_, steering_pub_, final_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr requested_steering_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rear_left_pub_, rear_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_pub_, front_right_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr requested_speed_sub_, requested_steering_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_, emergency_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlAdapter>());
  rclcpp::shutdown();
}
