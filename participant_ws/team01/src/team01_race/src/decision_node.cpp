#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "team01_race/predictive_planner.hpp"

using SteadyClock = std::chrono::steady_clock;

class DecisionNode : public rclcpp::Node
{
public:
  DecisionNode()
  : Node("team01_decision"), config_(read_config()), planner_(config_)
  {
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/team01/internal/scan");
    const auto start_topic = declare_parameter<std::string>("race_start_topic", "/race/start/team01");
    const auto speed_topic = declare_parameter<std::string>(
      "requested_speed_topic", "/team01/internal/requested_speed");
    const auto steering_topic = declare_parameter<std::string>(
      "requested_steering_topic", "/team01/internal/requested_steering");
    const auto controller_ready_topic = declare_parameter<std::string>(
      "controller_ready_topic", "/team01/internal/controller_ready");
    sensor_timeout_sec_ = declare_parameter<double>("safety.sensor_timeout_sec", 0.25);

    speed_pub_ = create_publisher<std_msgs::msg::Float32>(speed_topic, 10);
    steering_pub_ = create_publisher<std_msgs::msg::Float32>(steering_topic, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/team01/debug/state", 10);
    reason_pub_ = create_publisher<std_msgs::msg::String>("/team01/debug/stop_reason", 10);
    requested_speed_debug_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team01/debug/requested_speed", 10);
    confidence_pub_ = create_publisher<std_msgs::msg::Float32>("/team01/debug/path_confidence", 10);
    curvature_pub_ = create_publisher<std_msgs::msg::Float32>("/team01/debug/maximum_curvature", 10);
    ttc_pub_ = create_publisher<std_msgs::msg::Float32>("/team01/debug/minimum_ttc", 10);
    lookahead_pub_ = create_publisher<std_msgs::msg::Float32>("/team01/debug/lookahead", 10);
    passing_pub_ = create_publisher<std_msgs::msg::String>("/team01/debug/passing_side", 10);
    centerline_lateral_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team01/debug/centerline_lateral", 10);
    raw_trajectory_lateral_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team01/debug/raw_trajectory_lateral", 10);
    filtered_trajectory_lateral_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team01/debug/filtered_trajectory_lateral", 10);
    target_x_pub_ = create_publisher<std_msgs::msg::Float32>("/team01/debug/target_x", 10);
    target_y_pub_ = create_publisher<std_msgs::msg::Float32>("/team01/debug/target_y", 10);
    predicted_curvature_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team01/debug/predicted_curvature", 10);
    steering_ff_pub_ = create_publisher<std_msgs::msg::Float32>("/team01/debug/steering_ff", 10);
    steering_fb_pub_ = create_publisher<std_msgs::msg::Float32>("/team01/debug/steering_fb", 10);
    desired_steering_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team01/debug/desired_steering", 10);
    selected_trajectory_pub_ = create_publisher<std_msgs::msg::String>(
      "/team01/debug/selected_trajectory", 10);
    raw_curvature_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team01/debug/raw_maximum_curvature", 10);
    raw_target_y_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/team01/debug/raw_target_y", 10);
    left_boundary_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/team01/debug/left_boundary_valid", 10);
    right_boundary_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/team01/debug/right_boundary_valid", 10);
    straight_guard_active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/team01/debug/straight_guard_active", 10);
    straight_guard_reason_pub_ = create_publisher<std_msgs::msg::String>(
      "/team01/debug/straight_guard_reason", 10);
    obstacle_count_pub_ = create_publisher<std_msgs::msg::Int32>(
      "/team01/debug/confirmed_obstacle_count", 10);
    started_pub_ = create_publisher<std_msgs::msg::Bool>("/team01/debug/race_started", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        const auto frame = team01_race::extract_predictive_frame(*scan, config_);
        latest_plan_ = planner_.plan(frame, steady_seconds());
        if (latest_plan_.straight_guard_rejected) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[STRAIGHT_GUARD] reason=%s raw_y=%.3f filtered_y=%.3f raw_target_y=%.3f "
            "validated_target_y=%.3f raw_curvature=%.3f validated_curvature=%.3f "
            "trajectory=%s obstacles=%d delta_ff=%.3f delta_fb=%.3f desired=%.3f final=%.3f",
            latest_plan_.straight_guard_reason.c_str(), latest_plan_.raw_centerline_lateral_m,
            latest_plan_.centerline_lateral_m, latest_plan_.raw_target_y_m,
            latest_plan_.target_y_m, latest_plan_.raw_maximum_curvature,
            latest_plan_.predicted_curvature,
            latest_plan_.passing_side == team01_race::PassingSide::NONE ? "CENTER" :
            team01_race::passing_side_name(latest_plan_.passing_side),
            latest_plan_.confirmed_obstacle_count, latest_plan_.steering_ff_rad,
            latest_plan_.steering_fb_rad, latest_plan_.desired_internal_steering_rad,
            latest_plan_.official_steering_rad);
        }
        scan_received_ = true;
        last_scan_ = SteadyClock::now();
        if (race_started_) fresh_trajectory_after_start_ = latest_plan_.valid;
        publish_command();
      });
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      start_topic, rclcpp::QoS(10).reliable().durability_volatile(),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (message->data && !race_started_) fresh_trajectory_after_start_ = false;
        race_started_ = message->data;
        if (!race_started_) fresh_trajectory_after_start_ = false;
        publish_command();
      });
    controller_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      controller_ready_topic, 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        controller_ready_ = message->data;
        publish_command();
      });
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/team01/emergency_stop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        external_emergency_ = message->data;
        publish_command();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {publish_command();});
    publish_command();
  }

private:
  template<typename T>
  T parameter(const char * name, T value) {return declare_parameter<T>(name, value);}

  team01_race::PlannerConfig read_config()
  {
    team01_race::PlannerConfig config;
#define PARAM(field, name) config.field = parameter<decltype(config.field)>(name, config.field)
    PARAM(planning_horizon_m, "planner.planning_horizon_m");
    PARAM(trajectory_spacing_m, "planner.trajectory_spacing_m");
    PARAM(temporal_history_sec, "planner.temporal_history_sec");
    PARAM(trajectory_temporal_alpha, "planner.trajectory_temporal_alpha");
    PARAM(cruise_speed_mps, "planner.cruise_speed_mps");
    PARAM(min_path_confidence, "planner.min_path_confidence");
    PARAM(max_lateral_accel_mps2, "planner.max_lateral_accel_mps2");
    PARAM(default_track_width_m, "planner.default_track_width_m");
    PARAM(min_track_width_m, "planner.min_track_width_m");
    PARAM(max_track_width_m, "planner.max_track_width_m");
    PARAM(min_lookahead_m, "control.min_lookahead_m");
    PARAM(max_lookahead_m, "control.max_lookahead_m");
    PARAM(wheelbase_m, "control.wheelbase_m");
    PARAM(max_steering_angle_rad, "control.max_steering_angle_rad");
    PARAM(heading_gain, "control.heading_gain");
    PARAM(lateral_gain, "control.lateral_gain");
    PARAM(curvature_alpha, "control.curvature_alpha");
    PARAM(lookahead_alpha, "control.lookahead_alpha");
    PARAM(steering_deadband_rad, "control.steering_deadband_rad");
    PARAM(straight_curvature_threshold, "straight_guard.curvature_threshold");
    PARAM(straight_heading_threshold_rad, "straight_guard.heading_threshold_rad");
    PARAM(straight_lateral_variation_m, "straight_guard.lateral_variation_m");
    PARAM(boundary_heading_difference_rad, "straight_guard.boundary_heading_difference_rad");
    PARAM(trajectory_jump_threshold_m, "straight_guard.trajectory_jump_threshold_m");
    PARAM(target_jump_threshold_m, "straight_guard.target_jump_threshold_m");
    PARAM(curvature_spike_threshold, "straight_guard.curvature_spike_threshold");
    PARAM(straight_enter_frames, "straight_guard.enter_frames");
    PARAM(straight_exit_frames, "straight_guard.exit_frames");
    PARAM(path_change_confirm_frames, "straight_guard.path_change_confirm_frames");
    PARAM(detection_distance_m, "obstacle.detection_distance_m");
    PARAM(min_vehicle_width_m, "obstacle.min_vehicle_width_m");
    PARAM(max_vehicle_width_m, "obstacle.max_vehicle_width_m");
    PARAM(min_vehicle_depth_m, "obstacle.min_vehicle_depth_m");
    PARAM(max_vehicle_depth_m, "obstacle.max_vehicle_depth_m");
    PARAM(obstacle_confirm_frames, "obstacle.confirm_frames");
    PARAM(vehicle_lateral_margin_m, "obstacle.vehicle_lateral_margin_m");
    PARAM(vehicle_longitudinal_margin_m, "obstacle.vehicle_longitudinal_margin_m");
    PARAM(ego_width_m, "obstacle.ego_width_m");
    PARAM(emergency_ttc_sec, "safety.emergency_ttc_sec");
    PARAM(caution_ttc_sec, "safety.caution_ttc_sec");
    PARAM(assumed_max_decel_mps2, "safety.assumed_max_decel_mps2");
    PARAM(min_valid_scan_beams, "perception.min_valid_scan_beams");
    PARAM(min_boundary_points, "perception.min_boundary_points");
    PARAM(cluster_gap_m, "perception.cluster_gap_m");
#undef PARAM
    config.validate();
    return config;
  }

  static double steady_seconds()
  {
    return std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
  }

  void publish_command()
  {
    const bool stale = !scan_received_ ||
      std::chrono::duration<double>(SteadyClock::now() - last_scan_).count() > sensor_timeout_sec_;
    std::string state = "WAIT_START";
    std::string reason = "waiting_for_race_start";
    double speed = 0.0;
    double steering = 0.0;

    if (external_emergency_ || latest_plan_.emergency_stop) {
      state = "EMERGENCY_STOP";
      reason = external_emergency_ ? "external_emergency_stop" : latest_plan_.reason;
    } else if (!race_started_ || !fresh_trajectory_after_start_ || !controller_ready_) {
      state = "WAIT_START";
      if (!race_started_) reason = "waiting_for_race_start";
      else if (!fresh_trajectory_after_start_) reason = "waiting_for_fresh_trajectory";
      else reason = "waiting_for_controller_ready";
    } else if (stale || !latest_plan_.valid) {
      state = "FAULT";
      reason = stale ? "lidar_timeout" : latest_plan_.reason;
    } else {
      state = "RUNNING";
      reason = latest_plan_.reason;
      speed = latest_plan_.target_speed_mps;
      steering = latest_plan_.official_steering_rad;
    }

    std_msgs::msg::Float32 number;
    number.data = static_cast<float>(speed); speed_pub_->publish(number);
    number.data = static_cast<float>(steering); steering_pub_->publish(number);
    number.data = static_cast<float>(speed); requested_speed_debug_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.confidence); confidence_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.maximum_curvature); curvature_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.minimum_ttc_sec); ttc_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.lookahead_m); lookahead_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.centerline_lateral_m);
    centerline_lateral_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.raw_centerline_lateral_m);
    raw_trajectory_lateral_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.centerline_lateral_m);
    filtered_trajectory_lateral_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.target_x_m); target_x_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.target_y_m); target_y_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.predicted_curvature);
    predicted_curvature_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.steering_ff_rad); steering_ff_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.steering_fb_rad); steering_fb_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.desired_internal_steering_rad);
    desired_steering_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.raw_maximum_curvature);
    raw_curvature_pub_->publish(number);
    number.data = static_cast<float>(latest_plan_.raw_target_y_m);
    raw_target_y_pub_->publish(number);
    std_msgs::msg::String text;
    text.data = state; state_pub_->publish(text);
    text.data = reason; reason_pub_->publish(text);
    text.data = team01_race::passing_side_name(latest_plan_.passing_side); passing_pub_->publish(text);
    text.data = latest_plan_.passing_side == team01_race::PassingSide::NONE ?
      "CENTER" : std::string(team01_race::passing_side_name(latest_plan_.passing_side)) + "_PASS";
    selected_trajectory_pub_->publish(text);
    text.data = latest_plan_.straight_guard_reason;
    straight_guard_reason_pub_->publish(text);
    std_msgs::msg::Bool started;
    started.data = race_started_; started_pub_->publish(started);
    started.data = latest_plan_.left_boundary_valid; left_boundary_valid_pub_->publish(started);
    started.data = latest_plan_.right_boundary_valid; right_boundary_valid_pub_->publish(started);
    started.data = latest_plan_.straight_guard_active; straight_guard_active_pub_->publish(started);
    std_msgs::msg::Int32 count;
    count.data = latest_plan_.confirmed_obstacle_count; obstacle_count_pub_->publish(count);
  }

  team01_race::PlannerConfig config_;
  team01_race::PredictivePlanner planner_;
  team01_race::Plan latest_plan_;
  double sensor_timeout_sec_{0.25};
  bool scan_received_{false};
  bool race_started_{false};
  bool fresh_trajectory_after_start_{false};
  bool external_emergency_{false};
  bool controller_ready_{false};
  SteadyClock::time_point last_scan_{};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_, steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr requested_speed_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr confidence_pub_, curvature_pub_, ttc_pub_, lookahead_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr centerline_lateral_pub_, target_x_pub_, target_y_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_trajectory_lateral_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr filtered_trajectory_lateral_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr predicted_curvature_pub_, steering_ff_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_fb_pub_, desired_steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr raw_curvature_pub_, raw_target_y_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, reason_pub_, passing_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr straight_guard_reason_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr started_pub_, left_boundary_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_boundary_valid_pub_, straight_guard_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr obstacle_count_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_, controller_ready_sub_, emergency_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecisionNode>());
  rclcpp::shutdown();
}
