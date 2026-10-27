#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace team01_race
{
constexpr double kPi = 3.14159265358979323846;

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct Cluster
{
  Point2 centroid;
  double width{0.0};
  double depth{0.0};
  bool boundary_connected{false};
};

struct BoundaryFrame
{
  bool scan_valid{false};
  std::size_t valid_beams{0};
  std::vector<Point2> left;
  std::vector<Point2> right;
  std::vector<Cluster> obstacle_candidates;
};

struct TrajectoryPoint
{
  double x{0.0};
  double y{0.0};
  double heading{0.0};
  double curvature{0.0};
  double clearance{0.0};
  double confidence{0.0};
};

enum class PassingSide {NONE, LEFT, RIGHT};
inline const char * passing_side_name(PassingSide side)
{
  switch (side) {
    case PassingSide::LEFT: return "LEFT";
    case PassingSide::RIGHT: return "RIGHT";
    case PassingSide::NONE: return "NONE";
  }
  return "NONE";
}

struct PlannerConfig
{
  double planning_horizon_m{5.0};
  double trajectory_spacing_m{0.10};
  double temporal_history_sec{0.5};
  double cruise_speed_mps{3.5};
  double min_path_confidence{0.40};
  double max_lateral_accel_mps2{3.0};
  double default_track_width_m{1.5};
  double min_track_width_m{0.55};
  double max_track_width_m{4.0};
  double min_lookahead_m{1.0};
  double max_lookahead_m{3.0};
  double wheelbase_m{0.34};
  double max_steering_angle_rad{0.55};
  double heading_gain{0.45};
  double lateral_gain{0.35};
  double detection_distance_m{5.0};
  double min_vehicle_width_m{0.12};
  double max_vehicle_width_m{0.80};
  double min_vehicle_depth_m{0.10};
  double max_vehicle_depth_m{1.20};
  int obstacle_confirm_frames{3};
  double vehicle_lateral_margin_m{0.15};
  double vehicle_longitudinal_margin_m{0.30};
  double ego_width_m{0.2432};
  double emergency_ttc_sec{0.6};
  double caution_ttc_sec{1.5};
  double assumed_max_decel_mps2{2.5};
  int min_valid_scan_beams{20};
  int min_boundary_points{3};
  double cluster_gap_m{0.14};

  void validate() const
  {
    if (!(planning_horizon_m > 0.0 && trajectory_spacing_m > 0.0 &&
      trajectory_spacing_m <= planning_horizon_m && temporal_history_sec > 0.0 &&
      cruise_speed_mps > 0.0 && min_path_confidence > 0.0 && min_path_confidence <= 1.0 &&
      max_lateral_accel_mps2 > 0.0 && default_track_width_m >= min_track_width_m &&
      default_track_width_m <= max_track_width_m && min_lookahead_m > 0.0 &&
      max_lookahead_m >= min_lookahead_m && max_lookahead_m <= planning_horizon_m &&
      wheelbase_m > 0.0 && max_steering_angle_rad > 0.0 &&
      max_steering_angle_rad <= 0.6108652382 && detection_distance_m > 0.0 &&
      detection_distance_m <= planning_horizon_m && min_vehicle_width_m > 0.0 &&
      max_vehicle_width_m >= min_vehicle_width_m && min_vehicle_depth_m > 0.0 &&
      max_vehicle_depth_m >= min_vehicle_depth_m && obstacle_confirm_frames > 0 &&
      ego_width_m > 0.0 && emergency_ttc_sec > 0.0 &&
      caution_ttc_sec > emergency_ttc_sec && assumed_max_decel_mps2 > 0.0 &&
      min_valid_scan_beams > 0 && min_boundary_points > 0 && cluster_gap_m > 0.0)) {
      throw std::invalid_argument("invalid predictive planner configuration");
    }
  }
};

struct Plan
{
  bool valid{false};
  bool emergency_stop{false};
  double target_speed_mps{0.0};
  double official_steering_rad{0.0};
  double confidence{0.0};
  double lookahead_m{0.0};
  double maximum_curvature{0.0};
  double minimum_ttc_sec{std::numeric_limits<double>::infinity()};
  PassingSide passing_side{PassingSide::NONE};
  bool obstacle_confirmed{false};
  std::string reason{"trajectory_unavailable"};
  std::vector<TrajectoryPoint> trajectory;
};

inline double distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline double median(std::vector<double> values)
{
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  double result = values[middle];
  if (values.size() % 2 == 0) {
    const auto lower = std::max_element(values.begin(), values.begin() + middle);
    result = 0.5 * (result + *lower);
  }
  return result;
}

inline BoundaryFrame extract_predictive_frame(
  const sensor_msgs::msg::LaserScan & scan, const PlannerConfig & config)
{
  BoundaryFrame frame;
  if (scan.ranges.empty() || !std::isfinite(scan.angle_min) ||
    !std::isfinite(scan.angle_increment) || scan.angle_increment <= 0.0 ||
    !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
    scan.range_min < 0.0 || scan.range_max <= scan.range_min) {
    return frame;
  }

  std::vector<Point2> points;
  points.reserve(scan.ranges.size());
  const double usable_range = std::min(config.planning_horizon_m, static_cast<double>(scan.range_max));
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (angle < -kPi / 2.0 || angle > kPi / 2.0) continue;
    const double range = scan.ranges[i];
    if (std::isinf(range) && range > 0.0) {
      ++frame.valid_beams;
      continue;
    }
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) continue;
    ++frame.valid_beams;
    if (range > usable_range) continue;
    const Point2 point{range * std::cos(angle), range * std::sin(angle)};
    if (point.x >= 0.0 && point.x <= config.planning_horizon_m) points.push_back(point);
  }
  frame.scan_valid = frame.valid_beams >= static_cast<std::size_t>(config.min_valid_scan_beams);
  if (!frame.scan_valid) return frame;

  std::vector<std::vector<Point2>> groups;
  for (const auto & point : points) {
    if (groups.empty() || distance(groups.back().back(), point) > config.cluster_gap_m) {
      groups.push_back({});
    }
    groups.back().push_back(point);
  }

  for (const auto & group : groups) {
    if (group.empty()) continue;
    double min_x = group.front().x, max_x = group.front().x;
    double min_y = group.front().y, max_y = group.front().y;
    Point2 centroid;
    for (const auto & point : group) {
      min_x = std::min(min_x, point.x); max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y); max_y = std::max(max_y, point.y);
      centroid.x += point.x; centroid.y += point.y;
    }
    centroid.x /= static_cast<double>(group.size());
    centroid.y /= static_cast<double>(group.size());
    const double width = max_y - min_y;
    const double depth = max_x - min_x;
    const bool compact = group.size() >= 3 && width >= config.min_vehicle_width_m &&
      width <= config.max_vehicle_width_m && depth >= config.min_vehicle_depth_m &&
      depth <= config.max_vehicle_depth_m;
    const bool track_interior_candidate = compact && std::abs(centroid.y) < 0.8 * config.default_track_width_m;
    if (track_interior_candidate && centroid.x <= config.detection_distance_m) {
      frame.obstacle_candidates.push_back({centroid, width, depth, false});
    }
    for (const auto & point : group) {
      if (point.y > 0.12) frame.left.push_back(point);
      else if (point.y < -0.12) frame.right.push_back(point);
    }
  }
  return frame;
}

class PredictivePlanner
{
public:
  explicit PredictivePlanner(PlannerConfig config)
  : config_(std::move(config)), estimated_track_width_(config_.default_track_width_m)
  {
    config_.validate();
  }

  const PlannerConfig & config() const {return config_;}

  Plan plan(const BoundaryFrame & frame, double now_sec)
  {
    Plan result;
    if (!frame.scan_valid) {
      return continue_previous(now_sec, "invalid_lidar");
    }

    update_obstacle_track(frame.obstacle_candidates, now_sec);
    const std::size_t count = static_cast<std::size_t>(
      std::floor(config_.planning_horizon_m / config_.trajectory_spacing_m));
    if (count < 3) return result;

    auto left = frame.left.size() >= static_cast<std::size_t>(config_.min_boundary_points) ?
      sorted(frame.left) : std::vector<Point2>{};
    auto right = frame.right.size() >= static_cast<std::size_t>(config_.min_boundary_points) ?
      sorted(frame.right) : std::vector<Point2>{};
    std::vector<double> observed_widths;
    std::vector<TrajectoryPoint> path;
    path.reserve(count);
    double confidence_sum = 0.0;

    for (std::size_t i = 1; i <= count; ++i) {
      const double x = static_cast<double>(i) * config_.trajectory_spacing_m;
      const auto left_y = interpolate(left, x);
      const auto right_y = interpolate(right, x);
      double y = 0.0;
      double point_confidence = 0.0;
      double clearance = estimated_track_width_ / 2.0;
      if (left_y && right_y && *left_y > *right_y) {
        const double width = *left_y - *right_y;
        if (width >= config_.min_track_width_m && width <= config_.max_track_width_m) {
          observed_widths.push_back(width);
          y = 0.5 * (*left_y + *right_y);
          clearance = 0.5 * width;
          point_confidence = 1.0;
        }
      }
      if (point_confidence == 0.0 && left_y) {
        y = *left_y - estimated_track_width_ / 2.0;
        clearance = estimated_track_width_ / 2.0;
        point_confidence = 0.68;
      } else if (point_confidence == 0.0 && right_y) {
        y = *right_y + estimated_track_width_ / 2.0;
        clearance = estimated_track_width_ / 2.0;
        point_confidence = 0.68;
      }
      if (point_confidence == 0.0 && previous_valid_ &&
        now_sec - previous_time_sec_ <= config_.temporal_history_sec && i - 1 < previous_.size()) {
        const double age_ratio = std::clamp(
          (now_sec - previous_time_sec_) / config_.temporal_history_sec, 0.0, 1.0);
        y = previous_[i - 1].y;
        clearance = previous_[i - 1].clearance;
        point_confidence = previous_[i - 1].confidence * (1.0 - 0.65 * age_ratio);
      }
      path.push_back({x, y, 0.0, 0.0, clearance, point_confidence});
      confidence_sum += point_confidence;
    }

    if (!observed_widths.empty()) {
      const double width = median(observed_widths);
      estimated_track_width_ = std::clamp(
        0.8 * estimated_track_width_ + 0.2 * width,
        config_.min_track_width_m, config_.max_track_width_m);
    }

    fill_unobserved(path);
    result.confidence = confidence_sum / static_cast<double>(count);
    if (result.confidence < config_.min_path_confidence) {
      return continue_previous(now_sec, "low_trajectory_confidence");
    }

    // Exponential temporal fusion makes the retained previous path represent several
    // recent LiDAR frames over the configured history window, suppressing one-frame noise.
    if (previous_valid_ && now_sec >= previous_time_sec_ &&
      now_sec - previous_time_sec_ <= config_.temporal_history_sec && previous_.size() == path.size()) {
      const double age_ratio = std::clamp(
        (now_sec - previous_time_sec_) / config_.temporal_history_sec, 0.0, 1.0);
      const double history_weight = 0.35 * (1.0 - age_ratio);
      for (std::size_t i = 0; i < path.size(); ++i) {
        path[i].y = (1.0 - history_weight) * path[i].y + history_weight * previous_[i].y;
        path[i].clearance = (1.0 - history_weight) * path[i].clearance +
          history_weight * previous_[i].clearance;
      }
    }

    smooth(path);
    compute_geometry(path);
    apply_confirmed_obstacle(path, frame, result);
    compute_geometry(path);
    finish_control(path, result);
    result.valid = !result.emergency_stop;
    result.reason = result.emergency_stop ? "imminent_trajectory_collision" : "predicted_trajectory_ready";
    result.trajectory = path;

    if (!result.emergency_stop) {
      previous_ = path;
      previous_time_sec_ = now_sec;
      previous_valid_ = true;
      previous_speed_mps_ = result.target_speed_mps;
    }
    return result;
  }

private:
  struct ObstacleTrack
  {
    Cluster cluster;
    Point2 previous_centroid;
    double last_time_sec{0.0};
    double closing_speed_mps{0.0};
    int confirmations{0};
    bool active{false};
  };

  static std::vector<Point2> sorted(std::vector<Point2> points)
  {
    std::sort(points.begin(), points.end(), [](const Point2 & a, const Point2 & b) {
      return a.x < b.x;
    });
    return points;
  }

  std::optional<double> interpolate(const std::vector<Point2> & points, double x) const
  {
    if (points.empty()) return std::nullopt;
    const auto upper = std::lower_bound(points.begin(), points.end(), x,
      [](const Point2 & point, double value) {return point.x < value;});
    if (upper == points.begin()) {
      if (std::abs(upper->x - x) <= 0.35) return upper->y;
      return std::nullopt;
    }
    if (upper == points.end()) {
      const auto & last = points.back();
      if (std::abs(last.x - x) <= 0.35) return last.y;
      return std::nullopt;
    }
    const auto & b = *upper;
    const auto & a = *(upper - 1);
    if (b.x - a.x > 0.8) return std::nullopt;
    const double ratio = (x - a.x) / std::max(1e-6, b.x - a.x);
    return a.y + ratio * (b.y - a.y);
  }

  static void fill_unobserved(std::vector<TrajectoryPoint> & path)
  {
    for (std::size_t i = 0; i < path.size(); ++i) {
      if (path[i].confidence > 0.0) continue;
      std::optional<std::size_t> before, after;
      for (std::size_t j = i; j > 0; --j) {
        if (path[j - 1].confidence > 0.0) {before = j - 1; break;}
      }
      for (std::size_t j = i + 1; j < path.size(); ++j) {
        if (path[j].confidence > 0.0) {after = j; break;}
      }
      if (before && after) {
        const double ratio = (path[i].x - path[*before].x) /
          std::max(1e-6, path[*after].x - path[*before].x);
        path[i].y = path[*before].y + ratio * (path[*after].y - path[*before].y);
        path[i].clearance = std::min(path[*before].clearance, path[*after].clearance);
      } else if (before) {
        double slope = 0.0;
        if (*before > 0 && path[*before - 1].confidence > 0.0) {
          slope = (path[*before].y - path[*before - 1].y) /
            std::max(1e-6, path[*before].x - path[*before - 1].x);
        }
        path[i].y = path[*before].y + slope * (path[i].x - path[*before].x);
        path[i].clearance = path[*before].clearance;
      } else if (after) {
        double slope = 0.0;
        if (*after + 1 < path.size() && path[*after + 1].confidence > 0.0) {
          slope = (path[*after + 1].y - path[*after].y) /
            std::max(1e-6, path[*after + 1].x - path[*after].x);
        }
        path[i].y = path[*after].y - slope * (path[*after].x - path[i].x);
        path[i].clearance = path[*after].clearance;
      }
    }
  }

  static void smooth(std::vector<TrajectoryPoint> & path)
  {
    if (path.size() < 5) return;
    std::vector<double> filtered(path.size());
    for (std::size_t i = 0; i < path.size(); ++i) {
      double weighted = 0.0, weight = 0.0;
      for (int offset = -2; offset <= 2; ++offset) {
        const int index = static_cast<int>(i) + offset;
        if (index < 0 || index >= static_cast<int>(path.size())) continue;
        const double w = offset == 0 ? 3.0 : (std::abs(offset) == 1 ? 2.0 : 1.0);
        weighted += w * path[static_cast<std::size_t>(index)].y;
        weight += w;
      }
      filtered[i] = weighted / weight;
    }
    for (std::size_t i = 0; i < path.size(); ++i) path[i].y = filtered[i];
  }

  static void compute_geometry(std::vector<TrajectoryPoint> & path)
  {
    if (path.size() < 3) return;
    for (std::size_t i = 0; i < path.size(); ++i) {
      const std::size_t before = i == 0 ? 0 : i - 1;
      const std::size_t after = i + 1 >= path.size() ? path.size() - 1 : i + 1;
      const double dx = std::max(1e-6, path[after].x - path[before].x);
      const double derivative = (path[after].y - path[before].y) / dx;
      path[i].heading = std::atan(derivative);
    }
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      const double dx1 = path[i].x - path[i - 1].x;
      const double dx2 = path[i + 1].x - path[i].x;
      const double d1 = (path[i].y - path[i - 1].y) / std::max(1e-6, dx1);
      const double d2 = (path[i + 1].y - path[i].y) / std::max(1e-6, dx2);
      const double second = 2.0 * (d2 - d1) / std::max(1e-6, dx1 + dx2);
      const double first = 0.5 * (d1 + d2);
      path[i].curvature = second / std::pow(1.0 + first * first, 1.5);
    }
    path.front().curvature = path[1].curvature;
    path.back().curvature = path[path.size() - 2].curvature;
  }

  void update_obstacle_track(const std::vector<Cluster> & candidates, double now_sec)
  {
    if (candidates.empty()) {
      if (obstacle_.active && now_sec - obstacle_.last_time_sec > config_.temporal_history_sec) {
        obstacle_ = ObstacleTrack{};
      }
      return;
    }
    const auto closest = std::min_element(candidates.begin(), candidates.end(),
      [](const Cluster & a, const Cluster & b) {return a.centroid.x < b.centroid.x;});
    if (closest->boundary_connected || closest->centroid.x <= 0.0 ||
      closest->centroid.x > config_.detection_distance_m ||
      closest->width < config_.min_vehicle_width_m || closest->width > config_.max_vehicle_width_m ||
      closest->depth < config_.min_vehicle_depth_m || closest->depth > config_.max_vehicle_depth_m) {
      return;
    }
    if (obstacle_.active && distance(obstacle_.cluster.centroid, closest->centroid) < 0.6 &&
      now_sec > obstacle_.last_time_sec) {
      const double dt = now_sec - obstacle_.last_time_sec;
      obstacle_.closing_speed_mps = std::max(
        0.0, (obstacle_.cluster.centroid.x - closest->centroid.x) / dt);
      ++obstacle_.confirmations;
    } else {
      obstacle_.confirmations = 1;
      obstacle_.closing_speed_mps = 0.0;
    }
    obstacle_.previous_centroid = obstacle_.cluster.centroid;
    obstacle_.cluster = *closest;
    obstacle_.last_time_sec = now_sec;
    obstacle_.active = true;
  }

  double path_y_at(const std::vector<TrajectoryPoint> & path, double x) const
  {
    const auto it = std::min_element(path.begin(), path.end(), [x](const auto & a, const auto & b) {
      return std::abs(a.x - x) < std::abs(b.x - x);
    });
    return it == path.end() ? 0.0 : it->y;
  }

  void apply_confirmed_obstacle(
    std::vector<TrajectoryPoint> & path, const BoundaryFrame & frame, Plan & result)
  {
    if (!obstacle_.active || obstacle_.confirmations < config_.obstacle_confirm_frames) return;
    const Cluster & obstacle = obstacle_.cluster;
    result.obstacle_confirmed = true;
    const double current_path_y = path_y_at(path, obstacle.centroid.x);
    const double collision_half_width = obstacle.width / 2.0 + config_.ego_width_m / 2.0 +
      config_.vehicle_lateral_margin_m;
    if (std::abs(current_path_y - obstacle.centroid.y) > collision_half_width) return;

    const auto left_points = sorted(frame.left);
    const auto right_points = sorted(frame.right);
    const auto left_y = interpolate(left_points, obstacle.centroid.x);
    const auto right_y = interpolate(right_points, obstacle.centroid.x);
    const double center_need = config_.ego_width_m / 2.0 + config_.vehicle_lateral_margin_m;
    const double left_limit = left_y.value_or(estimated_track_width_ / 2.0) - center_need;
    const double right_limit = right_y.value_or(-estimated_track_width_ / 2.0) + center_need;
    const double left_target = obstacle.centroid.y + obstacle.width / 2.0 + center_need + 0.02;
    const double right_target = obstacle.centroid.y - obstacle.width / 2.0 - center_need - 0.02;
    const bool left_possible = left_target <= left_limit;
    const bool right_possible = right_target >= right_limit;

    double target_y = current_path_y;
    if (left_possible && (!right_possible || left_limit - left_target >= right_target - right_limit)) {
      result.passing_side = PassingSide::LEFT;
      target_y = left_target;
    } else if (right_possible) {
      result.passing_side = PassingSide::RIGHT;
      target_y = right_target;
    }

    const double closing_speed = std::max(
      0.2, std::max(previous_speed_mps_, config_.cruise_speed_mps) + obstacle_.closing_speed_mps);
    result.minimum_ttc_sec = std::max(0.0,
      obstacle.centroid.x - obstacle.depth / 2.0 - config_.vehicle_longitudinal_margin_m) /
      closing_speed;

    if (result.passing_side != PassingSide::NONE) {
      const double offset = target_y - current_path_y;
      const double spread = std::max(0.8, std::min(1.5, obstacle.centroid.x * 0.45));
      for (auto & point : path) {
        const double normalized = (point.x - obstacle.centroid.x) / spread;
        point.y += offset * std::exp(-0.5 * normalized * normalized);
        point.confidence *= 0.85;
      }
      return;
    }

    const double stopping_distance = previous_speed_mps_ * previous_speed_mps_ /
      (2.0 * config_.assumed_max_decel_mps2);
    const double available = obstacle.centroid.x - obstacle.depth / 2.0 -
      config_.vehicle_longitudinal_margin_m;
    if (result.minimum_ttc_sec <= config_.emergency_ttc_sec || available <= stopping_distance) {
      result.emergency_stop = true;
    }
  }

  void finish_control(const std::vector<TrajectoryPoint> & path, Plan & result) const
  {
    result.maximum_curvature = 0.0;
    for (const auto & point : path) {
      if (point.confidence >= config_.min_path_confidence) {
        result.maximum_curvature = std::max(result.maximum_curvature, std::abs(point.curvature));
      }
    }
    const double curve_ratio = std::clamp(result.maximum_curvature / 0.45, 0.0, 1.0);
    result.lookahead_m = config_.max_lookahead_m -
      curve_ratio * (config_.max_lookahead_m - config_.min_lookahead_m);
    const auto target = std::min_element(path.begin(), path.end(), [this, &result](const auto & a, const auto & b) {
      return std::abs(a.x - result.lookahead_m) < std::abs(b.x - result.lookahead_m);
    });
    if (target == path.end()) return;

    double future_curvature = 0.0, weight = 0.0;
    for (const auto & point : path) {
      if (point.x < result.lookahead_m * 0.7) continue;
      const double w = 1.0 / (1.0 + point.x);
      future_curvature += w * point.curvature;
      weight += w;
    }
    future_curvature = weight > 0.0 ? future_curvature / weight : target->curvature;
    const double predicted_curvature = 0.65 * target->curvature + 0.35 * future_curvature;
    const double feed_forward = std::atan(config_.wheelbase_m * predicted_curvature);
    const double feedback = config_.heading_gain * target->heading +
      config_.lateral_gain * target->y / std::max(0.2, result.lookahead_m);
    double passing_anticipation = 0.0;
    if (result.passing_side != PassingSide::NONE) {
      const auto lateral_goal = std::max_element(path.begin(), path.end(), [](const auto & a, const auto & b) {
        return std::abs(a.y) < std::abs(b.y);
      });
      if (lateral_goal != path.end()) {
        passing_anticipation = 0.5 * std::atan2(
          lateral_goal->y, std::max(0.5, lateral_goal->x));
      }
    }
    const double internal_left_positive = feed_forward + feedback + passing_anticipation;
    result.official_steering_rad = std::clamp(
      -internal_left_positive, -config_.max_steering_angle_rad, config_.max_steering_angle_rad);

    double curvature_speed = config_.cruise_speed_mps;
    if (result.maximum_curvature > 1e-4) {
      curvature_speed = std::sqrt(config_.max_lateral_accel_mps2 / result.maximum_curvature);
    }
    result.target_speed_mps = std::min(config_.cruise_speed_mps, curvature_speed);
    result.target_speed_mps *= std::clamp(
      (result.confidence - config_.min_path_confidence) /
      std::max(1e-6, 1.0 - config_.min_path_confidence), 0.25, 1.0);
    if (std::isfinite(result.minimum_ttc_sec) && result.minimum_ttc_sec < config_.caution_ttc_sec) {
      result.target_speed_mps *= std::clamp(
        result.minimum_ttc_sec / config_.caution_ttc_sec, 0.2, 1.0);
    }
    if (result.emergency_stop) {
      result.target_speed_mps = 0.0;
      result.official_steering_rad = 0.0;
    }
  }

  Plan continue_previous(double now_sec, const std::string & reason)
  {
    Plan result;
    if (!previous_valid_) {
      result.reason = reason;
      return result;
    }
    const double age = now_sec - previous_time_sec_;
    if (age < 0.0 || age > config_.temporal_history_sec) {
      previous_valid_ = false;
      result.reason = reason;
      return result;
    }
    const double decay = std::clamp(1.0 - age / config_.temporal_history_sec, 0.0, 1.0);
    result.trajectory = previous_;
    for (auto & point : result.trajectory) point.confidence *= decay;
    result.confidence = std::accumulate(
      result.trajectory.begin(), result.trajectory.end(), 0.0,
      [](double sum, const auto & point) {return sum + point.confidence;}) /
      static_cast<double>(result.trajectory.size());
    if (result.confidence < config_.min_path_confidence) {
      result.reason = "trajectory_history_expired";
      return result;
    }
    compute_geometry(result.trajectory);
    finish_control(result.trajectory, result);
    result.valid = true;
    result.reason = "temporal_trajectory_continuation";
    return result;
  }

  PlannerConfig config_;
  double estimated_track_width_;
  std::vector<TrajectoryPoint> previous_;
  double previous_time_sec_{0.0};
  double previous_speed_mps_{0.0};
  bool previous_valid_{false};
  ObstacleTrack obstacle_;
};
}  // namespace team01_race
