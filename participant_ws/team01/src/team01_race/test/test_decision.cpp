#include <gtest/gtest.h>
#include <cmath>
#include "team01_race/control_math.hpp"
#include "team01_race/predictive_planner.hpp"

using team01_race::BoundaryFrame;
using team01_race::Cluster;
using team01_race::PassingSide;
using team01_race::Point2;
using team01_race::PredictivePlanner;

namespace {
BoundaryFrame boundaries(double curve = 0.0, bool left = true, bool right = true,
  double half_width = 0.75, double lateral_offset = 0.0)
{
  BoundaryFrame frame;
  frame.scan_valid = true;
  frame.valid_beams = 181;
  for (int i = 1; i <= 50; ++i) {
    const double x = i * 0.1;
    const double center = lateral_offset + curve * x * x;
    if (left) frame.left.push_back({x, center + half_width});
    if (right) frame.right.push_back({x, center - half_width});
  }
  return frame;
}

Cluster vehicle(double x, double y, double width = 0.30, double depth = 0.30)
{
  return {{x, y}, width, depth, false};
}

team01_race::Plan confirm_obstacle(PredictivePlanner & planner, BoundaryFrame frame)
{
  planner.plan(frame, 0.0);
  planner.plan(frame, 0.1);
  return planner.plan(frame, 0.2);
}
}  // namespace

TEST(PredictiveTrajectory, StraightTargetsCruiseSpeedAndZeroSteering)
{
  PredictivePlanner planner({});
  const auto result = planner.plan(boundaries(), 0.0);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.official_steering_rad, 0.0, 1e-6);
  EXPECT_NEAR(result.target_speed_mps, 5.0, 1e-6);
  ASSERT_FALSE(result.trajectory.empty());
  EXPECT_NEAR(result.trajectory.back().x, 5.0, 1e-6);
}

TEST(PredictiveTrajectory, RightCurveProducesEarlyOfficialRightSteering)
{
  PredictivePlanner planner({});
  const auto result = planner.plan(boundaries(-0.025), 0.0);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.official_steering_rad, 0.0);
  EXPECT_LT(result.lookahead_m, 3.0);
}

TEST(PredictiveTrajectory, LeftCurveProducesEarlyOfficialLeftSteering)
{
  PredictivePlanner planner({});
  const auto result = planner.plan(boundaries(0.025), 0.0);
  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.official_steering_rad, 0.0);
}

TEST(PredictiveTrajectory, OneBoundaryReconstructsCenterlineWithTrackWidth)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  const auto result = planner.plan(boundaries(0.0, true, false), 0.1);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.target_speed_mps, 0.0);
  EXPECT_GT(result.confidence, 0.40);
}

TEST(PredictiveTrajectory, BriefLossContinuesPreviousTrajectoryWithoutSearchState)
{
  PredictivePlanner planner({});
  const auto initial = planner.plan(boundaries(-0.015), 0.0);
  ASSERT_TRUE(initial.valid);
  BoundaryFrame missing;
  missing.scan_valid = true;
  missing.valid_beams = 181;
  const auto continued = planner.plan(missing, 0.1);
  ASSERT_TRUE(continued.valid);
  EXPECT_EQ(continued.reason, "predicted_trajectory_ready");
  EXPECT_GT(continued.target_speed_mps, 0.0);
}

TEST(PredictiveTrajectory, HalfMeterBoundariesDoNotCauseFixedStop)
{
  PredictivePlanner planner({});
  const auto result = planner.plan(boundaries(0.0, true, true, 0.5), 0.0);
  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.emergency_stop);
  EXPECT_GT(result.target_speed_mps, 0.0);
}

TEST(FeedbackStability, AlternatingLidarJitterDoesNotFlipSteeringEveryFrame)
{
  PredictivePlanner planner({});
  int sign_flips = 0;
  int previous_sign = 0;
  for (int frame = 0; frame < 12; ++frame) {
    const double jitter = frame % 2 == 0 ? 0.08 : -0.08;
    const auto result = planner.plan(boundaries(0.0, true, true, 0.75, jitter), frame * 0.05);
    ASSERT_TRUE(result.valid);
    const int sign = result.official_steering_rad > 1e-5 ? 1 :
      (result.official_steering_rad < -1e-5 ? -1 : 0);
    if (previous_sign != 0 && sign != 0 && sign != previous_sign) ++sign_flips;
    if (sign != 0) previous_sign = sign;
  }
  EXPECT_LE(sign_flips, 1);
}

TEST(FeedbackStability, PersistentCurveIsPreservedByTemporalFilters)
{
  PredictivePlanner planner({});
  ASSERT_TRUE(planner.plan(boundaries(), 0.0).valid);
  team01_race::Plan result;
  for (int frame = 1; frame <= 8; ++frame) {
    result = planner.plan(boundaries(-0.025), frame * 0.05);
  }
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.official_steering_rad, 0.0);
  EXPECT_LT(result.lookahead_m, 3.0);
  EXPECT_LT(result.predicted_curvature, 0.0);
}

TEST(FeedbackStability, FilterParametersDoNotChangeStraightCruiseSpeed)
{
  team01_race::PlannerConfig config;
  config.trajectory_temporal_alpha = 0.30;
  config.curvature_alpha = 0.35;
  config.lookahead_alpha = 0.25;
  PredictivePlanner planner(config);
  const auto result = planner.plan(boundaries(), 0.0);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.target_speed_mps, config.cruise_speed_mps);
}

TEST(FeedbackStability, FilteredTrajectoryMovesLessThanAlternatingRawTrajectory)
{
  PredictivePlanner planner({});
  auto previous = planner.plan(boundaries(0.0, true, true, 0.75, 0.08), 0.0);
  ASSERT_TRUE(previous.valid);
  for (int frame = 1; frame <= 8; ++frame) {
    const double jitter = frame % 2 == 0 ? 0.08 : -0.08;
    const auto current = planner.plan(
      boundaries(0.0, true, true, 0.75, jitter), frame * 0.05);
    ASSERT_TRUE(current.valid);
    const double raw_delta = std::abs(
      current.raw_centerline_lateral_m - previous.raw_centerline_lateral_m);
    const double filtered_delta = std::abs(
      current.centerline_lateral_m - previous.centerline_lateral_m);
    EXPECT_LT(filtered_delta, raw_delta);
    previous = current;
  }
}

TEST(StraightGuard, RejectsSingleFrameLateralTrajectoryJump)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  planner.plan(boundaries(), 0.05);
  const auto stable = planner.plan(boundaries(), 0.10);
  ASSERT_TRUE(stable.straight_guard_active);

  const auto outlier = planner.plan(
    boundaries(0.0, true, true, 0.75, 0.90), 0.15);
  ASSERT_TRUE(outlier.valid);
  EXPECT_TRUE(outlier.straight_guard_rejected);
  EXPECT_EQ(outlier.straight_guard_reason, "trajectory_jump");
  EXPECT_NEAR(outlier.centerline_lateral_m, stable.centerline_lateral_m, 1e-6);
  EXPECT_EQ(outlier.passing_side, PassingSide::NONE);
}

TEST(StraightGuard, AcceptsPersistentSameDirectionPathChange)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  planner.plan(boundaries(), 0.05);
  planner.plan(boundaries(), 0.10);
  planner.plan(boundaries(-0.08), 0.15);
  const auto tentative = planner.plan(boundaries(-0.08), 0.20);
  ASSERT_TRUE(tentative.straight_guard_rejected);
  const auto confirmed = planner.plan(boundaries(-0.08), 0.25);
  ASSERT_TRUE(confirmed.valid);
  EXPECT_FALSE(confirmed.straight_guard_rejected);
  EXPECT_LT(confirmed.centerline_lateral_m, -0.02);
}

TEST(StraightGuard, RejectsSingleFrameCurvatureSpike)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  planner.plan(boundaries(), 0.05);
  const auto stable = planner.plan(boundaries(), 0.10);
  auto spike = boundaries();
  for (auto & point : spike.left) {
    if (point.x > 2.3 && point.x < 2.7) point.y += 0.50;
  }
  for (auto & point : spike.right) {
    if (point.x > 2.3 && point.x < 2.7) point.y += 0.50;
  }
  const auto rejected = planner.plan(spike, 0.15);
  ASSERT_TRUE(rejected.valid);
  EXPECT_TRUE(rejected.straight_guard_rejected);
  EXPECT_EQ(rejected.straight_guard_reason, "curvature_spike");
  EXPECT_GT(rejected.raw_maximum_curvature, 0.75);
  EXPECT_NEAR(rejected.centerline_lateral_m, stable.centerline_lateral_m, 1e-6);
}

TEST(StraightGuard, RealCurveReleasesGuardWithoutSuppressingPredictiveSteering)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  planner.plan(boundaries(), 0.05);
  ASSERT_TRUE(planner.plan(boundaries(), 0.10).straight_guard_active);
  const auto curve_one = planner.plan(boundaries(-0.08), 0.15);
  const auto curve_two = planner.plan(boundaries(-0.08), 0.20);
  const auto curve_three = planner.plan(boundaries(-0.08), 0.25);
  ASSERT_TRUE(curve_one.valid);
  ASSERT_TRUE(curve_two.valid);
  EXPECT_FALSE(curve_one.straight_guard_rejected);
  ASSERT_TRUE(curve_three.valid);
  EXPECT_FALSE(curve_three.straight_guard_rejected);
  EXPECT_GT(curve_two.official_steering_rad, 0.0);
  EXPECT_FALSE(curve_three.straight_guard_active);
}

TEST(StraightGuard, ConfirmedObstacleStillSelectsPassTrajectory)
{
  PredictivePlanner planner({});
  planner.plan(boundaries(), 0.0);
  planner.plan(boundaries(), 0.05);
  planner.plan(boundaries(), 0.10);
  auto frame = boundaries();
  frame.obstacle_candidates.push_back(vehicle(2.5, -0.30));
  planner.plan(frame, 0.15);
  planner.plan(frame, 0.20);
  const auto passing = planner.plan(frame, 0.25);
  ASSERT_TRUE(passing.valid);
  EXPECT_TRUE(passing.obstacle_confirmed);
  EXPECT_EQ(passing.passing_side, PassingSide::LEFT);
  EXPECT_FALSE(passing.straight_guard_rejected);
}


TEST(ObstacleTracking, LaserScanExtractsCompactInteriorCluster)
{
  team01_race::PlannerConfig config;
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -team01_race::kPi / 2.0;
  scan.angle_max = team01_race::kPi / 2.0;
  scan.angle_increment = team01_race::kPi / 360.0;
  scan.range_min = 0.04;
  scan.range_max = 6.0;
  scan.ranges.assign(361, std::numeric_limits<float>::infinity());
  for (int degree_twice = -20; degree_twice <= 20; ++degree_twice) {
    const double fraction = std::abs(degree_twice) / 20.0;
    scan.ranges[static_cast<std::size_t>(degree_twice + 180)] =
      static_cast<float>(2.0 + 0.22 * fraction);
  }
  const auto frame = team01_race::extract_predictive_frame(scan, config);
  ASSERT_TRUE(frame.scan_valid);
  ASSERT_FALSE(frame.obstacle_candidates.empty());
  EXPECT_LT(frame.obstacle_candidates.front().centroid.x, 5.0);
}

TEST(ObstacleTracking, VehicleSizedClusterRequiresTemporalConfirmation)
{
  PredictivePlanner planner({});
  auto frame = boundaries();
  frame.obstacle_candidates.push_back(vehicle(2.5, 0.0));
  EXPECT_FALSE(planner.plan(frame, 0.0).obstacle_confirmed);
  EXPECT_FALSE(planner.plan(frame, 0.1).obstacle_confirmed);
  EXPECT_TRUE(planner.plan(frame, 0.2).obstacle_confirmed);
}

TEST(ObstacleTracking, CompactTrackEdgeFragmentNeverBecomesConfirmedObstacle)
{
  PredictivePlanner planner({});
  auto frame = boundaries();
  frame.obstacle_candidates.push_back(vehicle(2.5, 0.70, 0.20, 0.20));
  for (int cycle = 0; cycle < 8; ++cycle) {
    const auto result = planner.plan(frame, cycle * 0.05);
    EXPECT_FALSE(result.obstacle_confirmed);
    EXPECT_EQ(result.confirmed_obstacle_count, 0);
    EXPECT_EQ(result.passing_side, PassingSide::NONE);
  }
}

TEST(ObstacleTracking, BoundaryConnectedCompactClusterIsRejected)
{
  PredictivePlanner planner({});
  auto frame = boundaries();
  auto wall_fragment = vehicle(2.5, 0.30, 0.20, 0.20);
  wall_fragment.boundary_connected = true;
  frame.obstacle_candidates.push_back(wall_fragment);
  for (int cycle = 0; cycle < 8; ++cycle) {
    const auto result = planner.plan(frame, cycle * 0.05);
    EXPECT_EQ(result.confirmed_obstacle_count, 0);
    EXPECT_EQ(result.passing_side, PassingSide::NONE);
  }
}

TEST(ObstacleTracking, ClearLeftSpaceBuildsLeftPassingTrajectory)
{
  PredictivePlanner planner({});
  auto frame = boundaries();
  frame.obstacle_candidates.push_back(vehicle(2.5, -0.30));
  const auto result = confirm_obstacle(planner, frame);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.passing_side, PassingSide::LEFT);
  EXPECT_LT(result.official_steering_rad, 0.0);
}

TEST(ObstacleTracking, ClearRightSpaceBuildsRightPassingTrajectory)
{
  PredictivePlanner planner({});
  auto frame = boundaries();
  frame.obstacle_candidates.push_back(vehicle(2.5, 0.30));
  const auto result = confirm_obstacle(planner, frame);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.passing_side, PassingSide::RIGHT);
  EXPECT_GT(result.official_steering_rad, 0.0);
}

TEST(ObstacleTracking, NoPassingSpaceAndImminentCollisionEmergencyStops)
{
  PredictivePlanner planner({});
  auto frame = boundaries(0.0, true, true, 0.35);
  frame.obstacle_candidates.push_back(vehicle(0.45, 0.0, 0.30, 0.20));
  const auto result = confirm_obstacle(planner, frame);
  EXPECT_TRUE(result.emergency_stop);
  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.target_speed_mps, 0.0);
}

TEST(PredictiveTrajectory, HistoryExpiresIntoFaultInsteadOfBlindCruise)
{
  PredictivePlanner planner({});
  ASSERT_TRUE(planner.plan(boundaries(), 0.0).valid);
  BoundaryFrame invalid;
  const auto result = planner.plan(invalid, 0.6);
  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.target_speed_mps, 0.0);
}

TEST(ControlMath, AckermannUsesOfficialRightPositiveConvention)
{
  const auto right_turn = team01_race::ackermann_joint_angles(0.3, 0.34, 0.18, 0.523599);
  EXPECT_LT(right_turn.first, 0.0);
  EXPECT_LT(right_turn.second, 0.0);
  EXPECT_GT(std::abs(right_turn.second), std::abs(right_turn.first));
  EXPECT_LE(std::abs(right_turn.first), 0.523599);
  EXPECT_LE(std::abs(right_turn.second), 0.523599);
}
