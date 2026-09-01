// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Trajectory-driven lane-crossing tests on the real test map (see docs/lane_crossing.md).

#include "synthetic_lanelet_maps.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <lane_event_classifier/detail/lane_tracker.hpp>
#include <lane_event_classifier/lane_crossing/classifier.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <lane_event_classifier_msgs/msg/driving_state.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace lane_event_classifier
{

namespace
{
using DS = lane_event_classifier_msgs::msg::DrivingState;
using TurnIndicatorsReport = autoware_vehicle_msgs::msg::TurnIndicatorsReport;

constexpr std::array<lanelet::Id, 4> route_ids()
{
  return {47, 1167, 51, 55};  // preferred-primitive sequence
}

lanelet::LaneletMapPtr load_test_map()
{
  const auto map_const =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(TEST_MAP_PATH);
  return map_const ? autoware::experimental::lanelet2_utils::remove_const(map_const) : nullptr;
}

std::vector<lanelet::BasicPoint2d> centerline_points(
  const lanelet::LaneletMapPtr & map, lanelet::Id id)
{
  const auto centerline = map->laneletLayer.get(id).centerline2d();
  std::vector<lanelet::BasicPoint2d> points;
  points.reserve(centerline.size());
  for (const auto & point : centerline) {
    points.emplace_back(point.x(), point.y());
  }
  return points;
}

lanelet::BasicPoint2d point_at_fraction(
  const std::vector<lanelet::BasicPoint2d> & points, double fraction)
{
  const auto index = static_cast<std::size_t>(
    std::clamp(fraction, 0.0, 1.0) * static_cast<double>(points.size() - 1));
  return points[index];
}

lanelet::BasicPoint2d nearest_point_on(
  const std::vector<lanelet::BasicPoint2d> & points, const lanelet::BasicPoint2d & query)
{
  lanelet::BasicPoint2d nearest = points.front();
  double nearest_distance_sq = std::numeric_limits<double>::max();
  for (const auto & point : points) {
    const double distance_sq = (point - query).squaredNorm();
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest = point;
    }
  }
  return nearest;
}

// A point inset_m inside lane id's right boundary, so the lateral-proximity gate is satisfied.
lanelet::BasicPoint2d point_near_right_boundary(
  const lanelet::LaneletMapPtr & map, lanelet::Id id, double frac, double inset_m)
{
  const auto right_bound = map->laneletLayer.get(id).rightBound2d();
  std::vector<lanelet::BasicPoint2d> bound_points;
  bound_points.reserve(right_bound.size());
  for (const auto & point : right_bound) {
    bound_points.emplace_back(point.x(), point.y());
  }
  const lanelet::BasicPoint2d boundary_point = point_at_fraction(bound_points, frac);
  const lanelet::BasicPoint2d center_point =
    nearest_point_on(centerline_points(map, id), boundary_point);
  const lanelet::BasicPoint2d inward = (center_point - boundary_point).normalized();
  return boundary_point + inward * inset_m;
}

// Same as point_near_right_boundary but for the left boundary.
lanelet::BasicPoint2d point_near_left_boundary(
  const lanelet::LaneletMapPtr & map, lanelet::Id id, double frac, double inset_m)
{
  const auto left_bound = map->laneletLayer.get(id).leftBound2d();
  std::vector<lanelet::BasicPoint2d> bound_points;
  bound_points.reserve(left_bound.size());
  for (const auto & point : left_bound) {
    bound_points.emplace_back(point.x(), point.y());
  }
  const lanelet::BasicPoint2d boundary_point = point_at_fraction(bound_points, frac);
  const lanelet::BasicPoint2d center_point =
    nearest_point_on(centerline_points(map, id), boundary_point);
  const lanelet::BasicPoint2d inward = (center_point - boundary_point).normalized();
  return boundary_point + inward * inset_m;
}

// Builds a trajectory that sweeps laterally through the listed lanes while advancing forward.
std::vector<lanelet::BasicPoint2d> build_lane_trajectory(
  const lanelet::LaneletMapPtr & map, const std::vector<lanelet::Id> & lane_ids, double start_frac,
  double end_frac, std::size_t point_count)
{
  std::vector<std::vector<lanelet::BasicPoint2d>> centerlines;
  centerlines.reserve(lane_ids.size());
  for (const auto id : lane_ids) {
    centerlines.push_back(centerline_points(map, id));
  }
  const auto & base = centerlines.front();
  const double lane_span = static_cast<double>(lane_ids.size() - 1);

  std::vector<lanelet::BasicPoint2d> trajectory;
  trajectory.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    const double progress = static_cast<double>(index) / static_cast<double>(point_count - 1);
    const double longitudinal_fraction = start_frac + (end_frac - start_frac) * progress;
    const lanelet::BasicPoint2d anchor = point_at_fraction(base, longitudinal_fraction);

    const double lane_position = progress * lane_span;
    const auto lane_index = static_cast<std::size_t>(lane_position);
    const auto next_lane_index = std::min(lane_index + 1, lane_ids.size() - 1);
    const double lateral_fraction = lane_position - static_cast<double>(lane_index);

    const lanelet::BasicPoint2d point_a = nearest_point_on(centerlines[lane_index], anchor);
    const lanelet::BasicPoint2d point_b = nearest_point_on(centerlines[next_lane_index], anchor);
    trajectory.push_back(point_a + (point_b - point_a) * lateral_fraction);
  }
  return trajectory;
}

// Builds an out-and-back poke toward poke_id: the closed departure the bracket onset detects.
std::vector<lanelet::BasicPoint2d> build_out_and_back_trajectory(
  const lanelet::LaneletMapPtr & map, lanelet::Id base_id, lanelet::Id poke_id, double start_frac,
  double end_frac, std::size_t point_count)
{
  const auto base = centerline_points(map, base_id);
  const auto poke = centerline_points(map, poke_id);
  std::vector<lanelet::BasicPoint2d> trajectory;
  trajectory.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    const double progress = static_cast<double>(index) / static_cast<double>(point_count - 1);
    const double longitudinal_fraction = start_frac + (end_frac - start_frac) * progress;
    const lanelet::BasicPoint2d anchor = point_at_fraction(base, longitudinal_fraction);
    const double lateral_fraction = 1.0 - std::abs(2.0 * progress - 1.0);  // triangle 0 -> 1 -> 0
    const lanelet::BasicPoint2d base_point = nearest_point_on(base, anchor);
    const lanelet::BasicPoint2d poke_point = nearest_point_on(poke, anchor);
    trajectory.push_back(base_point + (poke_point - base_point) * lateral_fraction);
  }
  return trajectory;
}

// Builds the tail of a poke: the path starts on poke_id and crosses back onto base_id once.
std::vector<lanelet::BasicPoint2d> build_back_only_trajectory(
  const lanelet::LaneletMapPtr & map, lanelet::Id base_id, lanelet::Id poke_id, double start_frac,
  double end_frac, std::size_t point_count)
{
  const auto base = centerline_points(map, base_id);
  const auto poke = centerline_points(map, poke_id);
  std::vector<lanelet::BasicPoint2d> trajectory;
  trajectory.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    const double progress = static_cast<double>(index) / static_cast<double>(point_count - 1);
    const double longitudinal_fraction = start_frac + (end_frac - start_frac) * progress;
    const lanelet::BasicPoint2d anchor = point_at_fraction(base, longitudinal_fraction);
    const double lateral_fraction = 1.0 - progress;  // starts on poke_id, ends on base_id
    const lanelet::BasicPoint2d base_point = nearest_point_on(base, anchor);
    const lanelet::BasicPoint2d poke_point = nearest_point_on(poke, anchor);
    trajectory.push_back(base_point + (poke_point - base_point) * lateral_fraction);
  }
  return trajectory;
}

// A small footprint square around a point (well within a road lane's width).
std::vector<lanelet::BasicPoint2d> footprint_box(const lanelet::BasicPoint2d & center)
{
  constexpr double half = 0.25;
  return {
    {center.x() - half, center.y() - half},
    {center.x() + half, center.y() - half},
    {center.x() + half, center.y() + half},
    {center.x() - half, center.y() + half}};
}

// Builds a footprint quad straddling lane 47's right boundary, far_m into 48 and near_m into 47.
std::vector<lanelet::BasicPoint2d> build_straddle_footprint(
  const lanelet::LaneletMapPtr & map, double near_m, double far_m)
{
  const auto right_bound = map->laneletLayer.get(47).rightBound2d();
  const auto centerline_47 = centerline_points(map, 47);
  const std::size_t index = right_bound.size() / 2;
  const lanelet::BasicPoint2d b1{right_bound[index - 1].x(), right_bound[index - 1].y()};
  const lanelet::BasicPoint2d b2{right_bound[index].x(), right_bound[index].y()};
  const lanelet::BasicPoint2d tangent = (b2 - b1).normalized();
  lanelet::BasicPoint2d normal{-tangent.y(), tangent.x()};
  // Point the normal away from lane 47's centerline (toward the neighbour lane 48).
  const lanelet::BasicPoint2d near_center = nearest_point_on(centerline_47, b1);
  if (normal.dot(lanelet::BasicPoint2d{b1 - near_center}) < 0.0) {
    normal = -normal;
  }
  return {b1 - normal * near_m, b1 + normal * far_m, b2 + normal * far_m, b2 - normal * near_m};
}

// Drives one cycle exactly as the node does: update, classify, then hold or release.
class Simulator
{
public:
  Simulator(lanelet::LaneletMapPtr map, LaneCrossingConfig config)
  : classifier_{
      true, config, tracker_,
      LaneCrossingGeometry{
        config.crossing_look_ahead_m, config.footprint_boundary_overshoot_m,
        config.predictive_lateral_trigger_distance_m, config.footprint_crossing_object_proximity_m},
      LaneCrossingObjects{config.object_longitudinal_window_m}}
  {
    const auto result = tracker_.set_lanelet_map(map);
    EXPECT_TRUE(result.has_value());
  }

  std::optional<uint8_t> step(const LaneEventInput & input)
  {
    [[maybe_unused]] const auto update_result = tracker_.update(input);
    classifier_.update(input);
    const auto state = classifier_.get_state();
    const bool is_active = state == DS::INTENTIONAL_LANE_CROSSING;
    if (is_active && !tracker_.is_reference_lane_held()) {
      tracker_.hold_reference_lane();
    } else if (!is_active && tracker_.is_reference_lane_held()) {
      tracker_.release_reference_lane();
    }
    return state;
  }

  [[nodiscard]] std::string debug_reason() const { return classifier_.debug_reason(); }

  [[nodiscard]] lanelet::Id reference_lane_id() const
  {
    return tracker_.reference_lane().reference_lane_id;
  }

private:
  LaneTracker tracker_;
  IntentionalCrossingClassifier classifier_;
};

LaneCrossingConfig make_config()
{
  LaneCrossingConfig config;
  config.enable_classifier = true;
  config.crossing_look_ahead_m = 50.0;
  config.predictive_lateral_trigger_distance_m = 0.75;
  config.crossing_persist_duration_s = 0.3;
  config.crossing_position_tolerance_m = 2.0;
  config.footprint_boundary_overshoot_m = 0.5;
  config.footprint_crossing_object_proximity_m = 10.0;
  config.settle_confirm_duration_s = 0.5;
  config.confidence_factor = 0.5;
  config.object_longitudinal_window_m = 50.0;
  config.object_qualifying_memory_s = 3.0;
  return config;
}

// Splits a running millisecond count into a (sec, nsec) stamp.
std::pair<int32_t, uint32_t> stamp_from_ms(int64_t total_ms)
{
  return {
    static_cast<int32_t>(total_ms / 1000), static_cast<uint32_t>((total_ms % 1000) * 1'000'000)};
}

// Drives an out-and-back poke with an object ahead, returning true once a crossing onsets.
bool run_until_crossing(
  Simulator & sim, const std::vector<lanelet::Id> & route,
  const std::vector<lanelet::BasicPoint2d> & trajectory, const lanelet::BasicPoint2d & ego,
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects, int max_cycles,
  int64_t & time_ms)
{
  for (int cycle = 0; cycle < max_cycles; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const auto state = sim.step(
      test_maps::make_trajectory_input(
        route, ego, sec, nsec, trajectory, {ego}, TurnIndicatorsReport::DISABLE, objects));
    if (state == DS::INTENTIONAL_LANE_CROSSING) {
      return true;
    }
    time_ms += 100;
  }
  return false;
}
}  // namespace

// Onset then completion: a poke around an object onsets, and the return inside 47 completes it.
TEST(LaneCrossingTest, onset_then_return_completes)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map)) << "failed to load " << TEST_MAP_PATH;

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = point_near_right_boundary(map, 47, 0.3, 0.4);  // near the crossed boundary
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  ASSERT_TRUE(run_until_crossing(sim, route, crossing_trajectory, ego_in_47, objects, 10, time_ms));
  EXPECT_EQ(sim.reference_lane_id(), 47);  // reference held to the origin lane at onset

  // Return: the footprint sits fully back inside the reference lane 47.
  const auto point_in_47 = point_at_fraction(centerline_points(map, 47), 0.6);
  const auto footprint_47 = footprint_box(point_in_47);
  const auto return_trajectory = build_lane_trajectory(map, {47}, 0.5, 0.95, 20);

  bool completed = false;
  std::optional<uint8_t> state = DS::INTENTIONAL_LANE_CROSSING;
  for (int cycle = 0; cycle < 12; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, point_in_47, sec, nsec, return_trajectory, footprint_47,
        TurnIndicatorsReport::DISABLE, objects));
    if (!state) {
      completed = true;
      break;
    }
  }
  EXPECT_TRUE(completed) << "footprint back inside 47 for the settle window should complete";
}

// Onset exemption: a dodge toward road shoulder 176949 is lane following, not a crossing.
TEST(LaneCrossingTest, dodge_toward_road_shoulder_is_not_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map)) << "failed to load " << TEST_MAP_PATH;

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 176949, 0.3, 0.9, 25);
  const auto ego_in_47 = point_near_left_boundary(map, 47, 0.3, 0.4);
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.6);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const auto state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, crossing_trajectory, {ego_in_47},
        TurnIndicatorsReport::DISABLE, objects));
    EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
      << "a dodge into a road shoulder is a shoulder use, not an intentional crossing: "
      << sim.debug_reason();
    time_ms += 100;
  }
}

// A trajectory already outside the sequence still onsets: the ego stands in for the exit.
TEST(LaneCrossingTest, trajectory_starting_outside_the_sequence_still_onsets)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map)) << "failed to load " << TEST_MAP_PATH;

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto return_only_trajectory = build_back_only_trajectory(map, 47, 48, 0.4, 0.9, 25);
  const auto ego_in_47 = point_near_right_boundary(map, 47, 0.4, 0.4);
  const auto object_point = point_near_right_boundary(map, 47, 0.5, 0.5);  // the object dodged
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  EXPECT_TRUE(
    run_until_crossing(sim, route, return_only_trajectory, ego_in_47, objects, 10, time_ms))
    << sim.debug_reason();
}

// No object: the identical out-and-back poke with nothing to go around is never a crossing.
TEST(LaneCrossingTest, no_object_is_not_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = point_near_right_boundary(map, 47, 0.3, 0.4);  // near the crossed boundary

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 15; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const auto state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, crossing_trajectory, {ego_in_47}));
    EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING) << "no obstacle, so no intentional crossing";
    time_ms += 100;
  }
}

// Lateral-proximity gate: a poke planned while the ego stays centred must not onset.
TEST(LaneCrossingTest, centered_ego_far_from_boundary_does_not_predictively_onset)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_centered = point_at_fraction(centerline_points(map, 47), 0.3);  // lane centre
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 15; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const auto state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_centered, sec, nsec, crossing_trajectory, {ego_centered},
        TurnIndicatorsReport::DISABLE, objects));
    EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
      << "a dodge only planned ahead, ego still centred, must not predictively onset";
    time_ms += 100;
  }
}

// A monotonic sweep into the neighbour forms no closed departure, so onset never fires.
TEST(LaneCrossingTest, monotonic_sweep_into_neighbour_is_not_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto sweep_trajectory = build_lane_trajectory(map, {47, 48}, 0.3, 0.9, 20);
  const auto ego_in_47 = sweep_trajectory.front();
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 15; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const auto state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, sweep_trajectory, {ego_in_47}, TurnIndicatorsReport::DISABLE,
        objects));
    EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
      << "a path that commits to the neighbour is a lane change, not a crossing";
    time_ms += 100;
  }
}

// A moving object qualifies: the closed departure around a slow-moving obstacle still onsets.
TEST(LaneCrossingTest, moving_object_can_qualify)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = point_near_right_boundary(map, 47, 0.3, 0.4);  // near the crossed boundary
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects = test_maps::make_objects(
    {test_maps::make_object(object_point.x(), object_point.y(), 2.0, 2.0, 5.0)});  // 5 m/s

  int64_t time_ms = 0;
  EXPECT_TRUE(run_until_crossing(sim, route, crossing_trajectory, ego_in_47, objects, 10, time_ms))
    << "a moving object the ego dodges around still qualifies";
}

// Two staggered objects: the closed departure need only bracket one candidate to onset.
TEST(LaneCrossingTest, staggered_objects_one_bracketed_is_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = point_near_right_boundary(map, 47, 0.3, 0.4);  // near the crossed boundary
  const auto centerline_47 = centerline_points(map, 47);
  const auto near_object = point_at_fraction(centerline_47, 0.5);  // at the poke
  const auto far_object = point_at_fraction(centerline_47, 0.95);  // beyond the re-enter
  const auto objects = test_maps::make_objects(
    {test_maps::make_object(near_object.x(), near_object.y()),
     test_maps::make_object(far_object.x(), far_object.y())});

  int64_t time_ms = 0;
  EXPECT_TRUE(run_until_crossing(sim, route, crossing_trajectory, ego_in_47, objects, 10, time_ms))
    << "bracketing one of several staggered objects is enough";
}

// Full-entry escape: a footprint fully inside off-route lane 48 drops the crossing classifier.
TEST(LaneCrossingTest, full_entry_into_adjacent_lane_escapes)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = point_near_right_boundary(map, 47, 0.3, 0.4);  // near the crossed boundary
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  ASSERT_TRUE(run_until_crossing(sim, route, crossing_trajectory, ego_in_47, objects, 10, time_ms));

  // The footprint is fully inside 48 with the object behind, so it is a lane change.
  const auto point_in_48 = point_at_fraction(centerline_points(map, 48), 0.8);
  const auto footprint_48 = footprint_box(point_in_48);
  const auto through_48_trajectory = build_lane_trajectory(map, {48}, 0.7, 0.95, 20);
  time_ms += 100;
  const auto [sec, nsec] = stamp_from_ms(time_ms);
  const auto state = sim.step(
    test_maps::make_trajectory_input(
      route, point_in_48, sec, nsec, through_48_trajectory, footprint_48,
      TurnIndicatorsReport::DISABLE, objects));
  EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
    << "full entry into 48 past the object is a lane change, not a crossing";
}

// Candidate-object memory: a perception dropout at the crossing moment must still let onset fire.
TEST(LaneCrossingTest, candidate_memory_bridges_perception_dropout)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  const auto straight_trajectory = build_lane_trajectory(map, {47}, 0.3, 0.6, 20);
  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = point_near_right_boundary(map, 47, 0.3, 0.4);  // near the crossed boundary

  int64_t time_ms = 0;
  // Phase 1: going straight, the object is perceived and latches the candidate memory. No crossing.
  for (int cycle = 0; cycle < 4; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, {ego_in_47},
        TurnIndicatorsReport::DISABLE, objects));
    time_ms += 100;
  }
  // Phase 2: the ego dodges while perception drops the object, so the memory must carry onset.
  bool became_crossing = false;
  for (int cycle = 0; cycle < 6; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const auto state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, crossing_trajectory, {ego_in_47},
        TurnIndicatorsReport::DISABLE, nullptr));
    if (state == DS::INTENTIONAL_LANE_CROSSING) {
      became_crossing = true;
      break;
    }
    time_ms += 100;
  }
  EXPECT_TRUE(became_crossing)
    << "a candidate seen a moment ago should bridge a transient perception dropout";
}

// Physical source: a shallow dodge onsets on the body alone, with no centerline departure.
TEST(LaneCrossingTest, footprint_over_boundary_is_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto straight_trajectory = build_lane_trajectory(map, {47}, 0.3, 0.9, 20);
  const auto ego_in_47 = straight_trajectory.front();
  const auto footprint = build_straddle_footprint(map, 0.6, 1.0);  // 1.0 m into 48, > 0.5 threshold
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  std::optional<uint8_t> state;
  int64_t time_ms = 0;
  bool became_crossing = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, footprint, TurnIndicatorsReport::DISABLE,
        objects));
    if (state == DS::INTENTIONAL_LANE_CROSSING) {
      became_crossing = true;
      break;
    }
    time_ms += 100;
  }
  EXPECT_TRUE(became_crossing)
    << "the body poking past the boundary is a crossing even on an in-lane path";
}

// Proximity guard: a candidate object far from the poking corner must not onset a crossing.
TEST(LaneCrossingTest, footprint_over_boundary_with_distant_object_is_not_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto straight_trajectory = build_lane_trajectory(map, {47}, 0.3, 0.9, 20);
  const auto ego_in_47 = straight_trajectory.front();
  const auto footprint = build_straddle_footprint(map, 0.6, 1.0);  // 1.0 m into 48, > 0.5 threshold
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.95);  // far ahead
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const auto state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, footprint, TurnIndicatorsReport::DISABLE,
        objects));
    EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
      << "a body overhang far from any candidate object is not a crossing";
    time_ms += 100;
  }
}

// Cornering guard: an overhang below footprint_boundary_overshoot_m is a graze, not a crossing.
TEST(LaneCrossingTest, slight_cornering_overhang_is_not_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto straight_trajectory = build_lane_trajectory(map, {47}, 0.3, 0.9, 20);
  const auto ego_in_47 = straight_trajectory.front();
  const auto footprint = build_straddle_footprint(map, 0.6, 0.1);  // 0.1 m into 48, < 0.5 threshold
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 15; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const auto state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, footprint, TurnIndicatorsReport::DISABLE,
        objects));
    EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
      << "a slight cornering overhang is not a crossing";
    time_ms += 100;
  }
}

// No time cap: a slow, long dodge holds the crossing until the ego actually returns.
TEST(LaneCrossingTest, long_dodge_does_not_time_out)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto straight_trajectory = build_lane_trajectory(map, {47}, 0.3, 0.9, 20);
  const auto ego_in_47 = straight_trajectory.front();
  const auto straddle_footprint =
    build_straddle_footprint(map, 0.6, 1.0);  // body over the boundary
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  std::optional<uint8_t> state;
  int64_t time_ms = 0;
  bool became_crossing = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, straddle_footprint,
        TurnIndicatorsReport::DISABLE, objects));
    if (state == DS::INTENTIONAL_LANE_CROSSING) {
      became_crossing = true;
      break;
    }
    time_ms += 100;
  }
  ASSERT_TRUE(became_crossing);

  // Hold the body over the boundary for 20 s: it must stay a crossing the whole time.
  for (int cycle = 0; cycle < 200; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, straddle_footprint,
        TurnIndicatorsReport::DISABLE, objects));
    ASSERT_EQ(state, DS::INTENTIONAL_LANE_CROSSING)
      << "a long dodge must not time out at cycle " << cycle;
  }

  // The ego finally returns fully inside 47: now it completes.
  const auto point_in_47 = point_at_fraction(centerline_points(map, 47), 0.6);
  const auto footprint_47 = footprint_box(point_in_47);
  const auto return_trajectory = build_lane_trajectory(map, {47}, 0.5, 0.95, 20);
  bool completed = false;
  for (int cycle = 0; cycle < 12; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, point_in_47, sec, nsec, return_trajectory, footprint_47,
        TurnIndicatorsReport::DISABLE, objects));
    if (!state) {
      completed = true;
      break;
    }
  }
  EXPECT_TRUE(completed) << "returning fully inside 47 completes the crossing";
}

// Ending requires the ego to be fully inside one lane, so a straddling ego holds the crossing.
TEST(LaneCrossingTest, object_gone_while_straddling_stays_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto straight_trajectory = build_lane_trajectory(map, {47}, 0.3, 0.9, 20);
  const auto ego_in_47 = straight_trajectory.front();
  const auto straddle_footprint = build_straddle_footprint(map, 0.6, 1.0);
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  std::optional<uint8_t> state;
  int64_t time_ms = 0;
  bool became_crossing = false;
  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, straddle_footprint,
        TurnIndicatorsReport::DISABLE, objects));
    if (state == DS::INTENTIONAL_LANE_CROSSING) {
      became_crossing = true;
      break;
    }
    time_ms += 100;
  }
  ASSERT_TRUE(became_crossing);

  // The object is gone but the ego keeps straddling, so there is nothing to end into.
  for (int cycle = 0; cycle < 60; ++cycle) {
    time_ms += 100;
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_47, sec, nsec, straight_trajectory, straddle_footprint,
        TurnIndicatorsReport::DISABLE, nullptr));
    ASSERT_EQ(state, DS::INTENTIONAL_LANE_CROSSING)
      << "a straddling ego does not end the crossing at cycle " << cycle;
  }
}

// Full entry ends the crossing even with an object still ahead: the ego has committed.
TEST(LaneCrossingTest, full_crossover_ends_even_with_object_ahead)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 47, 48, 0.3, 0.9, 25);
  const auto ego_in_47 = point_near_right_boundary(map, 47, 0.3, 0.4);  // near the crossed boundary
  const auto object_point = point_at_fraction(centerline_points(map, 47), 0.6);  // stays ahead
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  ASSERT_TRUE(run_until_crossing(sim, route, crossing_trajectory, ego_in_47, objects, 10, time_ms));

  // Whole footprint fully inside 48 at 0.35, with the object still ahead at 0.6 in the route lane.
  const auto point_in_48 = point_at_fraction(centerline_points(map, 48), 0.35);
  const auto footprint_48 = footprint_box(point_in_48);
  const auto through_48_trajectory = build_lane_trajectory(map, {48}, 0.3, 0.8, 20);
  time_ms += 100;
  const auto [sec, nsec] = stamp_from_ms(time_ms);
  const auto state = sim.step(
    test_maps::make_trajectory_input(
      route, point_in_48, sec, nsec, through_48_trajectory, footprint_48,
      TurnIndicatorsReport::DISABLE, objects));
  EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
    << "a full crossover into the neighbour ends the crossing (now a lane change)";
}

// Mutual exclusion: from off-route 48 the scope gate fails, so no crossing fires.
TEST(LaneCrossingTest, lane_change_regime_is_not_a_crossing)
{
  auto map = load_test_map();
  ASSERT_TRUE(static_cast<bool>(map));

  Simulator sim{map, make_config()};
  const std::vector<lanelet::Id> route{route_ids().begin(), route_ids().end()};

  const auto crossing_trajectory = build_out_and_back_trajectory(map, 48, 47, 0.3, 0.9, 25);
  const auto ego_in_48 = crossing_trajectory.front();
  const auto object_point = point_at_fraction(centerline_points(map, 48), 0.5);
  const auto objects =
    test_maps::make_objects({test_maps::make_object(object_point.x(), object_point.y())});

  int64_t time_ms = 0;
  for (int cycle = 0; cycle < 15; ++cycle) {
    const auto [sec, nsec] = stamp_from_ms(time_ms);
    const auto state = sim.step(
      test_maps::make_trajectory_input(
        route, ego_in_48, sec, nsec, crossing_trajectory, {ego_in_48},
        TurnIndicatorsReport::DISABLE, objects));
    EXPECT_NE(state, DS::INTENTIONAL_LANE_CROSSING)
      << "crossing from an off-route lane is a lane change";
    time_ms += 100;
  }
}

}  // namespace lane_event_classifier
