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

#include <autoware/lanelet2_utils/kind.hpp>
#include <lane_event_classifier/detail/geometry_utils.hpp>
#include <lane_event_classifier/lane_change/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <optional>
#include <unordered_set>
#include <vector>

namespace lane_event_classifier
{

namespace
{
namespace lanelet2_utils = autoware::experimental::lanelet2_utils;

// Outcome of scanning the trajectory for the reference lane's lateral boundary crossing.
struct TrajectoryCrossingScan
{
  std::optional<lanelet::Id> target_lane_id;  // first off-sequence route-preferred lane entered
  std::optional<lanelet::BasicPoint2d> crossing_point;  // first sample outside the lane sequence
};

// Scans forward for the first off-sequence sample; see docs/lane_change.md, "Finding a crossing".
TrajectoryCrossingScan scan_trajectory_for_crossing(
  const LaneTracker & tracker, const std::unordered_set<lanelet::Id> & lane_sequence,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points)
{
  TrajectoryCrossingScan scan;
  for (const auto & point : trajectory_points) {
    bool is_in_lane_sequence = false;
    bool has_off_sequence_lane = false;
    for (const auto id : tracker.lanelet_ids_at(point)) {
      if (lane_sequence.count(id) != 0) {
        is_in_lane_sequence = true;
        continue;
      }
      has_off_sequence_lane = true;
      // The target must be route-preferred, otherwise the settle check can never confirm it.
      if (!scan.target_lane_id && tracker.is_route_primitive(id)) {
        scan.target_lane_id = id;
      }
    }

    // First off-sequence sample marks the lateral boundary crossing.
    if (!scan.crossing_point && !is_in_lane_sequence && has_off_sequence_lane) {
      scan.crossing_point = point;
    }
    // Remaining samples cannot change the outcome.
    if (scan.target_lane_id && scan.crossing_point) {
      break;
    }
  }
  return scan;
}
}  // namespace

LaneChangeGeometry::LaneChangeGeometry(double crossing_look_ahead_m)
: crossing_look_ahead_m_{crossing_look_ahead_m}
{
}

LaneChangeObservation LaneChangeGeometry::observe(
  const LaneTracker & tracker, const LaneEventInput & input, const LaneEventContext & context) const
{
  // Compute the two per-cycle intermediates once and share them across the helpers below.
  const auto trajectory_points =
    forward_trajectory_points_from_input(input, crossing_look_ahead_m_);
  const auto & footprint_ids = context.footprint_lane_ids();

  LaneChangeObservation observation;
  observation.crossing = compute_crossing(tracker, input, context, trajectory_points);
  observation.trajectory_returns_to_reference = compute_trajectory_returns_to_reference(
    tracker, observation.crossing.has_value(), trajectory_points);
  observation.is_footprint_off_route_primitives =
    is_footprint_off_route_primitives(tracker, input, footprint_ids);
  observation.is_footprint_inside_reference_lane = tracker.is_footprint_fully_inside_lane(
    tracker.reference_lane().reference_lane_id, input.footprint);
  observation.settle_lane_id = compute_settle_lane_id(tracker, input, footprint_ids);
  return observation;
}

std::optional<LaneChangeCrossing> LaneChangeGeometry::compute_crossing(
  const LaneTracker & tracker, const LaneEventInput & input, const LaneEventContext & context,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points) const
{
  if (trajectory_points.empty() || !input.route_ptr) {
    return std::nullopt;
  }
  // Reference-lane attributes were resolved by the tracker when it anchored onto the lane.
  const auto & reference = tracker.reference_lane();
  const auto reference_lane_id = reference.reference_lane_id;

  // Straight-on-route skip; see docs/lane_change.md, "Finding a crossing".
  if (driving_straight_stays_on_route(tracker, reference_lane_id)) {
    return std::nullopt;
  }

  const auto reference_lane_opt = tracker.get_lanelet(reference_lane_id);
  if (!reference_lane_opt) {
    return std::nullopt;
  }

  const auto & reference_lane = *reference_lane_opt;

  if (reference.is_reference_lane_road_shoulder) {
    return std::nullopt;
  }

  const auto & lane_sequence = context.sequence_ids(crossing_look_ahead_m_);

  const auto [target_lane_id, crossing_point] =
    scan_trajectory_for_crossing(tracker, lane_sequence, trajectory_points);
  if (!target_lane_id || !crossing_point) {
    return std::nullopt;
  }

  // Onset exemptions (docs/lane_change.md, "Exemptions"); turn lanes carry turn_direction.
  if (reference.is_reference_lane_intersection) {
    return std::nullopt;
  }

  const auto target_lane_opt = tracker.get_lanelet(*target_lane_id);
  if (target_lane_opt && lanelet2_utils::is_shoulder_lane(*target_lane_opt)) {
    return std::nullopt;
  }

  // Signed offset from the centerline; positive means left.
  const double lateral_offset =
    lanelet::geometry::toArcCoordinates(reference_lane.centerline2d(), *crossing_point).distance;
  const bool is_to_left = lateral_offset > 0.0;

  // Onset exemption; see docs/lane_change.md, "Exemptions".
  const bool is_crossed_bound_virtual = is_to_left
                                          ? reference.is_reference_lane_left_bound_virtual
                                          : reference.is_reference_lane_right_bound_virtual;
  if (is_crossed_bound_virtual) {
    return std::nullopt;
  }

  LaneChangeCrossing crossing;
  crossing.target_lane_id = *target_lane_id;
  crossing.crossing_point = *crossing_point;
  crossing.is_to_left = is_to_left;
  return crossing;
}

bool LaneChangeGeometry::compute_trajectory_returns_to_reference(
  const LaneTracker & tracker, bool has_forward_crossing,
  const std::vector<lanelet::BasicPoint2d> & trajectory_points)
{
  // A forward crossing means the trajectory is still heading out, not back.
  if (has_forward_crossing || trajectory_points.empty()) {
    return false;
  }
  const auto reference_lane_opt = tracker.get_lanelet(tracker.reference_lane().reference_lane_id);
  if (!reference_lane_opt) {
    return false;
  }
  // The far look-ahead point is back inside the reference lane.
  return lanelet::geometry::inside(*reference_lane_opt, trajectory_points.back());
}

bool LaneChangeGeometry::is_footprint_off_route_primitives(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::vector<lanelet::Id> & footprint_ids)
{
  if (!input.route_ptr || footprint_ids.empty()) {
    return false;
  }

  // "Off the route primitives" == no footprint corner lies inside any route-preferred primitive.
  return std::none_of(
    footprint_ids.cbegin(), footprint_ids.cend(),
    [&tracker](const lanelet::Id id) { return tracker.is_route_primitive(id); });
}

std::optional<lanelet::Id> LaneChangeGeometry::compute_settle_lane_id(
  const LaneTracker & tracker, const LaneEventInput & input,
  const std::vector<lanelet::Id> & footprint_ids)
{
  if (!input.route_ptr) {
    return std::nullopt;
  }
  const auto reference_lane_id = tracker.reference_lane().reference_lane_id;

  // A settle candidate is a route-preferred primitive other than the reference lane
  const auto settled =
    std::find_if(footprint_ids.cbegin(), footprint_ids.cend(), [&](const lanelet::Id candidate_id) {
      return candidate_id != reference_lane_id && tracker.is_route_primitive(candidate_id) &&
             tracker.is_footprint_fully_inside_lane(candidate_id, input.footprint);
    });
  if (settled == footprint_ids.cend()) {
    return std::nullopt;
  }
  return *settled;
}

}  // namespace lane_event_classifier
