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

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <lane_event_classifier/detail/geometry_utils.hpp>
#include <lane_event_classifier/lane_following/checker.hpp>
#include <magic_enum.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <algorithm>
#include <limits>
#include <optional>
#include <string_view>
#include <unordered_set>

namespace lane_event_classifier
{

namespace
{
namespace lanelet2_utils = autoware::experimental::lanelet2_utils;

bool is_point_inside_any(
  const lanelet::LaneletMapPtr & map, const std::unordered_set<lanelet::Id> & ids,
  const lanelet::BasicPoint2d & point)
{
  return std::any_of(ids.cbegin(), ids.cend(), [&](const auto id) {
    return map->laneletLayer.exists(id) &&
           lanelet::geometry::inside(map->laneletLayer.get(id), point);
  });
}

bool is_point_within_tolerance_of_any(
  const lanelet::LaneletMapPtr & map, const std::unordered_set<lanelet::Id> & ids,
  const lanelet::BasicPoint2d & point, double tolerance)
{
  return std::any_of(ids.cbegin(), ids.cend(), [&](const auto id) {
    return map->laneletLayer.exists(id) &&
           lanelet::geometry::distance2d(map->laneletLayer.get(id), point) <= tolerance;
  });
}

// road_shoulder_exempt: shoulders are excluded from the routing graph, so this is an explicit
// check.
bool is_point_in_road_shoulder(
  const lanelet::LaneletMapPtr & map, const lanelet::BasicPoint2d & point)
{
  return !lanelet2_utils::get_shoulder_lanelets_at(map, point.x(), point.y()).empty();
}

// turn_lane_exempt: ego is in a turn / intersection lane, either the reference lane or a sequence
// one.
bool is_in_turn_lane(
  const lanelet::LaneletMapPtr & map, const ReferenceLane & reference,
  const std::unordered_set<lanelet::Id> & sequence_ids, const lanelet::BasicPoint2d & point)
{
  if (reference.is_reference_lane_intersection) {
    return true;
  }
  for (const auto id : sequence_ids) {
    // The sequence contains the reference lane, whose verdict the flag above already settled.
    if (id == reference.reference_lane_id || !map->laneletLayer.exists(id)) {
      continue;
    }
    const auto lane = map->laneletLayer.get(id);
    if (lanelet2_utils::is_intersection_lanelet(lane) && lanelet::geometry::inside(lane, point)) {
      return true;
    }
  }
  return false;
}

// The sequence lane the ego is closest to, ignoring lanes it has driven past or not yet reached.
std::optional<lanelet::ConstLanelet> nearest_sequence_lane_beside(
  const lanelet::LaneletMapPtr & map, const std::unordered_set<lanelet::Id> & ids,
  const lanelet::BasicPoint2d & point)
{
  std::optional<lanelet::ConstLanelet> nearest;
  double nearest_distance = std::numeric_limits<double>::max();
  for (const auto id : ids) {
    if (!map->laneletLayer.exists(id)) {
      continue;
    }
    const auto lane = map->laneletLayer.get(id);
    const auto centerline = lane.centerline2d();
    const double arc_length = lanelet::geometry::toArcCoordinates(centerline, point).length;
    // Projecting off either end means the ego is not abreast of this lane, so its bounds say
    // nothing about the boundary the ego crossed.
    if (arc_length <= 0.0 || arc_length >= lanelet::geometry::length(centerline)) {
      continue;
    }
    const double distance = lanelet::geometry::distance2d(lane, point);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest = lane;
    }
  }
  return nearest;
}

// virtual_boundary_exempt: the boundary the ego crossed is virtual.
bool crossed_sequence_boundary_is_virtual(
  const lanelet::LaneletMapPtr & map, const std::unordered_set<lanelet::Id> & ids,
  const ReferenceLane & reference, const lanelet::BasicPoint2d & point)
{
  const auto nearest_lane = nearest_sequence_lane_beside(map, ids, point);
  if (!nearest_lane) {
    return false;
  }
  // Left of the centerline means the ego left over the left bound.
  const bool has_departed_to_left =
    lanelet::geometry::signedDistance(nearest_lane->centerline2d(), point) > 0.0;
  // The sequence contains the reference lane, whose bounds the tracker already resolved.
  if (nearest_lane->id() == reference.reference_lane_id) {
    return has_departed_to_left ? reference.is_reference_lane_left_bound_virtual
                                : reference.is_reference_lane_right_bound_virtual;
  }
  return is_virtual_linestring(
    has_departed_to_left ? nearest_lane->leftBound() : nearest_lane->rightBound());
}

}  // namespace

std::string_view to_debug_string(LaneFollowingReason reason)
{
  return magic_enum::enum_name(reason);
}

LaneFollowingChecker::LaneFollowingChecker(LaneFollowingConfig config) : config_{config}
{
}

LaneFollowingResult LaneFollowingChecker::evaluate(
  const LaneTracker & tracker, const lanelet::BasicPoint2d & ego_point) const
{
  const auto & lanelet_map_ptr = tracker.lanelet_map_ptr();
  // Attribute flags were resolved by the tracker when it anchored onto the reference lane.
  const auto & reference = tracker.reference_lane();

  // no_reference_lane (docs/lane_following.md, "No reference lane").
  const auto reference_lane_opt = tracker.get_lanelet(reference.reference_lane_id);
  if (!reference_lane_opt) {
    return {true, LaneFollowingReason::no_reference_lane};
  }
  const auto & reference_lane = *reference_lane_opt;
  const auto & sequence_ids = lane_sequence_cache_.get(
    reference_lane, tracker.routing_graph_ptr(), config_.connected_sequence_length_m);

  // inside_connected_sequence (docs/lane_following.md, "Inside the connected sequence").
  if (is_point_inside_any(lanelet_map_ptr, sequence_ids, ego_point)) {
    return {true, LaneFollowingReason::inside_connected_sequence};
  }

  // within_lateral_tolerance (docs/lane_following.md, "Within lateral tolerance").
  if (is_point_within_tolerance_of_any(
        lanelet_map_ptr, sequence_ids, ego_point, config_.lateral_tolerance_m)) {
    return {true, LaneFollowingReason::within_lateral_tolerance};
  }

  // road_shoulder_exempt (docs/lane_following.md, "Road-shoulder exemption").
  if (
    config_.enable_road_shoulder_exemption &&
    is_point_in_road_shoulder(lanelet_map_ptr, ego_point)) {
    return {true, LaneFollowingReason::road_shoulder_exempt};
  }

  // turn_lane_exempt (docs/lane_following.md, "Turn / intersection-lane exemption").
  if (
    config_.enable_turn_lane_exemption &&
    is_in_turn_lane(lanelet_map_ptr, reference, sequence_ids, ego_point)) {
    return {true, LaneFollowingReason::turn_lane_exempt};
  }

  // virtual_boundary_exempt (docs/lane_following.md, "Virtual-boundary exemption").
  if (
    config_.enable_virtual_boundary_exemption &&
    crossed_sequence_boundary_is_virtual(lanelet_map_ptr, sequence_ids, reference, ego_point)) {
    return {true, LaneFollowingReason::virtual_boundary_exempt};
  }

  // departed (docs/lane_following.md, "Otherwise — Departed").
  return {false, LaneFollowingReason::departed};
}

}  // namespace lane_event_classifier
