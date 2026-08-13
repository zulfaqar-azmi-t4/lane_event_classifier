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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware/lanelet2_utils/kind.hpp>
#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/lanelet2_utils/topology.hpp>
#include <lane_event_classifier/detail/geometry_utils.hpp>
#include <lane_event_classifier/detail/lane_sequence.hpp>
#include <lane_event_classifier/detail/lane_tracker.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BoundingBox.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

namespace lane_event_classifier
{
namespace lanelet2_utils = autoware::experimental::lanelet2_utils;

tl::expected<void, std::string> LaneTracker::set_lanelet_map(
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  if (!lanelet_map_ptr) {
    return tl::make_unexpected("LaneTracker: lanelet map is null");
  }
  if (lanelet_map_ptr->laneletLayer.empty()) {
    return tl::make_unexpected("LaneTracker: lanelet map has no lanelets");
  }
  lanelet_map_ptr_ = lanelet_map_ptr;
  std::tie(routing_graph_ptr_, traffic_rules_ptr_) =
    autoware::experimental::lanelet2_utils::instantiate_routing_graph_and_traffic_rules(
      lanelet_map_ptr_);
  return {};
}

void LaneTracker::release_reference_lane()
{
  is_reference_lane_held_ = false;
  reference_lane_ = {};
}

bool LaneTracker::lanelet_exists(lanelet::Id lane_id) const
{
  return lanelet_map_ptr_ && lane_id != lanelet::InvalId &&
         lanelet_map_ptr_->laneletLayer.exists(lane_id);
}

std::optional<lanelet::ConstLanelet> LaneTracker::get_lanelet(lanelet::Id lane_id) const
{
  if (!lanelet_exists(lane_id)) {
    return std::nullopt;
  }
  return lanelet_map_ptr_->laneletLayer.get(lane_id);
}

std::optional<double> LaneTracker::distance_to_lane(
  lanelet::Id lane_id, const lanelet::BasicPoint2d & point) const
{
  if (!lanelet_exists(lane_id)) {
    return std::nullopt;
  }
  const auto lane = lanelet_map_ptr_->laneletLayer.get(lane_id);
  return lanelet::geometry::distance2d(lane, point);
}

std::vector<lanelet::Id> LaneTracker::lanelet_ids_at(const lanelet::BasicPoint2d & point) const
{
  std::vector<lanelet::Id> ids;
  if (!lanelet_map_ptr_) {
    return ids;
  }
  // R-tree query then precise inside test — avoids scanning the whole map.
  const lanelet::BoundingBox2d query_box(point, point);
  for (const auto & lane : lanelet_map_ptr_->laneletLayer.search(query_box)) {
    if (lanelet::geometry::inside(lane, point)) {
      ids.push_back(lane.id());
    }
  }

  return ids;
}

std::vector<lanelet::Id> LaneTracker::footprint_lane_ids(
  const std::vector<lanelet::BasicPoint2d> & footprint) const
{
  std::vector<lanelet::Id> ids;
  if (!lanelet_map_ptr_ || footprint.empty()) {
    return ids;
  }
  // R-tree query on the footprint bbox then per-corner inside test — avoids a whole-map scan.
  lanelet::BasicPoint2d min_corner = footprint.front();
  lanelet::BasicPoint2d max_corner = footprint.front();
  for (const auto & corner : footprint) {
    min_corner.x() = std::min(min_corner.x(), corner.x());
    min_corner.y() = std::min(min_corner.y(), corner.y());
    max_corner.x() = std::max(max_corner.x(), corner.x());
    max_corner.y() = std::max(max_corner.y(), corner.y());
  }
  const lanelet::BoundingBox2d query_box(min_corner, max_corner);
  for (const auto & lane : lanelet_map_ptr_->laneletLayer.search(query_box)) {
    const bool any_corner_inside = std::any_of(
      footprint.cbegin(), footprint.cend(), [&lane](const lanelet::BasicPoint2d & corner) {
        return lanelet::geometry::inside(lane, corner);
      });
    if (any_corner_inside) {
      ids.push_back(lane.id());
    }
  }
  return ids;
}

std::vector<lanelet::Id> LaneTracker::next_lane_ids(lanelet::Id lane_id) const
{
  std::vector<lanelet::Id> ids;
  const auto lane = get_lanelet(lane_id);
  if (!routing_graph_ptr_ || !lane) {
    return ids;
  }
  for (const auto & next_lane : lanelet2_utils::following_lanelets(*lane, routing_graph_ptr_)) {
    ids.push_back(next_lane.id());
  }
  return ids;
}

lanelet::ConstLanelets LaneTracker::get_forward_route_lane_sequence(
  lanelet::Id reference_lane_id, double downstream_length_m) const
{
  lanelet::ConstLanelets sequence;
  const auto reference_lane_opt = get_lanelet(reference_lane_id);
  if (!reference_lane_opt) {
    return sequence;
  }
  sequence.push_back(*reference_lane_opt);
  std::unordered_set<lanelet::Id> visited_ids{reference_lane_id};
  lanelet::Id current_id = reference_lane_id;

  // Walk the on-route straight continuation, adding a full window of downstream length beyond the
  // reference lane: the ego can be anywhere along the reference lane, so covering that extra window
  // guarantees an object (or a dodge crossing) up to downstream_length_m ahead of the ego is in the
  // sequence. Under the onset scope gate a route-primitive successor exists at each straight step.
  double downstream_length = 0.0;
  while (downstream_length < downstream_length_m) {
    const auto next_ids = next_lane_ids(current_id);
    const auto next_on_route =
      std::find_if(next_ids.cbegin(), next_ids.cend(), [&](const lanelet::Id next_id) {
        return visited_ids.count(next_id) == 0 && is_route_primitive(next_id);
      });
    if (next_on_route == next_ids.cend()) {
      break;
    }
    const auto next_lane_opt = get_lanelet(*next_on_route);
    if (!next_lane_opt) {
      break;
    }
    sequence.push_back(*next_lane_opt);
    downstream_length += lanelet::geometry::length2d(*next_lane_opt);
    visited_ids.insert(*next_on_route);
    current_id = *next_on_route;
  }
  return sequence;
}

bool LaneTracker::is_footprint_fully_inside_lane(
  lanelet::Id lane_id, const std::vector<lanelet::BasicPoint2d> & footprint) const
{
  const auto lane = get_lanelet(lane_id);
  if (!lane) {
    return false;
  }
  return lane_event_classifier::is_footprint_fully_inside_lane(*lane, footprint);
}

bool LaneTracker::is_route_primitive(lanelet::Id lane_id) const
{
  return route_primitive_ids_.count(lane_id) != 0;
}

bool LaneTracker::is_lane_directly_connected(
  lanelet::Id from_lane_id, lanelet::Id candidate_lane_id) const
{
  const auto next_ids = next_lane_ids(from_lane_id);
  return std::find(next_ids.cbegin(), next_ids.cend(), candidate_lane_id) != next_ids.cend();
}

std::optional<lanelet::Id> LaneTracker::select_current_lane_id(const LaneEventInput & input) const
{
  const auto & odom_pos = input.odometry_ptr->pose.pose.position;
  const lanelet::BasicPoint2d ego_pt{odom_pos.x, odom_pos.y};

  // Prefer a route primitive the ego is inside; R-tree query avoids a whole-map scan each cycle.
  // Not lanelet2_utils::get_road_lanelets_at: that filters by subtype=="road", excluding any
  // lanelet without that attribute, whereas the ego's actual containing lane must always win.
  const lanelet::BoundingBox2d query_box(ego_pt, ego_pt);
  std::optional<lanelet::Id> off_route_lane_id;
  for (const auto & candidate_lane : lanelet_map_ptr_->laneletLayer.search(query_box)) {
    if (!lanelet::geometry::inside(candidate_lane, ego_pt)) {
      continue;
    }
    if (is_route_primitive(candidate_lane.id())) {
      return candidate_lane.id();
    }
    if (!off_route_lane_id) {
      off_route_lane_id = candidate_lane.id();  // off-route lane the ego sits in
    }
  }
  if (off_route_lane_id) {
    return off_route_lane_id;
  }

  // Fall back to the nearest route primitive (ego between lanelets, e.g. at a boundary).
  std::vector<lanelet::Id> existing_route_ids;
  existing_route_ids.reserve(route_primitive_ids_.size());
  for (const auto id : route_primitive_ids_) {
    if (lanelet_exists(id)) {
      existing_route_ids.push_back(id);
    }
  }
  const auto route_lanelets = lanelet2_utils::from_ids(lanelet_map_ptr_, existing_route_ids);
  const auto nearest_lane =
    lanelet2_utils::get_closest_lanelet(route_lanelets, input.odometry_ptr->pose.pose);
  return nearest_lane ? std::optional<lanelet::Id>{nearest_lane->id()} : std::nullopt;
}

tl::expected<void, std::string> LaneTracker::update(const LaneEventInput & input)
{
  if (!input.route_ptr) {
    return tl::make_unexpected<std::string>("Empty route ptr.");
  }

  if (cached_route_ptr_ != input.route_ptr) {
    cached_route_ptr_ = input.route_ptr;
    route_primitive_ids_ = update_primitive_route_ids(cached_route_ptr_);
  }

  debug_is_last_reanchor_blocked_ = false;

  // A held reference lane is the normal state during an event, not a failure; see README.md,
  // "Holding the reference lane".
  if (is_reference_lane_held_) {
    return {};
  }

  if (const auto updated_lane_id_opt = new_reference_lane_id(input)) {
    if (const auto lane_opt = get_lanelet(*updated_lane_id_opt)) {
      reference_lane_ = set_reference_lane(*lane_opt);
    }
  }

  return {};
}

std::unordered_set<lanelet::Id> LaneTracker::update_primitive_route_ids(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr & route_ptr) const
{
  std::unordered_set<lanelet::Id> primitive_route_ids;

  primitive_route_ids.reserve(route_ptr->segments.size());
  for (const auto & segment : route_ptr->segments) {
    primitive_route_ids.insert(static_cast<lanelet::Id>(segment.preferred_primitive.id));
  }

  return primitive_route_ids;
}

std::optional<lanelet::Id> LaneTracker::new_reference_lane_id(const LaneEventInput & input) const
{
  const auto selected_lane_id = select_current_lane_id(input);
  debug_last_selected_lane_id_ = selected_lane_id.value_or(lanelet::InvalId);
  if (!selected_lane_id) {
    return std::nullopt;  // keep the previous reference lane if we cannot resolve a lane this cycle
  }
  const lanelet::Id current_lane_id = *selected_lane_id;

  // A lateral move keeps the reference lane, so a lane change is not misread as forward progress.
  const bool is_reference_lane_unset = reference_lane_.reference_lane_id == lanelet::InvalId;
  const bool is_advancing_to_next_lane =
    !is_reference_lane_unset && current_lane_id != reference_lane_.reference_lane_id &&
    is_lane_directly_connected(reference_lane_.reference_lane_id, current_lane_id);
  // A shoulder has no routing successor; see README.md, "Holding the reference lane".
  const bool is_leaving_road_shoulder = !is_reference_lane_unset &&
                                        reference_lane_.is_reference_lane_road_shoulder &&
                                        current_lane_id != reference_lane_.reference_lane_id;
  if (!is_reference_lane_unset && !is_advancing_to_next_lane && !is_leaving_road_shoulder) {
    // Neither the reference lane nor a next lane of it, so the reference lane cannot advance.
    debug_is_last_reanchor_blocked_ = current_lane_id != reference_lane_.reference_lane_id;
    return std::nullopt;
  }

  return current_lane_id;
}

ReferenceLane LaneTracker::set_reference_lane(
  const lanelet::ConstLanelet & new_reference_lane) const
{
  ReferenceLane reference_lane;
  reference_lane.reference_lane_id = new_reference_lane.id();
  reference_lane.debug_is_reference_lane_on_route = is_route_primitive(new_reference_lane.id());

  reference_lane.is_reference_lane_road_shoulder =
    lanelet2_utils::is_shoulder_lane(new_reference_lane);
  reference_lane.is_reference_lane_intersection =
    lanelet2_utils::is_intersection_lanelet(new_reference_lane);
  reference_lane.is_reference_lane_left_bound_virtual =
    is_virtual_linestring(new_reference_lane.leftBound());
  reference_lane.is_reference_lane_right_bound_virtual =
    is_virtual_linestring(new_reference_lane.rightBound());

  return reference_lane;
}

}  // namespace lane_event_classifier
