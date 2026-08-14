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

#ifndef LANE_EVENT_CLASSIFIER__DETAIL__GEOMETRY_UTILS_HPP_
#define LANE_EVENT_CLASSIFIER__DETAIL__GEOMETRY_UTILS_HPP_

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <lane_event_classifier/detail/lane_tracker.hpp>
#include <lane_event_classifier/types.hpp>
#include <rclcpp/time.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

namespace lane_event_classifier
{

/**
 * @brief Returns the vehicle footprint polygon in the map frame, or a single centre point when
 * vehicle info is unavailable.
 * @param input Per-cycle input providing the ego pose and optional vehicle info.
 */
inline std::vector<lanelet::BasicPoint2d> compute_footprint(const LaneEventInput & input)
{
  const auto & pose = input.odometry_ptr->pose.pose;

  if (!input.vehicle_info_ptr) {
    return {{pose.position.x, pose.position.y}};
  }

  const auto map_footprint = autoware_utils_geometry::transform_vector(
    input.vehicle_info_ptr->createFootprint(), autoware_utils_geometry::pose2transform(pose));

  std::vector<lanelet::BasicPoint2d> map_footprint_2d;
  map_footprint_2d.reserve(map_footprint.size());
  std::transform(
    map_footprint.cbegin(), map_footprint.cend(), std::back_inserter(map_footprint_2d),
    [](const auto & footprint_point) {
      return lanelet::BasicPoint2d{footprint_point.x(), footprint_point.y()};
    });
  return map_footprint_2d;
}

/**
 * @brief Returns true only if every footprint corner lies inside the lanelet.
 * @param lane Lanelet to test against.
 * @param footprint Footprint corners in the map frame.
 */
inline bool is_footprint_fully_inside_lane(
  const lanelet::ConstLanelet & lane, const std::vector<lanelet::BasicPoint2d> & footprint)
{
  return std::all_of(
    footprint.cbegin(), footprint.cend(), [&lane](const lanelet::BasicPoint2d & footprint_corner) {
      return lanelet::geometry::inside(lane, footprint_corner);
    });
}

/**
 * @brief Returns true if the boundary linestring is virtual (attribute type == "virtual").
 * @param bound Boundary linestring to test.
 */
inline bool is_virtual_linestring(const lanelet::ConstLineString3d & bound)
{
  // std::string default: a const char* default would make Attribute::as<> compare by pointer.
  return bound.attributeOr(lanelet::AttributeName::Type, std::string{}) ==
         lanelet::AttributeValueString::Virtual;
}

/** @brief This cycle's odometry stamp, in seconds. */
inline double stamp_to_seconds(const LaneEventInput & input)
{
  return rclcpp::Time(input.odometry_ptr->header.stamp).seconds();
}

/** @brief Whether the turn blinker is on toward the given crossing side.
 *
 * Shared by the lane-change and lane-crossing classifiers' confidence signal: the driver blinks
 * toward the side the crossing/dodge heads to.
 */
inline bool is_blinker_toward_side(bool is_to_left, uint8_t turn_indicator)
{
  using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
  return is_to_left ? turn_indicator == TurnIndicatorsReport::ENABLE_LEFT
                    : turn_indicator == TurnIndicatorsReport::ENABLE_RIGHT;
}

/** @brief Ego pose, speed and stamp of one cycle, as used to detect a localization discontinuity.
 */
struct EgoMotionSample
{
  lanelet::BasicPoint2d position;
  double speed_mps{0.0};
  double stamp_s{0.0};
};

/**
 * @brief Returns true when the step between two ego samples exceeds the motion their speed
 * explains.
 * @param previous Ego sample of the preceding cycle.
 * @param current Ego sample of this cycle.
 * @param noise_margin_m Localization-noise margin allowed on top of the explainable motion.
 *
 * See README.md, "Parameters" (`reposition_jump_margin_m`): a fixed distance threshold cannot tell
 * a reposition from normal driving, so the step is compared against speed * dt plus the margin.
 */
inline bool is_reposition_jump(
  const EgoMotionSample & previous, const EgoMotionSample & current, double noise_margin_m)
{
  const double elapsed_s = current.stamp_s - previous.stamp_s;
  if (elapsed_s <= 0.0) {
    return false;
  }
  const double measured_step_m = (current.position - previous.position).norm();
  // Braking reports a speed the step just taken predates, so the faster sample explains it.
  const double explaining_speed_mps = std::max(previous.speed_mps, current.speed_mps);
  return measured_step_m > explaining_speed_mps * elapsed_s + noise_margin_m;
}

/**
 * @brief True when the reference lane's reanchoring is blocked (the ego's current lane is not a
 * forward successor of it) and the ego also sits far from it.
 *
 * The tracker only ever re-anchors on forward progress (never a lateral or backward move), so once
 * blocked with no held event to eventually release it, the reference lane would otherwise stay
 * pinned forever. See README.md, "Holding the reference lane".
 * @param is_reanchor_blocked LaneTracker::debug_is_last_reanchor_blocked() this cycle.
 * @param distance_to_reference_m Distance from the ego to the reference lane, or nullopt if
 * unknown.
 * @param reset_distance_m Distance beyond which the ego counts as far from the reference lane.
 */
inline bool is_stuck_and_far_from_reference(
  bool is_reanchor_blocked, std::optional<double> distance_to_reference_m, double reset_distance_m)
{
  return is_reanchor_blocked && distance_to_reference_m &&
         *distance_to_reference_m > reset_distance_m;
}

/** @brief True when the reference lane is a route primitive whose straight successor is also a
 * route primitive, so going straight stays on-route.
 * @param reference_lane_id Lane id to test.
 *
 * Shared by the lane-change and lane-crossing geometry layers: the exact complement of each other's
 * scope condition, so the two classifiers never double-classify.
 */
inline bool driving_straight_stays_on_route(
  const LaneTracker & tracker, lanelet::Id reference_lane_id)
{
  if (!tracker.is_route_primitive(reference_lane_id)) {
    return false;
  }
  const auto next_ids = tracker.next_lane_ids(reference_lane_id);
  return std::any_of(next_ids.cbegin(), next_ids.cend(), [&tracker](const lanelet::Id next_id) {
    return tracker.is_route_primitive(next_id);
  });
}

/**
 * @brief Ordered trajectory points from the one nearest the ego, forward, until the accumulated arc
 * length reaches look_ahead_m.
 * @param trajectory Planned trajectory.
 * @param ego Ego reference point in the map frame.
 * @param look_ahead_m Arc length ahead to keep.
 *
 * Shared by the lane-change and lane-crossing geometry layers, whose onset is predictive over the
 * planned trajectory.
 */
inline std::vector<lanelet::BasicPoint2d> forward_trajectory_points(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const lanelet::BasicPoint2d & ego,
  double look_ahead_m)
{
  if (trajectory.points.empty()) {
    return {};
  }
  geometry_msgs::msg::Point ego_point;
  ego_point.x = ego.x();
  ego_point.y = ego.y();
  const auto nearest_index = autoware::motion_utils::findNearestIndex(trajectory.points, ego_point);
  const auto cropped = autoware::motion_utils::cropPoints(
    trajectory.points, ego_point, nearest_index, look_ahead_m, 0.0);

  std::vector<lanelet::BasicPoint2d> points;
  points.reserve(cropped.size());
  std::transform(
    cropped.cbegin(), cropped.cend(), std::back_inserter(points),
    [](const auto & trajectory_point) {
      return lanelet::BasicPoint2d{
        trajectory_point.pose.position.x, trajectory_point.pose.position.y};
    });
  return points;
}

/**
 * @brief forward_trajectory_points() fed from a cycle's LaneEventInput, or empty if either the
 * trajectory or odometry is not yet available.
 * @param input Per-cycle input.
 * @param look_ahead_m Arc length ahead to keep.
 */
inline std::vector<lanelet::BasicPoint2d> forward_trajectory_points_from_input(
  const LaneEventInput & input, double look_ahead_m)
{
  if (!input.trajectory_ptr || !input.odometry_ptr) {
    return {};
  }
  const auto & ego_position = input.odometry_ptr->pose.pose.position;
  return forward_trajectory_points(
    *input.trajectory_ptr, {ego_position.x, ego_position.y}, look_ahead_m);
}

}  // namespace lane_event_classifier

#endif  // LANE_EVENT_CLASSIFIER__DETAIL__GEOMETRY_UTILS_HPP_
