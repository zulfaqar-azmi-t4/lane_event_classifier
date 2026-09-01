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

#ifndef LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__GEOMETRY_HPP_
#define LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__GEOMETRY_HPP_

#include <lane_event_classifier/detail/lane_tracker.hpp>
#include <lane_event_classifier/lane_event_classifier_parameters.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>
#include <string_view>
#include <unordered_set>

namespace lane_event_classifier
{
using LaneFollowingConfig = ::lane_event_classifier::Params::LaneFollowing;

/** @brief Which rule decided the lane-following outcome (debug tracing / logging only). */
enum class LaneFollowingReason {
  no_reference_lane,          // no reference lane yet -> treated as following
  inside_connected_sequence,  // reference point inside a lane of the connected sequence
  within_lateral_tolerance,   // reference point within the lateral margin of the sequence
  road_shoulder_exempt,       // reference point overlaps a road shoulder
  turn_lane_exempt,           // reference_lane / current lane is a turn / intersection lane
  virtual_boundary_exempt,    // the boundary the ego crossed is virtual
  departed                    // none matched -> a real lateral departure (not following)
};

/** @brief Returns a short label for the reason (debug tracing / logging only). */
[[nodiscard]] std::string_view to_debug_string(LaneFollowingReason reason);

/** @brief The membership test and the exemption ladder behind the lane-following check. */
class LaneFollowingGeometry
{
public:
  LaneFollowingGeometry() = default;
  explicit LaneFollowingGeometry(LaneFollowingConfig config);

  /**
   * @brief Returns the rule that keeps the point following, or nullopt when it has departed.
   * @param map Lanelet map the sequence ids refer to.
   * @param reference Reference lane attribute flags resolved by the tracker.
   * @param sequence_ids Straight sequence of the reference lane.
   * @param point Ego reference point in the map frame.
   */
  [[nodiscard]] std::optional<LaneFollowingReason> is_lane_following(
    const lanelet::LaneletMapPtr & map, const ReferenceLane & reference,
    const std::unordered_set<lanelet::Id> & sequence_ids,
    const lanelet::BasicPoint2d & point) const;

private:
  LaneFollowingConfig config_{};
};

}  // namespace lane_event_classifier

#endif  // LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__GEOMETRY_HPP_
