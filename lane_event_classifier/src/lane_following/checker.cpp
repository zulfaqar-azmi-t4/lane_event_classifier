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

#include <lane_event_classifier/lane_following/checker.hpp>

#include <utility>

namespace lane_event_classifier
{

LaneFollowingChecker::LaneFollowingChecker(
  LaneFollowingConfig config, LaneFollowingGeometry geometry)
: config_{config}, geometry_{std::move(geometry)}
{
}

LaneFollowingResult LaneFollowingChecker::evaluate(
  const LaneTracker & tracker, const LaneEventContext & context,
  const lanelet::BasicPoint2d & ego_point) const
{
  // Attribute flags were resolved by the tracker when it anchored onto the reference lane.
  const auto & reference = tracker.reference_lane();

  // no_reference_lane (docs/lane_following.md, "No reference lane").
  if (!tracker.get_lanelet(reference.reference_lane_id)) {
    return {true, LaneFollowingReason::no_reference_lane};
  }

  const auto & sequence_ids = context.sequence_ids(config_.connected_sequence_length_m);
  const auto following_reason =
    geometry_.is_lane_following(tracker.lanelet_map_ptr(), reference, sequence_ids, ego_point);
  if (following_reason) {
    return {true, *following_reason};
  }
  return {false, LaneFollowingReason::departed};
}

}  // namespace lane_event_classifier
