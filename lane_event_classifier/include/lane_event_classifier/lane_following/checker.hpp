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

#ifndef LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__CHECKER_HPP_
#define LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__CHECKER_HPP_

#include <lane_event_classifier/detail/lane_event_context.hpp>
#include <lane_event_classifier/detail/lane_tracker.hpp>
#include <lane_event_classifier/lane_following/geometry.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <utility>

namespace lane_event_classifier
{

/** @brief Lane-following verdict plus the rule that decided it. */
struct LaneFollowingResult
{
  bool is_following{true};
  LaneFollowingReason debug_reason{LaneFollowingReason::no_reference_lane};
};

/** @brief Runs the lane-following check and reports the deciding rule (docs/lane_following.md). */
class LaneFollowingChecker
{
public:
  LaneFollowingChecker() = default;
  LaneFollowingChecker(LaneFollowingConfig config, LaneFollowingGeometry geometry);

  /** @brief Runs the check for the ego reference point against the reference lane. */
  [[nodiscard]] LaneFollowingResult evaluate(
    const LaneTracker & tracker, const LaneEventContext & context,
    const lanelet::BasicPoint2d & ego_point) const;

private:
  LaneFollowingConfig config_{};
  LaneFollowingGeometry geometry_;  // membership test and exemption ladder (injected)
};

}  // namespace lane_event_classifier

#endif  // LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__CHECKER_HPP_
