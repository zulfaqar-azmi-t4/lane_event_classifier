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

#ifndef LANE_EVENT_CLASSIFIER__LANE_CROSSING__OBJECTS_HPP_
#define LANE_EVENT_CLASSIFIER__LANE_CROSSING__OBJECTS_HPP_

#include <lane_event_classifier/detail/lane_tracker.hpp>
#include <lane_event_classifier/types.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <string>
#include <vector>

namespace lane_event_classifier
{

/** @brief The perceived-object half of the lane-crossing policy layer. */
class LaneCrossingObjects
{
public:
  explicit LaneCrossingObjects(double object_longitudinal_window_m);

  /** @brief The candidate objects for one cycle (returned by value; no out-params). */
  struct Result
  {
    // Map-frame poses of objects ahead on the sequence that the trajectory must bracket to onset.
    std::vector<geometry_msgs::msg::Pose> candidate_object_poses;
    // Human-readable breakdown of the candidate-object scan for this cycle (debug logging only).
    std::string debug_diagnostic;
  };

  /** @brief Observes the candidate objects the ego might cross to avoid this cycle. */
  [[nodiscard]] Result observe(const LaneTracker & tracker, const LaneEventInput & input) const;

private:
  using PredictedObject = autoware_perception_msgs::msg::PredictedObject;

  /** @brief Per-cycle tally of how the perceived objects sit relative to the lane sequence. */
  struct LaneSequenceScan
  {
    std::vector<geometry_msgs::msg::Pose>
      candidate_object_poses;                 // touch sequence, ahead in window
    int debug_lane_sequence_object_count{0};  // touch the lane sequence at all (any position)
    double debug_nearest_ahead_m{-1.0};  // arc distance to the nearest object ahead (-1 when none)
    double debug_nearest_ahead_speed_mps{0.0};
  };

  /** @brief Scans every object against the lane sequence and collects the candidate poses. */
  [[nodiscard]] LaneSequenceScan scan_lane_sequence_objects(
    const lanelet::ConstLanelets & lane_sequence, const std::vector<PredictedObject> & objects,
    double ego_arc_length_m) const;

  /** @brief True when the object lies ahead of the ego within the longitudinal window. */
  [[nodiscard]] bool object_is_ahead_within_window(double arc_distance_ahead_m) const;

  double object_longitudinal_window_m_;  // ahead-of-ego arc window for a candidate object
};

}  // namespace lane_event_classifier

#endif  // LANE_EVENT_CLASSIFIER__LANE_CROSSING__OBJECTS_HPP_
