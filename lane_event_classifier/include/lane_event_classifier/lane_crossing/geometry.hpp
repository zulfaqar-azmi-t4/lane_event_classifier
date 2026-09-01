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

#ifndef LANE_EVENT_CLASSIFIER__LANE_CROSSING__GEOMETRY_HPP_
#define LANE_EVENT_CLASSIFIER__LANE_CROSSING__GEOMETRY_HPP_

#include <lane_event_classifier/detail/lane_event_context.hpp>
#include <lane_event_classifier/detail/lane_tracker.hpp>
#include <lane_event_classifier/types.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace lane_event_classifier
{

/** @brief Where the planned trajectory crosses the reference lane's lateral boundary. */
struct LaneCrossingCrossing
{
  lanelet::Id target_lane_id{lanelet::InvalId};  // off-sequence lane sharing the crossed boundary
                                                 // (best-effort; InvalId if none is mapped)
  lanelet::BasicPoint2d crossing_point{
    0.0, 0.0};             // where the trajectory first crosses the reference boundary
  bool is_to_left{false};  // crossing is toward the reference lane's left side
};

/** @brief The boundary / footprint half of the per-cycle lane-crossing observation. */
struct LaneCrossingObservation
{
  // Onset: a predictive crossing that passed the scope gate and the exemptions; else nullopt.
  std::optional<LaneCrossingCrossing> crossing;
  // Return / completion: the footprint is fully inside a lane of the reference straight sequence.
  bool is_footprint_inside_reference_sequence{false};
  // Full-entry escape: a non-sequence lane the footprint is fully inside, so it is a lane change.
  std::optional<lanelet::Id> full_entry_lane_id;
  // Scope gate: the reference lane is on-route and going straight keeps the ego on-route.
  bool is_on_route_straight{false};
  // Human-readable breakdown of the crossing-detection check for this cycle (debug logging only).
  std::string debug_crossing_diagnostic;
};

/** @brief The boundary / footprint half of the lane-crossing policy layer. */
class LaneCrossingGeometry
{
public:
  LaneCrossingGeometry(
    double crossing_look_ahead_m, double footprint_boundary_overshoot_m,
    double predictive_lateral_trigger_distance_m, double footprint_crossing_object_proximity_m);

  /** @brief Builds the observation for this cycle from the tracker's (already refreshed) state.
   * @param candidate_object_poses Objects the ego might cross to avoid (from LaneCrossingObjects):
   * onset fires only when the trajectory brackets one of them. */
  [[nodiscard]] LaneCrossingObservation observe(
    const LaneTracker & tracker, const LaneEventInput & input, const LaneEventContext & context,
    const std::vector<geometry_msgs::msg::Pose> & candidate_object_poses) const;

private:
  /** @brief A crossing plus the per-cycle diagnostic that explains it (returned by value). */
  struct CrossingResult
  {
    std::optional<LaneCrossingCrossing> crossing;
    std::string debug_diagnostic;
  };

  /** @brief One cycle's crossing-detection inputs, gathered once by observe(). */
  struct CrossingRequest
  {
    const lanelet::ConstLanelet & reference_lane;
    // Forward trajectory samples over the planned arc length.
    const std::vector<lanelet::BasicPoint2d> & trajectory_points;
    // Ego footprint corners in the map frame (the physical body this cycle).
    const std::vector<lanelet::BasicPoint2d> & footprint;
    // Objects the ego might cross to avoid; onset requires at least one.
    const std::vector<geometry_msgs::msg::Pose> & candidate_object_poses;
    // Forward length the lane-sequence boundary is built to.
    double boundary_look_ahead_m{0.0};
  };

  /** @brief The valid lane-crossing crossing over the reference boundary (with its diagnostic).
   * Two sources, both gated on a candidate object to go around: (a) predictive - the planned
   * trajectory departs the lane sequence around the object (crosses a boundary out before it, back
   * in after it), detected while the ego body is still inside the lane; (b) physical - the current
   * ego footprint (the real body, so a yawed corner poking over the line counts) crosses the
   * boundary into a lane outside the sequence. A path that stays in the neighbour (never returns)
   * is left to the lane-change classifier. See docs/lane_crossing.md.
   * @param tracker Generic lane queries.
   * @param context Reference-lane geometry derived once for this cycle.
   * @param request This cycle's crossing-detection inputs. */
  [[nodiscard]] CrossingResult compute_crossing(
    const LaneTracker & tracker, const LaneEventContext & context,
    const CrossingRequest & request) const;

  /** @brief True when the footprint is fully inside a lane of the reference straight sequence.
   * @param footprint_ids Lanes the footprint touches (computed once per cycle by observe). */
  [[nodiscard]] static bool compute_is_footprint_inside_reference_sequence(
    const LaneTracker & tracker, const LaneEventInput & input,
    const std::unordered_set<lanelet::Id> & sequence_ids,
    const std::vector<lanelet::Id> & footprint_ids);

  /** @brief A non-sequence lane the footprint is fully inside (the full-entry escape), if any.
   * @param footprint_ids Lanes the footprint touches (computed once per cycle by observe). */
  [[nodiscard]] static std::optional<lanelet::Id> compute_full_entry_lane_id(
    const LaneTracker & tracker, const LaneEventInput & input,
    const std::unordered_set<lanelet::Id> & sequence_ids,
    const std::vector<lanelet::Id> & footprint_ids);

  // Fore/aft reach for the route-sequence membership set, and the boundary-length fallback.
  double crossing_look_ahead_m_;
  // Min body overshoot past the boundary for the physical (footprint) crossing source.
  double footprint_boundary_overshoot_m_;
  // Max ego-footprint distance to the crossed-side boundary for the predictive source to onset.
  double predictive_lateral_trigger_distance_m_;
  // Max distance from the poking footprint corner to the candidate object it dodges.
  double footprint_crossing_object_proximity_m_;
};

}  // namespace lane_event_classifier

#endif  // LANE_EVENT_CLASSIFIER__LANE_CROSSING__GEOMETRY_HPP_
