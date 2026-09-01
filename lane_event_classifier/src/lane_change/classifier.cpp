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

#include <lane_event_classifier/detail/geometry_utils.hpp>
#include <lane_event_classifier/lane_change/classifier.hpp>

#include <fmt/format.h>

#include <optional>

namespace lane_event_classifier
{

LaneChangeClassifier::LaneChangeClassifier(
  bool enabled, LaneChangeConfig config, const LaneTracker & tracker)
: enabled_{enabled}, config_{config}, tracker_{tracker}, geometry_{config.crossing_look_ahead_m}
{
}

void LaneChangeClassifier::reset_timers()
{
  crossing_signal_.reset();
  settle_signal_.reset();
  abort_signal_.reset();
}

bool LaneChangeClassifier::accumulate_crossing(
  const LaneChangeCrossing & crossing, double now_s, bool has_confidence_signal)
{
  // Both-persistence (docs/lane_change.md, "Persistence and confidence"): stable target and point.
  const auto matches_tracked =
    [this](const LaneChangeCrossing & tracked, const LaneChangeCrossing & current) {
      return tracked.target_lane_id == current.target_lane_id &&
             (current.crossing_point - tracked.crossing_point).norm() <=
               config_.crossing_position_tolerance_m;
    };

  // Confidence signal (docs/lane_change.md, "Persistence and confidence"): shortens the window.
  const double effective_persist_duration =
    config_.crossing_persist_duration_s * (has_confidence_signal ? config_.confidence_factor : 1.0);
  return crossing_signal_.update(crossing, now_s, effective_persist_duration, matches_tracked);
}

bool LaneChangeClassifier::has_confidence_signal(
  const LaneEventInput & input, const LaneChangeObservation & observation,
  const LaneChangeCrossing & crossing)
{
  // Confidence signal (docs/lane_change.md, "Persistence and confidence"): footprint or blinker.
  return observation.is_footprint_off_route_primitives ||
         is_blinker_toward_side(crossing.is_to_left, input.turn_indicator);
}

void LaneChangeClassifier::update(const LaneEventInput & input, const LaneEventContext & context)
{
  const double now_s = stamp_to_seconds(input);
  const LaneChangeObservation observation = geometry_.observe(tracker_, input, context);

  switch (phase_) {
    case Phase::idle:
      detect_onset(input, observation, now_s);
      break;
    case Phase::changing:
      detect_completion_or_abort(observation, now_s);
      break;
    case Phase::aborting:
      detect_abort_completion_or_recommit(input, observation, now_s);
      break;
  }
}

void LaneChangeClassifier::detect_onset(
  const LaneEventInput & input, const LaneChangeObservation & observation, double now_s)
{
  if (!observation.crossing) {
    crossing_signal_.reset();
    return;
  }
  const bool confidence = has_confidence_signal(input, observation, *observation.crossing);
  if (accumulate_crossing(*observation.crossing, now_s, confidence)) {
    debug_reason_ = fmt::format(
      "onset: trajectory crossing to lane {} persisted",
      crossing_signal_.tracked()->target_lane_id);
    phase_ = Phase::changing;
    reset_timers();
  }
}

void LaneChangeClassifier::detect_completion_or_abort(
  const LaneChangeObservation & observation, double now_s)
{
  // Completion (docs/lane_change.md, "Finishing or aborting"): footprint settled in the target.
  const auto same_lane = [](lanelet::Id tracked, lanelet::Id current) {
    return tracked == current;
  };
  if (settle_signal_.update(
        observation.settle_lane_id, now_s, config_.settle_confirm_duration_s, same_lane)) {
    debug_reason_ =
      fmt::format("settle: footprint fully inside route primitive {}", *settle_signal_.tracked());
    phase_ = Phase::idle;
    reset_timers();
    return;
  }
  // Abort onset (docs/lane_change.md, "Finishing or aborting"): trajectory heads back, persisted.
  if (persists(
        abort_signal_, observation.trajectory_returns_to_reference, now_s,
        config_.crossing_persist_duration_s)) {
    debug_reason_ = "abort: trajectory returned toward the reference lane";
    phase_ = Phase::aborting;
    reset_timers();
  }
}

void LaneChangeClassifier::detect_abort_completion_or_recommit(
  const LaneEventInput & input, const LaneChangeObservation & observation, double now_s)
{
  // Abort completion (docs/lane_change.md, "Aborting"): footprint fully back in the reference.
  if (observation.is_footprint_inside_reference_lane) {
    debug_reason_ = "abort completed: footprint fully back inside the reference lane";
    phase_ = Phase::idle;
    reset_timers();
    return;
  }
  // A settle stays live from ABORTING (docs/lane_change.md, "Settle still counts").
  const auto same_lane = [](lanelet::Id tracked, lanelet::Id current) {
    return tracked == current;
  };
  if (settle_signal_.update(
        observation.settle_lane_id, now_s, config_.settle_confirm_duration_s, same_lane)) {
    debug_reason_ = fmt::format(
      "settle from aborting: footprint fully inside route primitive {}", *settle_signal_.tracked());
    phase_ = Phase::idle;
    reset_timers();
    return;
  }
  // Re-commit (docs/lane_change.md, "Aborting"): trajectory swings back, persisted like onset.
  if (!observation.crossing) {
    crossing_signal_.reset();
    return;
  }
  const bool confidence = has_confidence_signal(input, observation, *observation.crossing);
  if (accumulate_crossing(*observation.crossing, now_s, confidence)) {
    debug_reason_ = fmt::format(
      "re-commit: trajectory crossing to lane {} persisted",
      crossing_signal_.tracked()->target_lane_id);
    phase_ = Phase::changing;
    reset_timers();
  }
}

std::optional<uint8_t> LaneChangeClassifier::get_state() const
{
  switch (phase_) {
    case Phase::changing:
      return DrivingState::LANE_CHANGING;
    case Phase::aborting:
      return DrivingState::ABORTING_LANE_CHANGE;
    case Phase::idle:
    default:
      // No lane-change event; the node falls back to the lane-following check.
      return std::nullopt;
  }
}

bool LaneChangeClassifier::is_enabled() const
{
  return enabled_;
}

}  // namespace lane_event_classifier
