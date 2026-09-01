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

#ifndef LANE_EVENT_CLASSIFIER__LANE_EVENT_CLASSIFIER_BASE_HPP_
#define LANE_EVENT_CLASSIFIER__LANE_EVENT_CLASSIFIER_BASE_HPP_

#include <lane_event_classifier/detail/lane_event_context.hpp>
#include <lane_event_classifier/types.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace lane_event_classifier
{

/** @brief Interface for lane-event classifiers. */
class LaneEventClassifierBase
{
public:
  virtual ~LaneEventClassifierBase() = default;

  /**
   * @brief Updates the classifier with the latest cycle's input.
   * @param input Per-cycle subscribed inputs and footprint.
   * @param context Reference-lane geometry derived once for this cycle.
   */
  virtual void update(const LaneEventInput & input, const LaneEventContext & context) = 0;

  /** @brief Returns this classifier's DrivingState, or nullopt when it claims no event. */
  [[nodiscard]] virtual std::optional<uint8_t> get_state() const = 0;

  /** @brief Returns whether this classifier is enabled. */
  [[nodiscard]] virtual bool is_enabled() const = 0;

  /** @brief Short label used in the processing-time overlay and logs. */
  [[nodiscard]] virtual std::string name() const = 0;

  /** @brief One-line reason for the most recent state transition (debug/logging only). */
  [[nodiscard]] virtual std::string debug_reason() const { return {}; }
};

}  // namespace lane_event_classifier

#endif  // LANE_EVENT_CLASSIFIER__LANE_EVENT_CLASSIFIER_BASE_HPP_
