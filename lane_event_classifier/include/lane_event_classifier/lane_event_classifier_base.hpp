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

#include <lane_event_classifier/types.hpp>

#include <cstdint>
#include <string>

namespace lane_event_classifier
{

/** @brief Interface for lane-event classifiers.
 *
 * Each classifier holds the (generic) LaneTracker it depends on and derives its own per-cycle
 * geometry from the tracker's queries, so no shared per-cycle context is passed here.
 */
class LaneEventClassifierBase
{
public:
  virtual ~LaneEventClassifierBase() = default;

  /**
   * @brief Updates the classifier with the latest cycle's input.
   * @param input Per-cycle subscribed inputs and footprint.
   */
  virtual void update(const LaneEventInput & input) = 0;

  /** @brief Returns the current DrivingState of this classifier. */
  [[nodiscard]] virtual uint8_t get_state() const = 0;

  /** @brief Returns whether this classifier is enabled. */
  [[nodiscard]] virtual bool is_enabled() const = 0;

  /** @brief Short label used in the processing-time overlay and logs. */
  [[nodiscard]] virtual std::string name() const = 0;

  /** @brief One-line reason for the most recent state transition (debug/logging only). */
  [[nodiscard]] virtual std::string debug_reason() const { return {}; }
};

}  // namespace lane_event_classifier

#endif  // LANE_EVENT_CLASSIFIER__LANE_EVENT_CLASSIFIER_BASE_HPP_
