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

#ifndef LANE_EVENT_CLASSIFIER__DETAIL__DEBOUNCED_SIGNAL_HPP_
#define LANE_EVENT_CLASSIFIER__DETAIL__DEBOUNCED_SIGNAL_HPP_

#include <functional>
#include <optional>

namespace lane_event_classifier
{

/**
 * @brief Confirms a per-cycle sample only once it has persisted, unchanged, for a duration.
 *
 * Every classifier debounce (onset crossing, settle, return, abort) is the same shape: track a
 * value across cycles, restart the window whenever it goes away or changes, and confirm once the
 * window elapses. Feed one sample per cycle through update(); the persist duration may vary
 * per-call (e.g. shortened by a confidence signal).
 */
template <typename T>
class DebouncedSignal
{
public:
  /**
   * @brief Feeds one cycle's sample.
   * @param current This cycle's value, or std::nullopt if the condition is not met.
   * @param now_s Current time in seconds.
   * @param persist_duration_s Window the value must persist before confirming.
   * @param matches Whether `current` continues the tracked value, rather than restarting it.
   */
  bool update(
    std::optional<T> current, double now_s, double persist_duration_s,
    const std::function<bool(const T &, const T &)> & matches)
  {
    if (!current) {
      tracked_.reset();
      return false;
    }
    if (!tracked_ || !matches(*tracked_, *current)) {
      tracked_ = *current;
      start_s_ = now_s;
    }
    return (now_s - start_s_) >= persist_duration_s;
  }

  [[nodiscard]] const std::optional<T> & tracked() const { return tracked_; }

  void reset()
  {
    tracked_.reset();
    start_s_ = 0.0;
  }

private:
  std::optional<T> tracked_;
  double start_s_{0.0};
};

/**
 * @brief Confirms a plain boolean condition once it has held for a duration.
 *
 * The abort and return signals both debounce a bare bool rather than a value with its own equality
 * rule, so `matches` is always "yes, still the same condition" — this hides that idiom.
 * @param signal Signal to feed; reset once condition goes false.
 * @param condition Whether this cycle's condition holds.
 * @param now_s Current time in seconds.
 * @param persist_duration_s Window the condition must hold before confirming.
 */
inline bool persists(
  DebouncedSignal<bool> & signal, bool condition, double now_s, double persist_duration_s)
{
  const auto always_matches = [](bool, bool) { return true; };
  const std::optional<bool> value = condition ? std::optional{true} : std::nullopt;
  return signal.update(value, now_s, persist_duration_s, always_matches);
}

}  // namespace lane_event_classifier

#endif  // LANE_EVENT_CLASSIFIER__DETAIL__DEBOUNCED_SIGNAL_HPP_
