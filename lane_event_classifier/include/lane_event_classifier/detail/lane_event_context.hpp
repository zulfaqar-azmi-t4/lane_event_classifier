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

#ifndef LANE_EVENT_CLASSIFIER__DETAIL__LANE_EVENT_CONTEXT_HPP_
#define LANE_EVENT_CLASSIFIER__DETAIL__LANE_EVENT_CONTEXT_HPP_

#include <lane_event_classifier/detail/lane_sequence.hpp>
#include <lane_event_classifier/detail/lane_tracker.hpp>
#include <lane_event_classifier/types.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <map>
#include <optional>
#include <unordered_set>
#include <vector>

namespace lane_event_classifier
{

/** @brief Reference-lane geometry derived once per cycle and shared by every consumer. */
class LaneEventContext
{
public:
  /** @brief Refreshes the derived geometry from the already-updated tracker. */
  void update(const LaneTracker & tracker, const LaneEventInput & input);

  /** @brief Ids of every lane that at least one footprint corner lies inside. */
  [[nodiscard]] const std::vector<lanelet::Id> & footprint_lane_ids() const
  {
    return footprint_lane_ids_;
  }

  /** @brief Straight sequence ids of the reference lane at the given fore/aft reach. */
  [[nodiscard]] const std::unordered_set<lanelet::Id> & sequence_ids(double reach_m) const;

private:
  std::vector<lanelet::Id> footprint_lane_ids_;
  std::optional<lanelet::ConstLanelet> reference_lane_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_;

  // One memo per distinct reach; each survives across cycles while lane and graph hold.
  mutable std::map<double, StraightLaneSequenceCache> sequence_caches_;
};

}  // namespace lane_event_classifier

#endif  // LANE_EVENT_CLASSIFIER__DETAIL__LANE_EVENT_CONTEXT_HPP_
