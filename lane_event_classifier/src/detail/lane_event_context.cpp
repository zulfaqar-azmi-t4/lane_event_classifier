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

#include <lane_event_classifier/detail/lane_event_context.hpp>

#include <unordered_set>

namespace lane_event_classifier
{

void LaneEventContext::update(const LaneTracker & tracker, const LaneEventInput & input)
{
  footprint_lane_ids_ = tracker.footprint_lane_ids(input.footprint);
  reference_lane_ = tracker.get_lanelet(tracker.reference_lane().reference_lane_id);
  routing_graph_ptr_ = tracker.routing_graph_ptr();
}

const std::unordered_set<lanelet::Id> & LaneEventContext::sequence_ids(double reach_m) const
{
  static const std::unordered_set<lanelet::Id> empty_ids;
  if (!reference_lane_) {
    return empty_ids;
  }
  return sequence_caches_[reach_m].get(*reference_lane_, routing_graph_ptr_, reach_m);
}

}  // namespace lane_event_classifier
