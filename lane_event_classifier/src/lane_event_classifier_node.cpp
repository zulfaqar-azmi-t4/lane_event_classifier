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

#include "lane_event_classifier/lane_event_classifier_node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <lane_event_classifier/detail/geometry_utils.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lane_event_classifier
{

LaneEventClassifierNode::LaneEventClassifierNode(const rclcpp::NodeOptions & node_options)
: Node("lane_event_classifier", node_options),
  param_listener_{this->get_node_parameters_interface()},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()},
  debug_{*this}
{
  params_ = param_listener_.get_params();
  build_classifiers();

  sub_map_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "/map/vector_map", rclcpp::QoS{1}.transient_local(),
    [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg) {
      map_callback(msg);
    });

  pub_driving_factor_ = create_publisher<lane_event_classifier_msgs::msg::DrivingFactor>(
    "/planning/driving_factor", rclcpp::QoS{1});

  sub_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "/planning/trajectory", rclcpp::QoS{1},
    [this](const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & msg) {
      on_trajectory(msg);
    });
}

void LaneEventClassifierNode::build_classifiers()
{
  // Every subsystem is assembled the same way: config, then its policy layers by value.
  lane_following_checker_ =
    LaneFollowingChecker(params_.lane_following, LaneFollowingGeometry{params_.lane_following});

  // Vector order is the arbitration priority: lane change first, then intentional crossing.
  classifiers_.clear();
  classifiers_.emplace_back(
    std::make_unique<LaneChangeClassifier>(
      params_.lane_change.enable_classifier, params_.lane_change, lane_tracker_,
      LaneChangeGeometry{params_.lane_change.crossing_look_ahead_m}));
  classifiers_.emplace_back(
    std::make_unique<IntentionalCrossingClassifier>(
      params_.lane_crossing.enable_classifier, params_.lane_crossing, lane_tracker_,
      LaneCrossingGeometry{
        params_.lane_crossing.crossing_look_ahead_m,
        params_.lane_crossing.footprint_boundary_overshoot_m,
        params_.lane_crossing.predictive_lateral_trigger_distance_m,
        params_.lane_crossing.footprint_crossing_object_proximity_m},
      LaneCrossingObjects{params_.lane_crossing.object_longitudinal_window_m}));
}

void LaneEventClassifierNode::map_callback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg)
{
  const auto map_const = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg);
  auto map = autoware::experimental::lanelet2_utils::remove_const(map_const);

  const auto result = lane_tracker_.set_lanelet_map(map);
  if (!result) {
    throw std::runtime_error(result.error());
  }
}

tl::expected<void, std::string> LaneEventClassifierNode::take_data(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory_msg)
{
  input_.trajectory_ptr = trajectory_msg;

  const auto odometry_msg = sub_odometry_.take_data();
  if (!odometry_msg) {
    return tl::make_unexpected("odometry_msg not available");
  }
  input_.odometry_ptr = odometry_msg;

  auto objects_msg = sub_objects_.take_data();
  if (!objects_msg) {
    return tl::make_unexpected("perceived objects_msg not available");
  }
  input_.objects_ptr = objects_msg;

  if (!lane_tracker_.has_lanelet_map()) {
    return tl::make_unexpected("lanelet map not yet available");
  }

  auto route_msg = sub_route_.take_data();
  if (!route_msg) {
    return tl::make_unexpected("route_msg not available");
  }

  if (route_msg->segments.empty()) {
    return tl::make_unexpected("route_msg does not contain any segments");
  }

  if (!input_.route_ptr || route_msg->uuid != input_.route_ptr->uuid) {
    input_.route_ptr = route_msg;
  }

  if (!input_.vehicle_info_ptr) {
    input_.vehicle_info_ptr =
      std::make_unique<autoware::vehicle_info_utils::VehicleInfo>(vehicle_info_);
  }

  // Turn indicator is optional: keep the previous value rather than failing the cycle.
  if (const auto turn_indicators_msg = sub_turn_indicators_.take_data()) {
    input_.turn_indicator = turn_indicators_msg->report;
  }

  input_.footprint = compute_footprint(input_);

  return {};
}

tl::expected<void, std::string> LaneEventClassifierNode::check_tracking_state()
{
  const auto & odometry = *input_.odometry_ptr;
  const auto & ego_position = odometry.pose.pose.position;
  const auto & velocity = odometry.twist.twist.linear;
  const lanelet::BasicPoint2d ego_point{ego_position.x, ego_position.y};
  const EgoMotionSample ego_motion{
    ego_point, std::hypot(velocity.x, velocity.y), rclcpp::Time{odometry.header.stamp}.seconds()};

  // Trigger 1 — a reposition jump (localization discontinuity).
  const bool ego_jumped =
    previous_ego_motion_ &&
    is_reposition_jump(*previous_ego_motion_, ego_motion, params_.reposition_jump_margin_m);
  previous_ego_motion_ = ego_motion;
  if (ego_jumped) {
    return tl::make_unexpected("reposition jump (step exceeds reported motion)");
  }

  // Trigger 2 — a held reference lane never re-anchors, so reset once the ego strays past it.
  if (lane_tracker_.is_reference_lane_held()) {
    const auto distance_to_reference =
      lane_tracker_.distance_to_lane(lane_tracker_.reference_lane().reference_lane_id, ego_point);
    if (distance_to_reference && *distance_to_reference > params_.lane_departure_reset_distance_m) {
      return tl::make_unexpected("ego departed far from the held reference lane");
    }
    stuck_reanchor_signal_.reset();
    return {};
  }

  // Trigger 3 — an unheld reference lane with a blocked reanchor resets after the debounce.
  const auto distance_to_reference =
    lane_tracker_.distance_to_lane(lane_tracker_.reference_lane().reference_lane_id, ego_point);
  const bool is_stuck_and_far = is_stuck_and_far_from_reference(
    lane_tracker_.debug_is_last_reanchor_blocked(), distance_to_reference,
    params_.lane_departure_reset_distance_m);
  if (persists(
        stuck_reanchor_signal_, is_stuck_and_far, ego_motion.stamp_s,
        params_.stuck_reanchor_reset_duration_s)) {
    return tl::make_unexpected("reference lane stuck: unreachable forward and far from ego");
  }

  return {};
}

void LaneEventClassifierNode::reset_tracking_state(const std::string & tracking_reset_reason)
{
  build_classifiers();                     // restart classifiers from LANE_FOLLOWING
  lane_tracker_.release_reference_lane();  // re-anchor the reference lane to the new lane
  debug_.log_reset(tracking_reset_reason);
}

void LaneEventClassifierNode::on_trajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & trajectory_msg)
{
  if (param_listener_.is_old(params_)) {
    params_ = param_listener_.get_params();
    build_classifiers();
  }

  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  lane_event_classifier_msgs::msg::DrivingFactor out;
  out.header.stamp = trajectory_msg->header.stamp;
  out.header.frame_id = "map";

  const auto lane_event_inputs_updated = take_data(trajectory_msg);
  if (!lane_event_inputs_updated) {
    debug_.log_warn(lane_event_inputs_updated.error());
    out.driving_state.state = DrivingState::UNDEFINED;
    pub_driving_factor_->publish(out);
    return;
  }

  if (const auto is_tracking_ok = check_tracking_state(); !is_tracking_ok) {
    reset_tracking_state(is_tracking_ok.error());
  }

  stop_watch.tic("lane_tracker");
  if (const auto tracker_updated = lane_tracker_.update(input_); !tracker_updated) {
    debug_.log_warn(tracker_updated.error());
  }
  context_.update(lane_tracker_, input_);
  const double lane_tracker_time_ms = stop_watch.toc("lane_tracker");

  const auto & ego_position = input_.odometry_ptr->pose.pose.position;
  stop_watch.tic("lane_following");
  const auto lane_following_result =
    lane_following_checker_.evaluate(lane_tracker_, context_, {ego_position.x, ego_position.y});
  const double lane_following_time_ms = stop_watch.toc("lane_following");

  uint8_t current_state_val = DrivingState::LANE_FOLLOWING;
  bool is_any_event_active = false;
  std::vector<std::pair<std::string, double>> classifier_processing_times_ms;
  classifier_processing_times_ms.reserve(classifiers_.size());
  for (auto & classifier : classifiers_) {
    if (!classifier->is_enabled()) {
      continue;
    }
    stop_watch.tic(classifier->name());
    classifier->update(input_, context_);
    classifier_processing_times_ms.emplace_back(
      classifier->name(), stop_watch.toc(classifier->name()));
    const auto candidate_state = classifier->get_state();
    if (!candidate_state) {
      continue;  // the classifier claims no event this cycle
    }
    if (!is_any_event_active) {
      current_state_val = *candidate_state;  // first claiming classifier wins (priority order)
    }
    is_any_event_active = true;
  }
  // No confirmed event: fall back to the lane-following check; a departure is then UNKNOWN.
  if (!is_any_event_active && !lane_following_result.is_following) {
    current_state_val = DrivingState::UNKNOWN;
  }

  debug_.log_state(current_state_val, input_, lane_following_result, lane_tracker_, classifiers_);

  out.driving_state.state = current_state_val;
  pub_driving_factor_->publish(out);

  // Freeze the reference lane while an event runs (README.md, "Holding the reference lane").
  const auto & ref_lane = lane_tracker_.reference_lane();
  if (is_any_event_active && !lane_tracker_.is_reference_lane_held()) {
    lane_tracker_.hold_reference_lane();
  } else if (
    (!is_any_event_active || ref_lane.is_reference_lane_road_shoulder ||
     ref_lane.is_reference_lane_intersection) &&
    lane_tracker_.is_reference_lane_held()) {
    lane_tracker_.release_reference_lane();
  }

  const double total_time_ms = stop_watch.toc();

  std::vector<std::pair<std::string, double>> section_times_ms;
  section_times_ms.reserve(classifier_processing_times_ms.size() + 2);
  section_times_ms.emplace_back("lane_tracker", lane_tracker_time_ms);
  section_times_ms.emplace_back("lane_following", lane_following_time_ms);
  section_times_ms.insert(
    section_times_ms.end(), classifier_processing_times_ms.begin(),
    classifier_processing_times_ms.end());
  debug_.publish_processing_time(trajectory_msg->header.stamp, total_time_ms, section_times_ms);
}
}  // namespace lane_event_classifier

RCLCPP_COMPONENTS_REGISTER_NODE(lane_event_classifier::LaneEventClassifierNode)
