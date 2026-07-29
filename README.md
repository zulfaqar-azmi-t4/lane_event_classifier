# lane_event_classifier

Classifies the ego vehicle's lane event (lane following, lane change, intentional lane crossing) from
the planned trajectory, odometry, route, and lanelet map, and publishes the result for downstream
event logging.

| Package                                                                              | Contents                                                                         |
| ------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------- |
| [`lane_event_classifier_msgs`](lane_event_classifier_msgs)                           | `DrivingState` and `DrivingFactor` definitions.                                  |
| [`lane_event_classifier`](lane_event_classifier)                                     | The node and the classifiers. See its [README](lane_event_classifier/README.md). |
| [`tier4_lane_event_classifier_rviz_plugin`](tier4_lane_event_classifier_rviz_plugin) | RViz panel showing the state published on `/planning/driving_factor`.            |
