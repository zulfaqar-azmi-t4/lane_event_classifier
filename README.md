# lane_event_classifier

Classifies the ego vehicle's lane event (lane following, lane change, intentional lane crossing) from
the planned trajectory, odometry, route, and lanelet map, and publishes the result for downstream
event logging.

## Packages

| Package                                   | Contents                                                                  |
| ----------------------------------------- | ------------------------------------------------------------------------- |
| `lane_event_classifier_msgs`              | `DrivingState` and `DrivingFactor` message definitions                    |
| `lane_event_classifier`                   | The classifier node, its parameters, launch file, and per-classifier docs |
| `tier4_lane_event_classifier_rviz_plugin` | RViz panel that displays the current driving state                        |

Depend on `lane_event_classifier_msgs` alone if you only need to publish or subscribe to the
messages. See [`lane_event_classifier/README.md`](lane_event_classifier/README.md) for topics,
parameters, and the classification logic.

## Building

```bash
# Import build dependencies
vcs import src < build_depends.repos

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-up-to lane_event_classifier tier4_lane_event_classifier_rviz_plugin
```

## Contributing

Commits must be signed off (Developer Certificate of Origin):

```bash
git commit --signoff
```

To sign off commits that are already written:

```bash
git rebase --signoff origin/main
```

Run `pre-commit install` once in your clone so the lint hooks in `.pre-commit-config.yaml` run
before each commit.
