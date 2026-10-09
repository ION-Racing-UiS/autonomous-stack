# Localization Interface Proposal

## Inputs
| Topic                        | Type                          | Used for                                      |
|------------------------------|-------------------------------|-----------------------------------------------|
| /imu                         | sensor_msgs/msg/Imu           | Angular Velocity and Linear Acceleration      |
| /gps                         | sensor_msgs/msg/NavSatFix     | Global Position                               |
| /wheel_states                | fs_msgs/msg/WheelStates       | Wheel Speeds (Possibly also steering angle)   |
|------------------------------|-------------------------------|-----------------------------------------------|

## Outputs
| Topic                        | Type                          | Used for                                      |
|------------------------------|-------------------------------|-----------------------------------------------|
| localization/state_estimate  | nav_msgs/msg/Odometry         | Estimated Vehicle Pose                        |
|------------------------------|-------------------------------|-----------------------------------------------|

## Notes
- No custom messages should be needed for state estimation.
- May need to add another topic and message later for SLAM, nav_msgs/msg/Trajectory could be a possibility.
- testing_only/odom may be used to evaluate state estimates.