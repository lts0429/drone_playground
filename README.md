# Introduction
Most of the agricultural drones rely on GPS for localization, which makes them not suitable for under canopy (GPS denied) environment. However in under canopy situation, there are enough objects for visual slam. This project applies ORB SLAM3 for localization.

Checkout the demonstration [here](https://www.youtube.com/watch?v=lQ9vAMirqSs)

![orb_slam3](doc/orb_slam3.gif)

# Milestones
- [x] setup gazebo environment
- [x] spawn drone and equipped with lidar and stereo camera
- [x] implement orb slam3 (stereo)
- [x] extract odometry from orb slam3 (stereo)
- [x] setup navigation stack

# Future Improvements
- Use stereo-inertial mode for better localization
- Apply 3D navigation stack

# Resources
1. ORB SLAM3 Ros2 Wrapper
https://github.com/zang09/ORB_SLAM3_ROS2/tree/humble
2. Nav2 Documentation
https://docs.nav2.org/index.html#

