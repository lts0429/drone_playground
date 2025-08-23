![orb_slam3_rviz](doc/orb_slam3_rviz.gif)

# Introduction
This project explores methods and solutions for navigating drones beneath forest canopies, where GPS signals are weak and visibility is limited. The goal is to research and prototype approaches that enable drones to move safely and efficiently in complex, cluttered environments—unlocking applications in forestry, environmental monitoring, precision agriculture, and search & rescue.

# Milestones
- [x] setup gazebo environment
- [x] spawn drone and equipped with lidar and stereo camera
- [x] setup EKF for localization
- [x] setup Nav2
- [x] implement orb slam3 (stereo)
- [x] extract odometry from orb slam3 (stereo)

# Todos
- [ ] use ekf to fuse imu and orb slam3