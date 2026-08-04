# Senior Engineering Review & Retrospective

## System Evaluation

### What's Solid
- **Safety Isolation**: the deterministic safety monitor runs independently of AI perception and the higher-level ROS 2 action loops, so physical collision avoidance still holds up even if something up the stack crashes.
- **State Estimation**: EKF fusion of wheel odometry and IMU keeps orientation from drifting during hard turns or fast acceleration.
- **Pluggable Planners**: the custom C++ A* and Dijkstra planners slot in cleanly as standard Nav2 plugins.

### What's Fragile
- **2D LiDAR Assumption**: relies heavily on planar LiDAR, so it misses low-profile obstacles and anything overhanging above the scan height.
- **Odometry Slip on Wet/Uneven Ground**: pure wheel odometry degrades fast on slippery surfaces once the IMU signal drops out.

### To Actually Deploy This on Hardware
1. **Sensor Calibration**: run intrinsic and extrinsic calibration for the LiDAR-to-camera transforms.
2. **Real-time Kernel Tuning**: get PREEMPT_RT patched onto the Ubuntu host so the motor control loop latency stays under 2ms.
3. **CAN/Serial Drivers**: swap the Gazebo diff drive plugin for real motor controller hardware interfaces via `ros2_control`.
