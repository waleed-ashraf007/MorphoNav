<img width="478" height="480" alt="image" src="https://github.com/user-attachments/assets/52fbbe27-4f7a-41f8-8ad7-1c5e48a32751" />



MorphoNav is built around the multi-modal mobility idea: a single robot platform that can adapt how it moves to the terrain it faces. Wheels are fast and efficient on open ground, while a legged body adds flexibility in rough or confined spaces. The autonomy stack in this repository currently drives the robot on its wheels, with the legs held in a fixed posture.

Target applications

Search and rescue in rubble and debris
Disaster response in hazardous areas
Planetary and rough-terrain exploration
Autonomous inspection and patrol of facilities
Navigation in confined or changing environments

Architecture. The software is split into four layers that communicate over ROS 2, so each can be developed and tested on its own:

Layer	Responsibility
Perception	Sensor drivers and sensor fusion (LiDAR, IMU, GPS)
Localization and mapping	SLAM and EKF-based pose estimation
Planning	Global and local path planning with costmaps
Control	Velocity commands to the embedded controller, which drives the motors and servos
Highlights
Full navigation stack on ROS 2 Humble: SLAM, localization, global and local planning, costmaps, and simulation.
Sensor fusion: LiDAR scan matching is the primary localization source. IMU and GPS are fused through an EKF (robot_localization) as supporting odometry.
Simulation-first workflow: separate launch files for Gazebo and the real robot, with use_sim_time handled consistently in each.
Real hardware deployment: runs on a Raspberry Pi 4 and talks to an ESP32 through a micro-ROS bridge.
Measured on the robot: 5 Hz map refresh and a global path replan time of about 1.5 s.
CAD to robot model: SolidWorks assembly converted to URDF with the sw2urdf plugin, including joint axis mapping and mesh scale fixes.
System overview
 RPLiDAR A1 ──┐
 MPU-9250 IMU ─┼──> Raspberry Pi 4 (ROS 2 Humble, Ubuntu 22.04)
 NEO-7M GPS ──┘     ├─ slam_toolbox        (mapping, scan matching)
                    ├─ robot_localization  (EKF fusion)
                    ├─ Nav2                (planning, costmaps, control)
                    └─ /cmd_vel ──> micro-ROS bridge ──> ESP32 ──> motors and servos

The autonomy layer publishes velocity commands on /cmd_vel. Low-level motor and joint control is handled by the embedded side of the project over micro-ROS.

Hardware
Component	Role
Raspberry Pi 4	On-robot compute, ROS 2 Humble on Ubuntu 22.04
ESP32-class MCU	Motor and servo control via micro-ROS
RPLiDAR A1	2D scanning, primary localization source
MPU-9250	IMU, orientation and motion for the EKF
u-blox NEO-7M	GPS, supporting odometry
4 legs x (3 servos + 1 wheel motor)	16 degrees of freedom

Known sensor limitations the design accounts for: GPS drift, IMU noise from motor vibration, and the A1's slow scan rate.
![Uploading image.png…]()

Software stack

ROS 2 Humble, Nav2, slam_toolbox, robot_localization, AMCL, explore_lite (frontier exploration), Gazebo, RViz2, micro-ROS, URDF/xacro.

Key engineering decisions
Why EKF fusion instead of wheel odometry. Wheel encoders are used only for motor PID. Odometry comes from fused LiDAR, IMU, and GPS data.
Posture hold during driving. A dedicated node locks the leg joints while the robot moves on wheels, so navigation sees a rigid base.
Tunable parameters. Values such as LINEAR_SCALE, ANGULAR_SCALE, MAX_SPEED, and WHEEL_BASE are ROS 2 parameters in a YAML config, so the robot can be retuned without code changes.
Consistent simulation time. All nodes in a launch file must agree on use_sim_time. A mismatch produces stale TF data and breaks SLAM and Nav2.
Scan consistency. The RPLiDAR A1 can return inconsistent scan counts, so a LaserScanAngularBoundsFilter enforces consistent readings per revolution.
Planner comparison. Dijkstra and A* were compared for global planning.
Repository structure
Folder	Contents
bringup/	Launch files for starting the robot stack
description/	Robot model (URDF and meshes)
firmware/morphonav_esp32/	ESP32 firmware
posture/	Leg posture control
sensors/	Sensor configuration
simulations/	Gazebo simulation
Getting started

Prerequisites: Ubuntu 22.04, ROS 2 Humble, xacro, Nav2, slam_toolbox, robot_localization.

bash
# Clone into a ROS 2 workspace
mkdir -p ~/morphonav_ws/src && cd ~/morphonav_ws/src
git clone https://github.com/waleed-ashraf007/MorphoNav.git

# Build
cd ~/morphonav_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
<!-- TODO: add the exact launch commands for simulation and real robot, from the files in bringup/ -->
Results
<!-- TODO: add screenshots (RViz, SLAM map, costmap, TF tree) and a demo video link -->
Metric	Value
Map refresh rate	5 Hz
Global path replan time	~1.5 s
My contribution

I led the autonomy side of the project: perception, SLAM, localization, path planning, Nav2 integration, and Gazebo simulation. I also built the Nav2 integration layer (differential drive controller and posture hold node) and converted the SolidWorks CAD into the URDF model. Low-level motor control on the ESP32 was developed by the embedded team.

Documentation

A 43-page technical report covers the URDF model, EKF derivation, AMCL, Dijkstra vs. A*, localization approaches, and frontier exploration.

<!-- TODO: link the PDF once it is added to docs/ -->
Roadmap
Complete the Nav2 go-to-goal pipeline in simulation, then on the physical robot.
Browser-based remote interface using rosbridge_suite, roslibjs, and ros2djs, with the Pi acting as a WiFi hotspot for field use.
Author

Waleed Abdelnabi, final-year engineering student, Cairo University.
