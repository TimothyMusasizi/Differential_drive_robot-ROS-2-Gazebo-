# Mobile Robot - ROS2 Gazebo Simulation

A differential drive mobile robot simulation built with ROS2 and Gazebo. This project demonstrates robot modeling with URDF/Xacro, launching simulation environments, and integrating ROS2 with the Gazebo physics engine.

## Overview

This is my first complete Gazebo simulation project using ROS2. The package includes:

- **Robot Model**: A differential drive robot with two drive wheels and a caster wheel
- **Simulation**: Gazebo integration using `ros_gz_sim` for physics simulation
- **ROS2 Integration**: Launch files and nodes to spawn the robot and manage the simulation

## Project Structure

```
mobile_robot/
├── model/
│   ├── robot.xacro        # Robot definition with parameters and plugins
│   └── robot.gazebo        # Gazebo-specific configuration
├── launch/
│   └── gazebo_model.launch.py  # Main launch file for Gazebo simulation
├── include/
├── src/                    # Source code directory (currently empty)
├── parameters/           # Contains YAML file for the parameter-bridge between ROS2 and Gazebo
├── CMakeLists.txt
└── package.xml
```

## Current Status

✅ **Working**
- Gazebo simulation launches successfully
- Robot model spawns correctly in the simulation world
- Robot state publisher and joint state publisher operate as expected

⚠️ **Known Issues**
- **Teleop Integration**: Commands from `teleop_twist_keyboard` are not being received by the simulated robot
  - The robot does not respond to velocity commands sent via the ROS2 topic
  - This appears to be a communication/bridge issue between the teleop node and the Gazebo simulation
  - Further investigation needed regarding the `ros_gz_bridge` configuration

## Building and Running

### Prerequisites
- ROS2 (tested with Jazzy)
- Gazebo (ros_gz_sim package)
- Xacro
- ros_gz_bridge

### Build

```bash
cd ~/ros_ws
colcon build --packages-select mobile_robot
source install/setup.bash
```

### Launch Simulation

```bash
ros2 launch mobile_robot gazebo_model.launch.py
```

This will:
1. Launch Gazebo with an empty world
2. Spawn the differential drive robot
3. Start the robot state publisher

## Dependencies

- `ament_cmake`
- `joint_state_publisher`
- `robot_state_publisher`
- `gazebo_ros`
- `xacro`
- `ros_gz_bridge`

## Troubleshooting

### Robot doesn't move with teleop_twist_keyboard

Verify that:
- The teleop node is running: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- Check the ROS2 topic connections: `ros2 topic list` and `ros2 topic echo /cmd_vel`
- Verify the Gazebo physics plugin is correctly configured in the URDF
- Check that the `ros_gz_bridge` is properly launching the necessary bridges

## Future Work

- [ ] Debug and fix teleop_twist_keyboard command reception
- [ ] Add custom sensors (IMU, camera, etc.)
- [ ] Implement autonomous navigation nodes
- [ ] Add performance benchmarks

## License

TODO

## References

- [ROS2 Official Documentation](https://docs.ros.org/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [URDF/Xacro Tutorials](https://wiki.ros.org/urdf)
