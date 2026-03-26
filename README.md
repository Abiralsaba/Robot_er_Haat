# Rover Arm ROS 2 Package

A complete, simulation-ready ROS 2 package for controlling a 5-DOF robotic arm (plus gripper) using a standard joystick/gamepad. The package includes a physically accurate URDF, `ros2_control` configurations, RViz visualization, and Gazebo simulation support.

## Quick Start (Build & Test)

1. **Build the package:**
   ```bash
   cd "/Users/mdabiralsaba/Documents/March Rover"
   colcon build --packages-select rover_arm --base-paths arm
   ```
2. **Launch RViz & Joystick control:**
   ```bash
   source install/setup.zsh
   ros2 launch rover_arm arm.launch.py
   ```
   *(Ensure your joystick is connected before launching!)*

## Prerequisites

Ensure you have a working ROS 2 environment (ROS 2 Jazzy) and the necessary dependencies installed:

```bash
sudo apt update
sudo apt install ros-<distro>-ros2-control \
                 ros-<distro>-ros2-controllers \
                 ros-<distro>-gazebo-ros2-control \
                 ros-<distro>-joy \
                 ros-<distro>-xacro \
                 ros-<distro>-robot-state-publisher \
                 ros-<distro>-rviz2
```
*(Replace `<distro>` with `jazzy`)*

## Building the Package

1. Navigate to your colcon workspace (or the directory containing the `arm` folder):
   ```bash
   cd ~/ros2_ws  # or /Users/mdabiralsaba/Documents/March Rover
   ```
2. Build the `rover_arm` package:
   ```bash
   colcon build --packages-select rover_arm
   ```
   *Note: If building directly from the `March Rover` folder without a standard `src/` layout, you may need to specify the base path:*
   ```bash
   colcon build --packages-select rover_arm --base-paths arm
   ```
3. Source the overlay workspace:
   ```bash
   source install/setup.bash # or setup.zsh
   ```

## Running the System

### Option 1: RViz Visualization & Joystick Control (No Physics)
Use this option to verify the URDF kinematics and joystick mapping quickly without the overhead of Gazebo.

1. Connect your joystick/gamepad.
2. Launch the system:
   ```bash
   ros2 launch rover_arm arm.launch.py
   ```
   This command starts:
   - The Joy node reading from `/dev/input/js0`
   - The custom Python node (`arm_joy_controller`) mapping joystick inputs to joint commands
   - `robot_state_publisher` broadcasting TF frames
   - RViz loaded with a pre-configured layout to visualize the arm

### Option 2: Gazebo Simulation with ros2_control
Use this option to run the fully physics-simulated arm inside Gazebo, utilizing `ros2_control` and `JointGroupPositionController`.

1. Launch Gazebo and spawn the arm:
   ```bash
   ros2 launch rover_arm gazebo.launch.py
   ```
   This command:
   - Starts an empty Gazebo world
   - Spawns the `rover_arm` entity
   - Automatically loads and activates the `joint_state_broadcaster` and `arm_controller`

*Note: To control the simulated arm in Gazebo using the joystick, you can run the joy controller node in a separate terminal (ensure it publishes to the controller's command topic, or adapt `arm.launch.py` to use `use_sim_time:=true`).*

## Controls Mapping (Standard Gamepad)

The custom Python controller (`arm_joy_controller.py`) maps joystick axes and buttons to the corresponding joints, respecting the physical limits defined in the URDF:

| Input | Joint | Type | Limit (rad or m) |
|-------|-------|------|------------------|
| **Left Stick L/R** | Base Yaw (`joint_1_base_yaw`) | Revolute | `[-3.14, 3.14]` |
| **Left Stick U/D** | Shoulder Pitch (`joint_2_shoulder_pitch`) | Revolute | `[-0.35, 2.60]` |
| **Right Stick U/D** | Elbow Pitch (`joint_3_elbow_pitch`) | Revolute | `[-2.35, 2.35]` |
| **Right Stick L/R** | Wrist Roll (`joint_4_wrist_roll`) | Revolute | `[-3.14, 3.14]` |
| **D-Pad U/D** | Wrist Pitch (`joint_5_wrist_pitch`) | Revolute | `[-1.57, 1.57]` |
| **Button B (Open)** | Gripper (Fingers) | Prismatic | `[-0.04, 0.01]` |
| **Button A (Close)**| Gripper (Fingers) | Prismatic | `[-0.04, 0.01]` |

## Node Information

### `arm_joy_controller`
- **Subscribes to:** `/joy` (`sensor_msgs/msg/Joy`)
- **Publishes to:** `/joint_states` (`sensor_msgs/msg/JointState`)
- **Parameters:**
  - `publish_rate` (double, default: 50.0): The rate at which joint states are published.
  - `joint_speed` (double, default: 1.0): Rad/s base multiplier for revolute joint velocity.
  - `gripper_speed` (double, default: 0.015): m/s speed for gripper prismatic joints.
