# Rover Arm ROS 2 Package (IK Visualizer & RViz)

This package allows you to control the UMRT 5-DOF robotic arm using a standard joystick/gamepad. It features a custom Matplotlib 3D Inverse Kinematics solver natively integrated directly alongside the RViz visualizer!

## Quick Start
1. **Build the package:**
   ```bash
   colcon build
   ```
2. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```
3. **Connect your Joystick** and launch the new unified IK system:
   ```bash
   ros2 launch rover_arm ik.launch.py
   ```

## Joystick Controls (Standard Gamepad)

- **Left Stick X/Y:**  Base left,right / wrist up,down
- **Right Stick X/Y:** shoulder forward,backward/ nothing  
- **L2 = gripper close / R2 = gripper open**         
- **L1 = gripper rotation left / R1 = gripper rotation right**        



*(You can also use the Inverse Kinematics panel in the new Matplotlib window to enter exact (X,Y,Z) Cartesian coordinates!)*
