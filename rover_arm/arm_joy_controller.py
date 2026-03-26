#!/usr/bin/env python3
"""
Arm Joy Controller Node
-----------------------
Subscribes to /joy (sensor_msgs/Joy) from a joystick and publishes
/joint_states (sensor_msgs/JointState) to drive the rover arm.

Mapping (standard Xbox / PS4-style gamepad):
  Left Stick  L/R  (axis 0)  →  joint_1_base_yaw
  Left Stick  U/D  (axis 1)  →  joint_2_shoulder_pitch
  Right Stick U/D  (axis 4)  →  joint_3_elbow_pitch
  Right Stick L/R  (axis 3)  →  joint_4_wrist_roll
  D-Pad       U/D  (axis 7)  →  joint_5_wrist_pitch
  Button A / B     (0 / 1)   →  Gripper close / open
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header
import math


class ArmJoyController(Node):
    def __init__(self):
        super().__init__('arm_joy_controller')

        # ── Parameters (overridable via launch) ──
        self.declare_parameter('publish_rate', 50.0)    # Hz
        self.declare_parameter('joint_speed', 1.0)      # rad/s base multiplier
        self.declare_parameter('gripper_speed', 0.015)   # m/s

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.joint_speed = self.get_parameter('joint_speed').get_parameter_value().double_value
        self.gripper_speed = self.get_parameter('gripper_speed').get_parameter_value().double_value

        # ── ROS interfaces ──
        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.timer_period = 1.0 / rate
        self.timer = self.create_timer(self.timer_period, self._timer_cb)

        # ── Joint state ──
        self.latest_joy = None

        self.positions = {
            'joint_1_base_yaw': 0.0,
            'joint_2_shoulder_pitch': 0.0,
            'joint_3_elbow_pitch': 0.0,
            'joint_4_wrist_roll': 0.0,
            'joint_5_wrist_pitch': 0.0,
            'left_finger_joint': 0.0,
        }

        # Joint limits (must match the URDF exactly)
        self.limits = {
            'joint_1_base_yaw':       (-3.14159,  3.14159),
            'joint_2_shoulder_pitch':  (-0.35,     2.60),
            'joint_3_elbow_pitch':     (-2.35,     2.35),
            'joint_4_wrist_roll':      (-3.14159,  3.14159),
            'joint_5_wrist_pitch':     (-1.5708,   1.5708),
            'left_finger_joint':       (-0.04,     0.01),
        }

        # Per-joint velocity multipliers (rad/s or m/s for gripper)
        self.velocities = {
            'joint_1_base_yaw':       1.0,
            'joint_2_shoulder_pitch':  0.8,
            'joint_3_elbow_pitch':     1.0,
            'joint_4_wrist_roll':      1.5,
            'joint_5_wrist_pitch':     1.0,
            'left_finger_joint':       1.0,   # scaled by gripper_speed
        }

        self.get_logger().info(
            f'Arm Joy Controller started — publishing at {rate} Hz'
        )

    # ── Callbacks ──
    def _joy_cb(self, msg: Joy):
        self.latest_joy = msg

    def _timer_cb(self):
        if self.latest_joy is None:
            self._publish()
            return

        joy = self.latest_joy
        dt = self.timer_period
        spd = self.joint_speed

        def axis(idx):
            return joy.axes[idx] if idx < len(joy.axes) else 0.0

        def button(idx):
            return joy.buttons[idx] if idx < len(joy.buttons) else 0

        # Base Yaw — Left Stick L/R
        self.positions['joint_1_base_yaw'] += (
            axis(0) * self.velocities['joint_1_base_yaw'] * spd * dt
        )
        # Shoulder Pitch — Left Stick U/D
        self.positions['joint_2_shoulder_pitch'] += (
            axis(1) * self.velocities['joint_2_shoulder_pitch'] * spd * dt
        )
        # Elbow Pitch — Right Stick U/D
        self.positions['joint_3_elbow_pitch'] += (
            axis(4) * self.velocities['joint_3_elbow_pitch'] * spd * dt
        )
        # Wrist Roll — Right Stick L/R
        self.positions['joint_4_wrist_roll'] += (
            axis(3) * self.velocities['joint_4_wrist_roll'] * spd * dt
        )
        # Wrist Pitch — D-pad U/D
        self.positions['joint_5_wrist_pitch'] += (
            axis(7) * self.velocities['joint_5_wrist_pitch'] * spd * dt
        )
        # Gripper — Button B (open) / Button A (close)
        gripper_cmd = button(1) - button(0)
        self.positions['left_finger_joint'] += (
            gripper_cmd * self.gripper_speed * dt
        )

        # Clamp to URDF limits
        for jname in self.positions:
            lo, hi = self.limits[jname]
            self.positions[jname] = max(lo, min(hi, self.positions[jname]))

        self._publish()

    def _publish(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        for name, pos in self.positions.items():
            msg.name.append(name)
            msg.position.append(pos)

        # Mimic joint: right_finger = -1 × left_finger (from URDF mimic tag)
        msg.name.append('right_finger_joint')
        msg.position.append(-self.positions['left_finger_joint'])

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmJoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
