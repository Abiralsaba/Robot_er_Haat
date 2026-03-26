#!/usr/bin/env python3
"""
Arm IK Joy Controller  —  UMRT 5-DOF Arm  (Jacobian-based)
───────────────────────────────────────────────────────────
Uses the 2×2 Jacobian of the shoulder–elbow planar chain to
smoothly move the wrist in Cartesian space.  This is the standard
approach for joystick teleoperation — no configuration jumps.

URDF Y-axis convention:
  positive angle → arm tilts DOWN (−Z)
  negative angle → arm tilts UP   (+Z)
  FK:  r =  L1·cos(sh) + L2·cos(sh+el)
       z = −L1·sin(sh) − L2·sin(sh+el)

Gamepad:
  Left  Stick X  (axis 0) → Base yaw L/R
  Left  Stick Y  (axis 1) → Wrist pitch U/D
  Right Stick Y  (axis 4) → Tip forward/backward  (Jacobian IK)
  D-Pad Y        (axis 7) → Tip up/down           (Jacobian IK)
  L1 (btn 4)              → Wrist roll LEFT
  R1 (btn 5)              → Wrist roll RIGHT
  L2 (axis 2)             → Gripper CLOSE
  R2 (axis 5)             → Gripper OPEN
"""

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

# ═══════════════════════════════════════════════════════════════
#  Physical parameters
# ═══════════════════════════════════════════════════════════════
L1 = 0.440       # shoulder → elbow
L2 = 0.485       # elbow    → wrist

# URDF joint limits (rad)
LIM_BASE = (-3.14159,  3.14159)
LIM_SH   = (-1.57,     2.60)
LIM_EL   = (-2.35,     2.35)
LIM_WP   = (-1.5708,   1.5708)
LIM_WR   = (-3.14159,  3.14159)
LIM_GRIP = (-0.04,     0.01)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ═══════════════════════════════════════════════════════════════
#  FK  (URDF angles → wrist Cartesian)
# ═══════════════════════════════════════════════════════════════
def fk(sh, el):
    """Return (r, z):  r = horizontal reach, z = height above shoulder."""
    c = sh + el
    r =  L1 * math.cos(sh) + L2 * math.cos(c)
    z = -(L1 * math.sin(sh) + L2 * math.sin(c))
    return r, z


# ═══════════════════════════════════════════════════════════════
#  Jacobian-based IK step
#
#  Given desired Δr and Δz, compute Δsh and Δel:
#
#  J = [ ∂r/∂sh  ∂r/∂el ]   [Δsh]   [Δr]
#      [ ∂z/∂sh  ∂z/∂el ] · [Δel] = [Δz]
#
#  Solve Δq = J⁻¹ · Δx  (2×2 matrix inverse)
# ═══════════════════════════════════════════════════════════════
def ik_step(sh, el, dr, dz):
    """Apply one incremental IK step with damped least-squares.
    Returns (new_sh, new_el) clamped to limits.
    Never produces large jumps; rejects moves that flip the arm."""
    c = sh + el
    sin_sh = math.sin(sh);  cos_sh = math.cos(sh)
    sin_c  = math.sin(c);   cos_c  = math.cos(c)

    # Jacobian  J = [[a, b], [cc, d]]
    a  = -L1 * sin_sh - L2 * sin_c      # ∂r/∂sh
    b  = -L2 * sin_c                     # ∂r/∂el
    cc = -L1 * cos_sh - L2 * cos_c      # ∂z/∂sh
    d  = -L2 * cos_c                     # ∂z/∂el

    det = a * d - b * cc

    # ── Damped Least-Squares ──
    # Instead of Δq = J⁻¹·Δx, use Δq = Jᵀ(JJᵀ + λ²I)⁻¹·Δx
    # For a 2×2 matrix this simplifies to scaling by det/(det²+λ²)
    LAMBDA = 0.05
    det_dls = det / (det * det + LAMBDA * LAMBDA)

    d_sh = ( d * dr - b  * dz) * det_dls
    d_el = (-cc * dr + a * dz) * det_dls

    # ── Step-size limit (max 0.05 rad per tick = ~3°) ──
    MAX_STEP = 0.05
    if abs(d_sh) > MAX_STEP:
        scale = MAX_STEP / abs(d_sh)
        d_sh *= scale
        d_el *= scale
    if abs(d_el) > MAX_STEP:
        scale = MAX_STEP / abs(d_el)
        d_sh *= scale
        d_el *= scale

    new_sh = clamp(sh + d_sh, *LIM_SH)
    new_el = clamp(el + d_el, *LIM_EL)

    # ── Elbow-up guard ──
    # Check that the elbow remains the highest point.
    # Elbow height = −L1·sin(new_sh)
    # Wrist height = −L1·sin(new_sh) − L2·sin(new_sh + new_el)
    # If wrist goes ABOVE elbow, the arm has flipped → reject.
    elbow_z = -L1 * math.sin(new_sh)
    wrist_z = elbow_z - L2 * math.sin(new_sh + new_el)
    if wrist_z > elbow_z + 0.02:   # wrist above elbow = flipped
        return sh, el

    return new_sh, new_el


# ═══════════════════════════════════════════════════════════════
#  Initial pose:  arm pointing UP with elbow-up
#    sh = −0.2  (slightly above horizontal)
#    el = −0.6  (forearm angles further up)
# ═══════════════════════════════════════════════════════════════
INIT_SH = -0.8     # ~46° above horizontal → upper arm goes UP steeply
INIT_EL =  1.0     # forearm bends BACK DOWN from elbow peak (elbow = highest point)


# ═══════════════════════════════════════════════════════════════
#  ROS 2 Node
# ═══════════════════════════════════════════════════════════════
class ArmIKJoyController(Node):
    def __init__(self):
        super().__init__('arm_joy_controller')

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('joint_speed', 1.0)
        self.declare_parameter('gripper_speed', 0.015)
        self.declare_parameter('cart_speed', 0.40)

        rate = self.get_parameter('publish_rate').value
        self.joint_speed   = self.get_parameter('joint_speed').value
        self.gripper_speed = self.get_parameter('gripper_speed').value
        self.cart_speed    = self.get_parameter('cart_speed').value

        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.js_pub  = self.create_publisher(JointState, '/joint_states', 10)
        self.dt      = 1.0 / rate
        self.timer   = self.create_timer(self.dt, self._tick)
        self.joy     = None

        # URDF joint positions (rad)
        self.base_yaw    = 0.0
        self.sh          = INIT_SH
        self.el          = INIT_EL
        self.wrist_pitch = 0.0
        self.wrist_roll  = 0.0
        self.gripper     = 0.0

        r0, z0 = fk(self.sh, self.el)
        self.get_logger().info(
            f'IK Joy Controller — {rate} Hz  |  '
            f'Wrist r={r0:.3f}m  z={z0:.3f}m  '
            f'sh={math.degrees(self.sh):.1f}°  '
            f'el={math.degrees(self.el):.1f}°')

    def _joy_cb(self, msg):
        self.joy = msg

    def _tick(self):
        if self.joy is not None:
            self._update()
        self._publish()

    def _update(self):
        j  = self.joy
        dt = self.dt

        def ax(i):
            return j.axes[i] if i < len(j.axes) else 0.0
        def bt(i):
            return j.buttons[i] if i < len(j.buttons) else 0

        spd = self.joint_speed
        cspd = self.cart_speed

        # ── 1. BASE YAW — Left Stick X ──
        self.base_yaw = clamp(
            self.base_yaw + ax(0) * spd * dt, *LIM_BASE)

        # ── 2. TIP FORWARD / BACKWARD — Right Stick Y ──
        fwd = ax(4)
        if abs(fwd) > 0.05:
            dr = fwd * cspd * dt
            self.sh, self.el = ik_step(self.sh, self.el, dr, 0.0)

        # ── 3. TIP UP / DOWN — D-Pad Y ──
        ud = ax(7)
        if abs(ud) > 0.05:
            dz = ud * cspd * dt
            self.sh, self.el = ik_step(self.sh, self.el, 0.0, dz)

        # ── 4. WRIST PITCH — Left Stick Y ──
        self.wrist_pitch = clamp(
            self.wrist_pitch + ax(1) * spd * dt, *LIM_WP)

        # ── 5. WRIST ROLL — L1/R1 ──
        self.wrist_roll = clamp(
            self.wrist_roll + (bt(5) - bt(4)) * 1.5 * spd * dt,
            *LIM_WR)

        # ── 6. GRIPPER — L2 close / R2 open ──
        l2 = (1.0 - ax(2)) / 2.0
        r2 = (1.0 - ax(5)) / 2.0
        self.gripper = clamp(
            self.gripper + (r2 - l2) * self.gripper_speed * dt,
            *LIM_GRIP)

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'j0_base_yaw', 'j1_shoulder_pitch',
            'j2_elbow_pitch', 'j3_wrist_pitch',
            'j4_wrist_roll', 'left_finger_joint',
            'right_finger_joint']
        msg.position = [
            self.base_yaw, self.sh, self.el,
            self.wrist_pitch, self.wrist_roll,
            self.gripper, -self.gripper]
        self.js_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmIKJoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

if __name__ == '__main__':
    main()
