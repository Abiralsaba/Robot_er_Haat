#!/usr/bin/env python3
"""
IK Visualizer Node
──────────────────
A standalone Matplotlib-based 3D arm visualiser with numerical Inverse
Kinematics, joystick control via ROS 2 /joy topic, and simultaneous
/joint_states publishing so that RViz mirrors the Matplotlib view.

Improvements over the original 5arm.py
───────────────────────────────────────
• Publishes sensor_msgs/JointState so RViz moves in sync.
• Subscribes to the standard /joy topic (not /joy/controller).
• Wrapped in a proper ROS 2 main() entry-point for colcon.
• Graceful shutdown of ROS spinner thread.
• Minor IK robustness tweaks (clamped norm, safer linalg).
"""

import numpy as np
import matplotlib
matplotlib.use("TkAgg")          # explicit backend so plt.show() works
import matplotlib.pyplot as plt
import matplotlib.widgets as mwidgets
from matplotlib.gridspec import GridSpec
import threading, math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header

# ═══════════════════════════════════════════════════════════════════════
#  Physical parameters  (match the URDF pivot-to-pivot distances)
# ═══════════════════════════════════════════════════════════════════════
LINK1_LENGTH       = 0.440       # shoulder → elbow
LINK2_LENGTH       = 0.485       # elbow    → wrist
WRIST_LENGTH       = 0.120
GRIPPER_LENGTH     = 0.130
GRIPPER_SPREAD     = 0.050
WRIST_SPOKE_RADIUS = 0.045

# Joint limits in degrees
LIMITS = [
    (-180, 180),   # θ0 base yaw
    (   0,  90),   # θ1 shoulder pitch (never below base)
    (-135, 135),   # θ2 elbow pitch
    ( -90,  90),   # θ3 wrist pitch
    (-180, 180),   # θ4 wrist roll
]

# URDF joint names (must match arm.urdf & arm_controllers.yaml)
JOINT_NAMES = [
    "j0_base_yaw",
    "j1_shoulder_pitch",
    "j2_elbow_pitch",
    "j3_wrist_pitch",
    "j4_wrist_roll",
    "left_finger_joint",
]

SPEED_DEG_PER_SEC  = 90.0
SLOW_MULTIPLIER    = 0.25
UPDATE_HZ          = 50
INITIAL_ANGLES     = [30.0, 40.0, -30.0, 0.0, 0.0]

# IK tuning
IK_STEP_DEG  = 1.5
IK_TOL       = 0.005
IK_MAX_ITER  = 300
IK_SPEED     = 30.0

# ═══════════════════════════════════════════════════════════════════════
#  Forward Kinematics
# ═══════════════════════════════════════════════════════════════════════
def _rodrigues(v, axis, ang):
    """Rodrigues rotation of vector *v* about *axis* by *ang* radians."""
    c, s = np.cos(ang), np.sin(ang)
    return v * c + np.cross(axis, v) * s + axis * np.dot(axis, v) * (1 - c)


def compute_fk(t1d, t2d, t3d, t4d, t5d):
    t1, t2, t3, t4, t5 = np.radians([t1d, t2d, t3d, t4d, t5d])
    p0 = np.zeros(3)

    r1 = LINK1_LENGTH * np.cos(t2)
    p1 = np.array([r1 * np.cos(t1), r1 * np.sin(t1),
                   LINK1_LENGTH * np.sin(t2)])

    cum = t2 + t3
    r2 = r1 + LINK2_LENGTH * np.cos(cum)
    p2 = np.array([r2 * np.cos(t1), r2 * np.sin(t1),
                   p1[2] + LINK2_LENGTH * np.sin(cum)])

    cp, sp = np.cos(cum), np.sin(cum)
    fwd    = np.array([np.cos(t1) * cp, np.sin(t1) * cp, sp])
    lat    = np.array([-np.sin(t1), np.cos(t1), 0.0])
    up     = np.cross(fwd, lat)
    up    /= (np.linalg.norm(up) + 1e-12)

    fwd_p = _rodrigues(fwd, lat, t4)
    up_p  = _rodrigues(up, lat, t4)
    p3    = p2 + fwd_p * WRIST_LENGTH

    sb = _rodrigues(up_p, fwd_p, t5)
    sp_vec = np.cross(fwd_p, sb)
    sp_vec /= (np.linalg.norm(sp_vec) + 1e-12)

    f1  = p3 + sp_vec * GRIPPER_SPREAD + fwd_p * GRIPPER_LENGTH
    f2  = p3 - sp_vec * GRIPPER_SPREAD + fwd_p * GRIPPER_LENGTH
    tip = p3 + fwd_p * GRIPPER_LENGTH

    spokes = [p3 + _rodrigues(sb, fwd_p, i * np.pi / 2) * WRIST_SPOKE_RADIUS
              for i in range(4)]

    return dict(p0=p0, p1=p1, p2=p2, p3=p3,
                f1=f1, f2=f2, tip=tip, spokes=spokes)


def get_tip(q):
    return compute_fk(*q)["tip"]

# ═══════════════════════════════════════════════════════════════════════
#  Numerical IK  (Damped Least-Squares, multi-seed)
# ═══════════════════════════════════════════════════════════════════════
def solve_ik(target_xyz, seed_angles):
    target = np.asarray(target_xyz, dtype=float)
    max_reach = LINK1_LENGTH + LINK2_LENGTH + WRIST_LENGTH + GRIPPER_LENGTH

    if np.linalg.norm(target) > max_reach * 1.02:
        return None
    if target[2] < -0.02:
        return None

    seeds = [np.array(seed_angles, dtype=float)]
    for _ in range(10):
        seeds.append(np.array([np.random.uniform(*LIMITS[j]) for j in range(5)]))
    ba = np.degrees(np.arctan2(target[1], target[0]))
    for s2, s3 in [(45, -45), (75, -75), (85, -90)]:
        seeds.append(np.array([ba, s2, s3, 0, 0], dtype=float))

    best_q, best_dist = None, float("inf")

    for idx, seed in enumerate(seeds):
        q = seed.copy()
        eps, lam = 0.5, 0.1
        for _ in range(IK_MAX_ITER):
            tip = get_tip(q)
            err = target - tip
            d = np.linalg.norm(err)
            if d < IK_TOL:
                break
            J = np.zeros((3, 4))
            for j in range(4):
                qp = q.copy(); qp[j] += eps
                J[:, j] = (get_tip(qp) - tip) / np.radians(eps)
            JT = J.T
            dq = JT @ np.linalg.solve(J @ JT + lam**2 * np.eye(3), err)
            dq_deg = np.clip(np.degrees(dq), -IK_STEP_DEG, IK_STEP_DEG)
            for j in range(4):
                q[j] = float(np.clip(q[j] + dq_deg[j], *LIMITS[j]))

        pose = compute_fk(*q)
        if min(pose["p1"][2], pose["p2"][2], pose["p3"][2]) < -0.01:
            continue
        fe = np.linalg.norm(target - get_tip(q))
        if fe < 0.03:
            if idx == 0:
                return q.tolist()
            jd = np.linalg.norm(q[:4] - np.array(seed_angles[:4]))
            if jd < best_dist:
                best_dist = jd; best_q = q.copy()

    return best_q.tolist() if best_q is not None else None

# ═══════════════════════════════════════════════════════════════════════
#  Thread-safe Joy state container
# ═══════════════════════════════════════════════════════════════════════
class JoyState:
    def __init__(self):
        self._lock = threading.Lock()
        self.axes = [0.0] * 8
        self.buttons = [0] * 14

    def update(self, axes, buttons):
        with self._lock:
            self.axes = list(axes)
            self.buttons = list(buttons)

    def get(self):
        with self._lock:
            return list(self.axes), list(self.buttons)


joy_state = JoyState()

# ═══════════════════════════════════════════════════════════════════════
#  ROS 2 node — subscribes to /joy, publishes /joint_states
# ═══════════════════════════════════════════════════════════════════════
class IKBridgeNode(Node):
    def __init__(self):
        super().__init__("ik_visualizer")
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)

    def _joy_cb(self, msg):
        joy_state.update(msg.axes, msg.buttons)

    def publish_joints(self, angles_deg):
        """Publish the 5 revolute joints (rad) + gripper (0) to /joint_states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        rads = [math.radians(a) for a in angles_deg]
        for name, pos in zip(JOINT_NAMES[:5], rads):
            msg.name.append(name)
            msg.position.append(pos)
        # gripper fingers
        msg.name.append("left_finger_joint")
        msg.position.append(0.0)
        msg.name.append("right_finger_joint")
        msg.position.append(0.0)
        self.js_pub.publish(msg)


_ros_node = None
_ros_thread = None


def _ros_spin():
    global _ros_node
    rclpy.init()
    _ros_node = IKBridgeNode()
    rclpy.spin(_ros_node)
    _ros_node.destroy_node()
    rclpy.shutdown()

# ═══════════════════════════════════════════════════════════════════════
#  Matplotlib GUI
# ═══════════════════════════════════════════════════════════════════════
def _build_gui():
    global _ros_thread

    # start ROS spinner
    _ros_thread = threading.Thread(target=_ros_spin, daemon=True)
    _ros_thread.start()

    # wait briefly for _ros_node to initialise
    import time; time.sleep(0.5)

    # ── state ────────────────────────────────────────────────
    angles      = list(INITIAL_ANGLES)
    state       = dict(mode="manual", ik_target=None,
                       ik_solution=None, ik_status="Manual mode",
                       prev_reset=False)
    dt = 1.0 / UPDATE_HZ

    # ── figure ───────────────────────────────────────────────
    fig = plt.figure(figsize=(18, 10), facecolor="#0d1117")
    gs = GridSpec(12, 4, figure=fig, hspace=0.3, wspace=0.25,
                  top=0.95, bottom=0.05, left=0.05, right=0.95)

    ax3d = fig.add_subplot(gs[0:10, 0:3], projection="3d")
    ax3d.set_facecolor("#0d1117")
    ax3d.view_init(elev=25, azim=45)
    for pane in (ax3d.xaxis.pane, ax3d.yaxis.pane, ax3d.zaxis.pane):
        pane.fill = False; pane.set_edgecolor("#30363d")
    ax3d.tick_params(colors="#8b949e")

    # ── right panels ─────────────────────────────────────────
    def _panel(rows, title):
        p = fig.add_subplot(gs[rows, 3])
        p.set_facecolor("#161b22"); p.set_xticks([]); p.set_yticks([])
        for s in p.spines.values(): s.set_edgecolor("#30363d")
        p.set_title(f" {title} ", color="#8b949e", fontsize=10,
                    pad=10, fontweight="bold")
        return p

    jp = _panel(slice(0, 4), "JOINT ANGLES")
    joint_text = jp.text(0.05, 0.95, "", transform=jp.transAxes,
                         ha="left", va="top", color="#e6edf3",
                         fontsize=9, fontfamily="monospace")

    ikp = _panel(slice(4, 8), "INVERSE KINEMATICS")
    ik_txt = ikp.text(0.5, 0.85, "Manual mode", transform=ikp.transAxes,
                      ha="center", va="center", color="#f39c12",
                      fontsize=9, fontfamily="monospace",
                      bbox=dict(boxstyle="round,pad=0.5",
                                facecolor="#21262d", edgecolor="#f39c12"))

    cp = _panel(slice(8, 12), "CONTROLS")
    cp.text(0.5, 0.5,
            "🎮 JOYSTICK CONTROLS\n\n"
            "Left Stick X   →  Base L/R\n"
            "Left Stick Y   →  Wrist U/D\n"
            "Right Stick X  →  Shoulder\n"
            "D-Pad U/D      →  Elbow\n"
            "L1 / R1        →  Roll L/R\n"
            "L2 / R2        →  Grip C/O\n\n"
            "⚠️  Stick input cancels IK",
            transform=cp.transAxes, ha="center", va="center",
            color="#8b949e", fontsize=9, fontfamily="monospace",
            linespacing=1.5)

    # ── IK widgets ───────────────────────────────────────────
    pos = ikp.get_position()
    l, b, w, h = pos.x0, pos.y0, pos.width, pos.height
    _tb = dict(color="#21262d", hovercolor="#2d333b")

    tbs = []
    for i, (lbl, val) in enumerate([("X:", "0.30"), ("Y:", "0.20"), ("Z:", "0.40")]):
        a = fig.add_axes([l + w*0.15, b + h*(0.65 - i*0.08), w*0.7, h*0.07])
        tb = mwidgets.TextBox(a, lbl, initial=val, **_tb)
        tb.text_disp.set_color("#e6edf3"); tb.text_disp.set_fontsize(10)
        tb.label.set_color("#8b949e"); tb.label.set_fontsize(9)
        tbs.append(tb)
    tb_x, tb_y, tb_z = tbs

    bw = w * 0.35; bh = h * 0.12; by = b + h * 0.12
    a_go = fig.add_axes([l + w*0.1, by, bw, bh])
    a_cn = fig.add_axes([l + w*0.55, by, bw, bh])
    btn_go = mwidgets.Button(a_go, "▶ GO", color="#0d4f1c", hovercolor="#1a6b28")
    btn_cn = mwidgets.Button(a_cn, "✕ CANCEL", color="#4f1515", hovercolor="#6b1e1e")
    btn_go.label.set_color("#2ecc71"); btn_go.label.set_fontsize(10)
    btn_go.label.set_fontweight("bold")
    btn_cn.label.set_color("#e74c3c"); btn_cn.label.set_fontsize(10)
    btn_cn.label.set_fontweight("bold")

    # ── 3D art ───────────────────────────────────────────────
    def mkl(c, lw=4): return ax3d.plot([], [], [], color=c, linewidth=lw, zorder=3)[0]
    def mks(c, s=80, m="o"):
        return ax3d.scatter([], [], [], color=c, s=s, edgecolors="white",
                            linewidths=0.8, zorder=5, marker=m)
    def sl(ln, a, b): ln.set_data_3d([a[0],b[0]], [a[1],b[1]], [a[2],b[2]])

    l1, l2, l3 = mkl("#0077b6", 5), mkl("#0096c7", 5), mkl("#48cae4", 4)
    g1, g2 = mkl("#2ecc71", 3), mkl("#2ecc71", 3)
    sk = [mkl("#e74c3c", 1.5) for _ in range(4)]
    sc_arm = mks("#f39c12", 120); sc_wr = mks("#ff4d4d", 140)
    tgt_dot = mks("#ff00ff", 200, "*")

    for v, c in [([1,0,0],"red"), ([0,1,0],"green"), ([0,0,1],"blue")]:
        ax3d.plot([0, 0.25*v[0]], [0, 0.25*v[1]], [0, 0.25*v[2]], color=c)

    LIM = LINK1_LENGTH + LINK2_LENGTH + WRIST_LENGTH + GRIPPER_LENGTH + 0.2
    gr = np.linspace(-LIM, LIM, 20); GX, GY = np.meshgrid(gr, gr)
    ax3d.plot_wireframe(GX, GY, np.zeros_like(GX),
                        color="#30363d", linewidths=0.5, alpha=0.3)
    ax3d.set_xlim(-LIM, LIM); ax3d.set_ylim(-LIM, LIM)
    ax3d.set_zlim(0, LIM*1.1); ax3d.set_box_aspect([1, 1, 0.8])
    ax3d.set_xlabel("X (m)", color="#8b949e", fontsize=9)
    ax3d.set_ylabel("Y (m)", color="#8b949e", fontsize=9)
    ax3d.set_zlabel("Z (m)", color="#8b949e", fontsize=9)
    title3d = ax3d.set_title("", color="#e6edf3", fontsize=11, pad=15)

    jlabels = {}
    for n in ["Base", "Shoulder", "Elbow", "Wrist"]:
        jlabels[n] = ax3d.text(0, 0, 0, n, color="#e6edf3", fontsize=8, alpha=0.8)

    # ── render helpers ───────────────────────────────────────
    def draw(r):
        p0, p1, p2, p3 = r["p0"], r["p1"], r["p2"], r["p3"]
        sl(l1, p0, p1); sl(l2, p1, p2); sl(l3, p2, p3)
        sl(g1, p3, r["f1"]); sl(g2, p3, r["f2"])
        for i, s in enumerate(sk): sl(s, p3, r["spokes"][i])
        arm = np.array([p0, p1, p2])
        sc_arm._offsets3d = (arm[:,0], arm[:,1], arm[:,2])
        sc_wr._offsets3d = ([p3[0]], [p3[1]], [p3[2]])
        for n, p in dict(Base=p0, Shoulder=p1, Elbow=p2, Wrist=p3).items():
            jlabels[n].set_position((p[0], p[1]))
            jlabels[n].set_3d_properties(p[2] + 0.04)
        ml = {"manual":"MANUAL","ik":"IK MOVING","hold":"IK HOLD"}.get(state["mode"],"")
        tip = r["tip"]
        title3d.set_text(
            f"Tip: X={tip[0]:+.3f}  Y={tip[1]:+.3f}  Z={tip[2]:+.3f}  |  {ml}")

    # ── callbacks ────────────────────────────────────────────
    def cancel(_=None):
        state.update(mode="manual", ik_target=None,
                     ik_solution=None, ik_status="Manual mode")

    def go(_):
        try:
            tx, ty, tz = float(tb_x.text), float(tb_y.text), float(tb_z.text)
        except ValueError:
            state["ik_status"] = "❌ Invalid input"; return
        state["ik_status"] = "🔍 Solving…"
        ik_txt.set_text(state["ik_status"]); fig.canvas.draw_idle()

        def _solve():
            sol = solve_ik([tx, ty, tz], angles)
            if sol is None:
                state.update(ik_status=f"❌ Unreachable ({tx:.2f},{ty:.2f},{tz:.2f})",
                             ik_target=None, ik_solution=None, mode="manual")
            else:
                state.update(ik_target=[tx, ty, tz], ik_solution=sol,
                             ik_status=f"🚀 Moving to ({tx:.2f},{ty:.2f},{tz:.2f})",
                             mode="ik")
        threading.Thread(target=_solve, daemon=True).start()

    btn_go.on_clicked(go); btn_cn.on_clicked(cancel)

    # ── timer ────────────────────────────────────────────────
    def tick(_=None):
        axes, btns = joy_state.get()

        stick_active = max(abs(axes[0]), abs(axes[1]),
                           abs(axes[3])) > 0.05

        if state["mode"] == "ik" and state["ik_solution"] is not None:
            ms = IK_SPEED * dt; reached = True
            for i in range(5):
                d = state["ik_solution"][i] - angles[i]
                if abs(d) > 0.15:
                    reached = False
                    angles[i] = float(np.clip(
                        angles[i] + np.clip(d, -ms, ms), *LIMITS[i]))
            if reached:
                angles[:] = state["ik_solution"]
                t = state["ik_target"]
                state["ik_status"] = f"✓ Reached ({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})"
                state["mode"] = "hold"
            if stick_active: cancel()

        elif state["mode"] == "manual":
            spd = SPEED_DEG_PER_SEC
            # axis 0 → base yaw,  axis 1 → wrist pitch
            # axis 3 → shoulder,  axis 7 → elbow (D-Pad)
            # L1/R1 (btn4/5) → wrist roll,  L2/R2 (axis2/5) → gripper
            roll_cmd = (1 if btns[5] else 0) - (1 if btns[4] else 0)
            angles[0] = float(np.clip(angles[0] + axes[0]*spd*dt, *LIMITS[0]))
            angles[1] = float(np.clip(angles[1] + axes[3]*spd*dt, *LIMITS[1]))
            angles[2] = float(np.clip(angles[2] + axes[7]*spd*dt, *LIMITS[2]))
            angles[3] = float(np.clip(angles[3] + axes[1]*spd*dt, *LIMITS[3]))
            angles[4] = float(np.clip(angles[4] + roll_cmd*spd*dt, *LIMITS[4]))

        elif state["mode"] == "hold" and stick_active:
            cancel()

        # target dot
        if state["ik_target"]:
            t = state["ik_target"]
            tgt_dot._offsets3d = ([t[0]], [t[1]], [t[2]])
        else:
            tgt_dot._offsets3d = ([], [], [])

        # FK + draw
        r = compute_fk(*angles); draw(r)

        # publish to RViz
        if _ros_node is not None:
            _ros_node.publish_joints(angles)

        # joint readout
        names = ["θ0 Base Yaw", "θ1 Shoulder", "θ2 Elbow",
                 "θ3 Wrist Pitch", "θ4 Wrist Roll"]
        lines = [f"{'─'*30}"]
        for j, n in enumerate(names):
            a = angles[j]; lo, hi = LIMITS[j]
            pct = (a - lo) / (hi - lo); f = int(pct * 20)
            lines.append(f"{n:15} {a:+7.1f}°")
            lines.append(f"               [{'█'*f}{'░'*(20-f)}]")
        tip = r["tip"]
        lines += [f"{'─'*30}",
                  f"Tip X: {tip[0]:+.3f} m",
                  f"Tip Y: {tip[1]:+.3f} m",
                  f"Tip Z: {tip[2]:+.3f} m"]
        joint_text.set_text("\n".join(lines))
        ik_txt.set_text(state["ik_status"])
        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=int(1000 / UPDATE_HZ))
    timer.add_callback(tick); timer.start()
    tick()
    plt.show()


# ═══════════════════════════════════════════════════════════════════════
#  Entry point (called by colcon console_scripts)
# ═══════════════════════════════════════════════════════════════════════
def main(args=None):
    _build_gui()


if __name__ == "__main__":
    main()
