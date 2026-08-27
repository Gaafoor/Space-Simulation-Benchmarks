#!/usr/bin/env python3
"""Hill-frame wrench PID controller for Basilisk-ROS 2 bridge scenarios.

Subscribes to Hill translational / rotational guidance states and publishes
body-frame force and torque commands (wrench mode).

Example (single spacecraft):
    ros2 run bsk-ros2-bridge pid-control --ros-args \\
        -r __ns:=/leaderSc \\
        -p setpoint_position:=[1.5,0.0,0.0] \\
        -p use_sim_time:=true
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from bsk_msgs.msg import (
    AttGuidMsgPayload,
    CmdForceBodyMsgPayload,
    CmdTorqueBodyMsgPayload,
    HillRelStateMsgPayload,
)
from geometry_msgs.msg import PoseStamped


def mrp_to_dcm(sigma: np.ndarray) -> np.ndarray:
    """DCM [BR] mapping R-frame vectors into the B frame (Basilisk MRP convention)."""
    s = np.asarray(sigma, dtype=float).reshape(3)
    s2 = float(np.dot(s, s))
    if s2 > 1.0:
        s = -s / s2
        s2 = float(np.dot(s, s))
    skew = np.array(
        [
            [0.0, -s[2], s[1]],
            [s[2], 0.0, -s[0]],
            [-s[1], s[0], 0.0],
        ]
    )
    return np.eye(3) + (8.0 * skew @ skew - 4.0 * (1.0 - s2) * skew) / (1.0 + s2) ** 2


def mrp_to_quat(sigma: np.ndarray) -> np.ndarray:
    """MRP → quaternion [w, x, y, z] (Basilisk convention)."""
    s = np.asarray(sigma, dtype=float).reshape(3)
    s2 = float(np.dot(s, s))
    if s2 > 1.0:
        s = -s / s2
        s2 = float(np.dot(s, s))
    denom = 1.0 + s2
    q = np.array([(1.0 - s2) / denom, 2.0 * s[0] / denom, 2.0 * s[1] / denom, 2.0 * s[2] / denom])
    return q / np.linalg.norm(q)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product, [w,x,y,z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_error_mrp(q_cur: np.ndarray, q_des: np.ndarray) -> np.ndarray:
    """Attitude error as MRP: body relative to desired (Hill-frame quats)."""
    if np.dot(q_cur, q_des) < 0.0:
        q_des = -q_des
    q_err = quat_multiply(quat_conjugate(q_des), q_cur)
    q_err = q_err / (np.linalg.norm(q_err) + 1e-12)
    w = q_err[0]
    v = q_err[1:4]
    if abs(1.0 + w) < 1e-8:
        return np.zeros(3)
    return v / (1.0 + w)


def saturate(vec: np.ndarray, limit: float) -> np.ndarray:
    if limit <= 0.0:
        return vec
    norm = float(np.linalg.norm(vec))
    if norm > limit:
        return vec * (limit / norm)
    return vec


class PidControlNode(Node):
    """PID station-keeping controller in the Hill frame with body-frame wrench output."""

    def __init__(self) -> None:
        super().__init__("pid_control")
        self._declare_parameters()
        self._load_parameters()

        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.sigma_br = np.zeros(3)
        self.omega_br_b = np.zeros(3)
        self.setpoint_attitude = np.array([1.0, 0.0, 0.0, 0.0])  # [w,x,y,z] Hill frame
        self.have_trans = False
        self.have_rot = False

        self.integral_pos = np.zeros(3)
        self.integral_att = np.zeros(3)
        self.prev_time = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            HillRelStateMsgPayload,
            "bsk/out/hill_trans_state",
            self._hill_trans_callback,
            qos,
        )
        self.create_subscription(
            AttGuidMsgPayload,
            "bsk/out/hill_rot_state",
            self._hill_rot_callback,
            qos,
        )
        self.create_subscription(
            PoseStamped,
            "pid/setpoint_pose",
            self._setpoint_callback,
            10,
        )

        self.force_pub = self.create_publisher(
            CmdForceBodyMsgPayload, "bsk/in/cmd_force", qos
        )
        self.torque_pub = self.create_publisher(
            CmdTorqueBodyMsgPayload, "bsk/in/cmd_torque", qos
        )

        self.create_timer(1.0 / self.control_rate, self._control_loop)

        self.get_logger().info(
            f"PID control ready | setpoint={self.setpoint_position.tolist()} "
            f"| rate={self.control_rate:.1f} Hz"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("setpoint_position", [1.5, 0.0, 0.0])
        self.declare_parameter("setpoint_velocity", [0.0, 0.0, 0.0])
        # Translational PID (Hill frame) -> body force
        # Tuned for m≈17.8 kg: higher Kd / lower Ki to reduce overshoot oscillation.
        self.declare_parameter("kp_pos", [0.9, 0.9, 0.9])
        self.declare_parameter("ki_pos", [0.0, 0.0, 0.0])
        self.declare_parameter("kd_pos", [9.0, 9.0, 9.0])
        # Attitude MRP PD (+ optional I) -> body torque
        self.declare_parameter("kp_att", [0.15, 0.15, 0.15])
        self.declare_parameter("ki_att", [0.0, 0.0, 0.0])
        self.declare_parameter("kd_att", [0.5, 0.5, 0.5])
        self.declare_parameter("force_limit", 3.0)
        self.declare_parameter("torque_limit", 0.4)
        self.declare_parameter("integral_limit_pos", 0.5)
        self.declare_parameter("integral_limit_att", 0.1)
        self.declare_parameter("enable_attitude_control", True)

    def _load_parameters(self) -> None:
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.setpoint_position = np.array(
            self.get_parameter("setpoint_position").value, dtype=float
        )
        self.setpoint_velocity = np.array(
            self.get_parameter("setpoint_velocity").value, dtype=float
        )
        self.kp_pos = np.array(self.get_parameter("kp_pos").value, dtype=float)
        self.ki_pos = np.array(self.get_parameter("ki_pos").value, dtype=float)
        self.kd_pos = np.array(self.get_parameter("kd_pos").value, dtype=float)
        self.kp_att = np.array(self.get_parameter("kp_att").value, dtype=float)
        self.ki_att = np.array(self.get_parameter("ki_att").value, dtype=float)
        self.kd_att = np.array(self.get_parameter("kd_att").value, dtype=float)
        self.force_limit = float(self.get_parameter("force_limit").value)
        self.torque_limit = float(self.get_parameter("torque_limit").value)
        self.integral_limit_pos = float(self.get_parameter("integral_limit_pos").value)
        self.integral_limit_att = float(self.get_parameter("integral_limit_att").value)
        self.enable_attitude_control = bool(
            self.get_parameter("enable_attitude_control").value
        )

    def _hill_trans_callback(self, msg: HillRelStateMsgPayload) -> None:
        self.position = np.array(msg.r_dc_h, dtype=float)
        self.velocity = np.array(msg.v_dc_h, dtype=float)
        self.have_trans = True

    def _hill_rot_callback(self, msg: AttGuidMsgPayload) -> None:
        self.sigma_br = np.array(msg.sigma_br, dtype=float)
        self.omega_br_b = np.array(msg.omega_br_b, dtype=float)
        self.have_rot = True

    def _setpoint_callback(self, msg: PoseStamped) -> None:
        self.setpoint_position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )
        q = np.array(
            [
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ],
            dtype=float,
        )
        n = float(np.linalg.norm(q))
        if n > 1e-9:
            q = q / n
            if np.dot(self.setpoint_attitude, q) < 0.0:
                q = -q
            self.setpoint_attitude = q

    def _control_loop(self) -> None:
        if not (self.have_trans and self.have_rot):
            return

        now = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = now
            return
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        pos_error = self.setpoint_position - self.position
        vel_error = self.setpoint_velocity - self.velocity
        self.integral_pos = saturate(
            self.integral_pos + pos_error * dt, self.integral_limit_pos
        )

        force_hill = (
            self.kp_pos * pos_error
            + self.ki_pos * self.integral_pos
            + self.kd_pos * vel_error
        )
        dcm_br = mrp_to_dcm(self.sigma_br)
        force_body = saturate(dcm_br @ force_hill, self.force_limit)

        if self.enable_attitude_control:
            q_cur = mrp_to_quat(self.sigma_br)
            sigma_err = quat_error_mrp(q_cur, self.setpoint_attitude)
            if float(np.dot(sigma_err, sigma_err)) > 1.0:
                sigma_err = -sigma_err / np.dot(sigma_err, sigma_err)
            self.integral_att = saturate(
                self.integral_att + sigma_err * dt, self.integral_limit_att
            )
            torque_body = saturate(
                -(
                    self.kp_att * sigma_err
                    + self.ki_att * self.integral_att
                    + self.kd_att * self.omega_br_b
                ),
                self.torque_limit,
            )
        else:
            torque_body = np.zeros(3)

        stamp = now.to_msg()
        force_msg = CmdForceBodyMsgPayload()
        force_msg.stamp = stamp
        force_msg.forcerequestbody = force_body.tolist()
        self.force_pub.publish(force_msg)

        torque_msg = CmdTorqueBodyMsgPayload()
        torque_msg.stamp = stamp
        torque_msg.torquerequestbody = torque_body.tolist()
        self.torque_pub.publish(torque_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PidControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
