#!/usr/bin/env python3
"""Identical Hill-frame goal sequence for PID and MPC bag comparison runs.

Publishes the same absolute pose setpoints (position + orientation) to:
  /<ns>/pid/setpoint_pose          (PID node)
  /<ns>/bsk_mpc/setpoint_pose      (MPC when use_rviz:=false)
and calls /<ns>/set_pose when available (MPC when use_rviz:=true).

Orientation is quaternion [w, x, y, z] in the Hill / map frame.

Example:
  ros2 run bsk-ros2-bridge formation-goal-sequence --ros-args -p use_sim_time:=true
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

try:
    from bsk_mpc_msgs.srv import SetPose
    _HAS_SET_POSE = True
except ImportError:
    SetPose = None
    _HAS_SET_POSE = False


def quat_from_yaw_deg(yaw_deg: float):
    """Yaw about Hill +Z → quaternion [w, x, y, z]."""
    half = math.radians(yaw_deg) * 0.5
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def pose(xyz, yaw_deg=0.0):
    return {"xyz": list(xyz), "quat_wxyz": quat_from_yaw_deg(yaw_deg)}


# Hold each waypoint for `hold_duration` seconds (sim time if use_sim_time).
GOAL_SEQUENCE = [
    # 0: initial formation, Hill-aligned
    {
        "leaderSc": pose([1.5, 0.0, 0.0], 0.0),
        "followerSc_1": pose([0.5, -0.3, 0.0], 0.0),
        "followerSc_2": pose([0.5, 0.3, 0.0], 0.0),
    },
    # 1: translate +0.5 m along-track, yaw +45 deg
    {
        "leaderSc": pose([2.0, 0.0, 0.0], 45.0),
        "followerSc_1": pose([1.0, -0.3, 0.0], 45.0),
        "followerSc_2": pose([1.0, 0.3, 0.0], 45.0),
    },
    # 2: lateral move, yaw +90 deg
    {
        "leaderSc": pose([2.0, 0.4, 0.0], 90.0),
        "followerSc_1": pose([1.0, 0.1, 0.0], 90.0),
        "followerSc_2": pose([1.0, 0.7, 0.0], 90.0),
    },
    # 3: return near IC, yaw back to 0
    {
        "leaderSc": pose([1.5, 0.0, 0.0], 0.0),
        "followerSc_1": pose([0.5, -0.3, 0.0], 0.0),
        "followerSc_2": pose([0.5, 0.3, 0.0], 0.0),
    },
]

NAMESPACES = ("leaderSc", "followerSc_1", "followerSc_2")


def _pose_msg(stamp, goal) -> PoseStamped:
    xyz = goal["xyz"]
    q = goal["quat_wxyz"]  # [w, x, y, z]
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = "map"
    msg.pose.position.x = float(xyz[0])
    msg.pose.position.y = float(xyz[1])
    msg.pose.position.z = float(xyz[2])
    msg.pose.orientation.w = float(q[0])
    msg.pose.orientation.x = float(q[1])
    msg.pose.orientation.y = float(q[2])
    msg.pose.orientation.z = float(q[3])
    return msg


class FormationGoalSequence(Node):
    def __init__(self) -> None:
        super().__init__("formation_goal_sequence")
        self.declare_parameter("hold_duration", 30.0)
        self.declare_parameter("publish_rate", 5.0)
        self.declare_parameter("start_delay", 5.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("call_set_pose_service", True)

        self.hold_duration = float(self.get_parameter("hold_duration").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.start_delay = float(self.get_parameter("start_delay").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.call_set_pose = bool(self.get_parameter("call_set_pose_service").value)

        self.use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        if self.use_sim_time:
            self.get_logger().info("use_sim_time=True — waiting for /clock")
            self._wait_for_clock()

        self.pid_pubs = {}
        self.mpc_pubs = {}
        self.set_pose_clients = {}
        for ns in NAMESPACES:
            self.pid_pubs[ns] = self.create_publisher(
                PoseStamped, f"/{ns}/pid/setpoint_pose", 10
            )
            self.mpc_pubs[ns] = self.create_publisher(
                PoseStamped, f"/{ns}/bsk_mpc/setpoint_pose", 10
            )
            if _HAS_SET_POSE and self.call_set_pose:
                self.set_pose_clients[ns] = self.create_client(
                    SetPose, f"/{ns}/set_pose"
                )

        self.wp_index = 0
        self.phase_t0 = None
        self.started = False
        self.t_node_start = self.get_clock().now()
        self._last_service_wp = -1

        period = 1.0 / max(self.publish_rate, 0.1)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Goal sequence ready | {len(GOAL_SEQUENCE)} waypoints | "
            f"hold={self.hold_duration}s | start_delay={self.start_delay}s | loop={self.loop}"
        )
        for i, wp in enumerate(GOAL_SEQUENCE):
            g = wp["leaderSc"]
            self.get_logger().info(
                f"  wp[{i}] L_xyz={g['xyz']} L_yaw≈{2*math.degrees(math.atan2(g['quat_wxyz'][3], g['quat_wxyz'][0])):g} deg"
            )

    def _wait_for_clock(self) -> None:
        rate = self.create_rate(10)
        while rclpy.ok():
            if "/clock" in dict(self.get_topic_names_and_types()):
                return
            rate.sleep()

    def _tick(self) -> None:
        now = self.get_clock().now()
        if not self.started:
            if (now - self.t_node_start).nanoseconds * 1e-9 < self.start_delay:
                return
            self.started = True
            self.phase_t0 = now
            self.get_logger().info("Sequence started")

        elapsed = (now - self.phase_t0).nanoseconds * 1e-9
        if elapsed >= self.hold_duration:
            self.wp_index += 1
            if self.wp_index >= len(GOAL_SEQUENCE):
                if self.loop:
                    self.wp_index = 0
                    self.get_logger().info("Sequence loop → wp[0]")
                else:
                    self.wp_index = len(GOAL_SEQUENCE) - 1
                    if not hasattr(self, "_done_logged"):
                        self.get_logger().info(
                            "Sequence complete — holding final waypoint"
                        )
                        self._done_logged = True
            else:
                self.get_logger().info(f"Advance → wp[{self.wp_index}]")
            self.phase_t0 = now
            self._last_service_wp = -1

        wp = GOAL_SEQUENCE[self.wp_index]
        stamp = now.to_msg()
        for ns in NAMESPACES:
            msg = _pose_msg(stamp, wp[ns])
            self.pid_pubs[ns].publish(msg)
            self.mpc_pubs[ns].publish(msg)

        if (
            _HAS_SET_POSE
            and self.call_set_pose
            and self._last_service_wp != self.wp_index
        ):
            self._last_service_wp = self.wp_index
            for ns in NAMESPACES:
                client = self.set_pose_clients.get(ns)
                if client is None or not client.service_is_ready():
                    continue
                req = SetPose.Request()
                req.pose = _pose_msg(stamp, wp[ns]).pose
                client.call_async(req)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FormationGoalSequence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
