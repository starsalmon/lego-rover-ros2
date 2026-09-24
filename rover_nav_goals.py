#!/usr/bin/env python3
"""While Go is held (session on), ask Nav2 for the next metre ahead.

Nav2 plans around the map. This node only picks a point and cancels it when
the session stops. It does not command the wheels itself.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
import tf2_ros


class RoverNavGoals(Node):
    def __init__(self) -> None:
        super().__init__('rover_nav_goals')
        self._session = False
        self._goal_handle = None
        self._busy = False
        self._heading_idx = 0
        self._fail_streak = 0
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(Bool, '/rover/session', self._on_session, 10)
        self.create_timer(2.0, self._tick)
        self.get_logger().info('nav goals waiting for Go')

    def _on_session(self, msg: Bool) -> None:
        active = bool(msg.data)
        if active == self._session:
            return
        self._session = active
        if not active:
            self._heading_idx = 0
            self._fail_streak = 0
            self._cancel()
            self.get_logger().info('session off — cancel nav goal')
        else:
            self.get_logger().info('session on — Nav2 will take the next goal')

    def _cancel(self) -> None:
        handle = self._goal_handle
        self._goal_handle = None
        self._busy = False
        if handle is not None:
            handle.cancel_goal_async()

    def _tick(self) -> None:
        if not self._session or self._busy:
            return
        if not self._client.server_is_ready():
            return
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        base_yaw = _yaw(tf.transform.rotation.z, tf.transform.rotation.w)
        yaw = base_yaw + self._heading_idx * (math.pi / 3.0)
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.transform.translation.x + 1.2 * math.cos(yaw)
        pose.pose.position.y = tf.transform.translation.y + 1.2 * math.sin(yaw)
        half = 0.5 * yaw
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        goal.pose = pose
        self._busy = True
        self.get_logger().info(
            f'goal {pose.pose.position.x:.2f},{pose.pose.position.y:.2f} heading {math.degrees(yaw):.0f}°'
        )
        send = self._client.send_goal_async(goal)
        send.add_done_callback(self._accepted)

    def _accepted(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self._busy = False
            self._fail_streak += 1
            self._heading_idx = (self._heading_idx + 1) % 6
            self.get_logger().info(
                f'goal rejected — try heading slot {self._heading_idx}/6'
            )
            return
        self._goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(self._finished)

    def _finished(self, future) -> None:
        self._busy = False
        self._goal_handle = None
        if not self._session:
            return
        status = future.result().status
        # 4 = SUCCEEDED. Anything else, swing the next target so it doesn't
        # keep asking for the same blocked point.
        if status != 4:
            self._fail_streak += 1
            self._heading_idx = (self._heading_idx + 1) % 6
            self.get_logger().info(
                f'goal ended status={status} — heading slot {self._heading_idx}/6'
            )
        else:
            self._fail_streak = 0
            self._heading_idx = 0
            self.get_logger().info('goal reached — next metre')


def _yaw(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def main() -> None:
    rclpy.init()
    node = RoverNavGoals()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
