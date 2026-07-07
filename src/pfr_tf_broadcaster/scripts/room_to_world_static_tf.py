#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    half_roll = 0.5 * roll
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class RoomToWorldStaticTf(Node):
    def __init__(self):
        super().__init__("room_to_world_static_tf")

        self.declare_parameter("parent_frame", "room")
        self.declare_parameter("child_frame", "world")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("roll", math.pi / 2.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 0.0)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = (
            self.get_parameter("parent_frame").get_parameter_value().string_value
        )
        transform.child_frame_id = (
            self.get_parameter("child_frame").get_parameter_value().string_value
        )
        transform.transform.translation.x = (
            self.get_parameter("x").get_parameter_value().double_value
        )
        transform.transform.translation.y = (
            self.get_parameter("y").get_parameter_value().double_value
        )
        transform.transform.translation.z = (
            self.get_parameter("z").get_parameter_value().double_value
        )

        roll = self.get_parameter("roll").get_parameter_value().double_value
        pitch = self.get_parameter("pitch").get_parameter_value().double_value
        yaw = self.get_parameter("yaw").get_parameter_value().double_value
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(transform)

        self.get_logger().info(
            "Published static TF %s -> %s: xyz=(%.3f, %.3f, %.3f), "
            "rpy=(%.3f, %.3f, %.3f) rad"
            % (
                transform.header.frame_id,
                transform.child_frame_id,
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                roll,
                pitch,
                yaw,
            )
        )


def main():
    rclpy.init()
    node = RoomToWorldStaticTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
