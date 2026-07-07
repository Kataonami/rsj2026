#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_quaternion(q):
    norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if norm < 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def multiply_quaternions(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize_quaternion(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def rotate_vector(q, v):
    qx, qy, qz, qw = normalize_quaternion(q)
    vx, vy, vz = v

    # q * v * q^-1, expanded to avoid extra temporary objects.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class BasePoseTransformer(Node):
    def __init__(self):
        super().__init__("base_pose_transformer")

        self.declare_parameter("input_topic", "/vrpn_mocap/PFR_Arm/pose")
        self.declare_parameter("output_topic", "/pfr/base_pose")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("source_frame_override", "room")
        self.declare_parameter("default_source_frame", "room")

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.source_frame_override = self.get_parameter("source_frame_override").value
        self.default_source_frame = self.get_parameter("default_source_frame").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.subscription = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Transforming base pose %s -> %s in target frame '%s'."
            % (self.input_topic, self.output_topic, self.target_frame)
        )

    def pose_callback(self, msg):
        source_frame = (
            self.source_frame_override or msg.header.frame_id or self.default_source_frame
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time(),
            )
        except TransformException as error:
            self.get_logger().warn(
                "Could not transform '%s' to '%s': %s"
                % (source_frame, self.target_frame, error),
                throttle_duration_sec=2.0,
            )
            return

        tq = transform.transform.rotation
        tp = transform.transform.translation
        transform_q = normalize_quaternion((tq.x, tq.y, tq.z, tq.w))

        input_p = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        rotated_p = rotate_vector(transform_q, input_p)
        output_q = multiply_quaternions(
            transform_q,
            (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
        )

        output = PoseStamped()
        output.header = msg.header
        output.header.frame_id = self.target_frame
        output.pose.position.x = rotated_p[0] + tp.x
        output.pose.position.y = rotated_p[1] + tp.y
        output.pose.position.z = rotated_p[2] + tp.z
        output.pose.orientation.x = output_q[0]
        output.pose.orientation.y = output_q[1]
        output.pose.orientation.z = output_q[2]
        output.pose.orientation.w = output_q[3]

        self.publisher.publish(output)


def main():
    rclpy.init()
    node = BasePoseTransformer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
