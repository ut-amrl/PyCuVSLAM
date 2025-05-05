#!/usr/bin/env python3
import os
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, JointState
from ament_index_python.packages import get_package_share_directory


import numpy as np
from rosbags.image import message_to_cvimage


from .tracker import VisualWheelOdometry


class VisualWheelOdometryNode(Node):
    def __init__(self):
        super().__init__("pycuvslam_node")
        share = get_package_share_directory('pycuvslam_ros2')
        default_cfg = os.path.join(share, 'config', 'zero.yaml')

        self.declare_parameter("config_path", default_cfg)
        config_path = self.get_parameter("config_path").value

        # Initialize the tracker
        self.vwo = VisualWheelOdometry(config_path)
        self.last_pose = None
        self.last_time = None

        self.image_buffer = deque(maxlen=100)

        # Subscribers
        image_topic = "/front_camera/compressed"
        wheel_topic = "/wheel_encoder"
        self.create_subscription(CompressedImage, image_topic, self.image_callback, 10)
        self.create_subscription(JointState, wheel_topic, self.wheel_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.get_logger().info("Visual Wheel Odometry Node Initialized")

        self.create_timer(0.1, self.tick)

    def wheel_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        omega_left = 0.5 * (msg.velocity[0] + msg.velocity[2])
        omega_right = 0.5 * (msg.velocity[1] + msg.velocity[3])
        self.vwo.update_wheel(ts, omega_left, omega_right)

    def image_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.image_buffer.append((ts, msg))

    def tick(self):
        if not self.image_buffer:
            return

        img_ts, img_msg = self.image_buffer[0]

        if img_ts > self.vwo.last_wheel_ts:
            self.get_logger().debug("Waiting for wheel data to catch up")
            return

        self.image_buffer.popleft()

        # update image frame
        image = message_to_cvimage(img_msg, "rgb8")
        pose = self.vwo.update_frame(img_ts, image)

        if pose is None:
            self.get_logger().warning("Pose is None, skipping frame")
            return

        x, y, yaw = pose[0], pose[1], pose[2]
        qz, qw = np.sin(yaw / 2), np.cos(yaw / 2)

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = img_msg.header.stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set pose
        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = float(qz)
        odom_msg.pose.pose.orientation.w = float(qw)

        if self.last_pose is not None:
            dt = img_ts - self.last_time
            if dt > 1e-6:
                dx = x - self.last_pose[0]
                dy = y - self.last_pose[1]
                v = np.hypot(dx, dy) / dt
                delta_yaw = (yaw - self.last_pose[2] + np.pi) % (2 * np.pi) - np.pi
                w = delta_yaw / dt
                odom_msg.twist.twist.linear.x = float(v)
                odom_msg.twist.twist.angular.z = float(w)

        # Publish odometry
        self.odom_pub.publish(odom_msg)
        self.get_logger().debug(
            f"Fused Odom t={img_ts:.3f} x={x:.3f} y={y:.3f} yaw={yaw:.3f}"
        )

        self.last_pose = (x, y, yaw)
        self.last_time = img_ts


def main(args=None):
    rclpy.init(args=args)
    node = VisualWheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
