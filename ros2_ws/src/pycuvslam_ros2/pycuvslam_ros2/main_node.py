#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import cv2
import numpy as np
from rosbags.image import message_to_cvimage


# from .tracker import PyCuVSLAMTracker  # Placeholder for actual PyCuVSLAM import


class PyCuVSLAMNode(Node):
    def __init__(self):
        super().__init__("pycuvslam_node")

        # Initialize PyCuVSLAMTracker
        self.vslam = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, "/front_camera", self.image_callback, 10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

    def image_callback(self, msg):
        try:
            image = message_to_cvimage(msg, 'bgr8')
            ts = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

            pose = self.tracker.track(ts, image)

            # Create Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = msg.header.stamp
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            # Set pose (assuming pose is [x, y, z, qx, qy, qz, qw])
            odom_msg.pose.pose.position.x = pose[0]
            odom_msg.pose.pose.position.y = pose[1]
            odom_msg.pose.pose.position.z = pose[2]
            odom_msg.pose.pose.orientation.x = pose[3]
            odom_msg.pose.pose.orientation.y = pose[4]
            odom_msg.pose.pose.orientation.z = pose[5]
            odom_msg.pose.pose.orientation.w = pose[6]

            # Publish odometry
            self.odom_pub.publish(odom_msg)
            self.get_logger().debug("Published odometry")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PyCuVSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
