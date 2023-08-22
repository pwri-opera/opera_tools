#!/usr/bin/python
import rclpy
import math
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.node import Node


class OdomBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.declare_parameter('odom_frame', "odom")
        self.declare_parameter('base_link_frame', "base_link")
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value
        self.odom_sub = self.create_subscription(Odometry, self.odom_frame, self.odom_cb, 10)

    def odom_cb(self, msg):
        self.br.sendTransform(
            (msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
            (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
            msg.header.stamp,
            self.base_link_frame,
            self.odom_frame
        )


def main(args=None):
    rclpy.init(args=args)
    odom_broadcaster = OdomBroadcaster()
    rclpy.spin(odom_broadcaster)
    odom_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()