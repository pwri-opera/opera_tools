#!/usr/bin/python3
import rclpy
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node


class OdomBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.declare_parameter('odom_topic', "/ic120/odom")
        self.odom_frame = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_sub = self.create_subscription(Odometry, self.odom_frame, self.odom_cb, 10)

    def odom_cb(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "ic120_tf/odom"
        transform.child_frame_id = "ic120_tf/base_link"
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation.x = msg.pose.pose.orientation.x
        transform.transform.rotation.y = msg.pose.pose.orientation.y
        transform.transform.rotation.z = msg.pose.pose.orientation.z
        transform.transform.rotation.w = msg.pose.pose.orientation.w
        self.br.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    odom_broadcaster = OdomBroadcaster()
    rclpy.spin(odom_broadcaster)
    odom_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()