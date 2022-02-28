#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
# from geometry_msgs.msg import PoseStamped, TransformStamped
# from tf2_msgs.msg import TFMessage
import tf

br = tf.TransformBroadcaster()
odom_frame = "odom"
base_link_frame = "base_link"
def odom_cb(odom):
    br.sendTransform(
        (odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z),
        [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w],
        odom.header.stamp,
        base_link_frame, odom_frame)

rospy.init_node('odom_tf_broadcaster')

odom_frame = rospy.get_param("~odom_frame")
base_link_frame = rospy.get_param("~base_link_frame")

odom_sub = rospy.Subscriber('odom', Odometry, odom_cb, queue_size=10)

rospy.spin()
