#!/usr/bin/python
import rclpy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from rclpy.node import Node

pose = PoseStamped()
is_sub = bool()

class PoseToOdom(Node):

    def __init__(self):
        super().__init__('pose_to_odom')
        self.counter = 0
        self.x =0
        self.y = 0
        self.x_prev = 0
        self.y_prev = 0
        self.z_prev = 0
        self.declare_parameter('odom_header_frame',"world")
        self.declare_parameter('odom_child_frame',"ic120_tf/base_link")
        self.declare_parameter('poseStamped_topic_name',"base_link/pose")
        self.declare_parameter('odom_topic_name',"tracking/ground_truth")
        self.odom_header_frame = self.get_parameter('odom_header_frame').get_parameter_value().string_value
        self.odom_child_frame = self.get_parameter('odom_child_frame').get_parameter_value().string_value
        self.poseStamped_topic_name = self.get_parameter('poseStamped_topic_name').get_parameter_value().string_value
        self.odom_topic_name = self.get_parameter('odom_topic_name').get_parameter_value().string_value
        self.vicon_sub = self.create_subscription(Odometry, self.poseStamped_topic_name, self.pose_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic_name, 10)


    def odom_publisher(self):
        rate = rclpy.Rate(50.0)
        dt = 1./50.

        (v_roll, v_pitch, v_yaw) = self.quaternion_to_euler_angle(pose.pose.orientation.w,pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z)
        v_phi = float((v_roll))
        v_theta = float((v_pitch))
        v_psi = float((v_yaw))

        self.x = pose.pose.position.x
        self.y = pose.pose.position.y
        self.z = pose.pose.position.z

        yaw = math.radians(v_psi)

        if counter > 0:
            vel_x_world = (self.x - self.x_prev) / dt
            vel_y_world = (self.y - self.y_prev) / dt
                
            self.x_prev = self.x
            self.y_prev = self.y
                
            twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world
            twist_y = math.cos(yaw) * vel_y_world - math.sin(yaw) * vel_x_world
                
            odom = Odometry()
            odom.header.frame_id = self.odom_header_frame
            odom.child_frame_id = self.odom_child_frame
            # odom.header.stamp = rospy.Time.now()
            # odom.header.stamp = pose.header.stamp
                
            odom.pose.pose.position.x = pose.pose.position.x
            odom.pose.pose.position.y = pose.pose.position.y
            odom.pose.pose.position.z = pose.pose.position.z
                
            odom.pose.pose.orientation.x = pose.pose.orientation.x
            odom.pose.pose.orientation.y = pose.pose.orientation.y
            odom.pose.pose.orientation.z = pose.pose.orientation.z
            odom.pose.pose.orientation.w = pose.pose.orientation.w
                
            odom.twist.twist.linear.x = twist_x
            odom.twist.twist.linear.y = twist_y
            odom.twist.twist.linear.z = (self.z - z_prev) / dt
            z_prev = self.z
                
            odom.twist.twist.angular.x = 0.
            odom.twist.twist.angular.y = 0.
            odom.twist.twist.angular.z = 0.
                
            if is_sub == True:
                self.odom_pub.publish(odom)
                # br = tf.TransformBroadcaster()
                # br.sendTransform((x, y, z), [pose.pose.orientation.x, pose.pose.orientation.y,
                #                          pose.pose.orientation.z, pose.pose.orientation.w], rospy.Time.now(), "gnss/base_link", "map")
                is_sub = False
                
        else:
            self.x_prev = self.x
            self.y_prev = self.y
            self.z_prev = self.z
            counter += 1

        rate.sleep()


    def pose_cb(self, data):
        global pose
        pose = data
        global is_sub
        is_sub = True
        self.odom_publisher()


    def quaternion_to_euler_angle(w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0, t1))
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3, t4))
        
        return X, Y, Z


def main(args=None):
    rclpy.init(args=args)
    pose_to_odom = PoseToOdom()
    rclpy.spin(pose_to_odom)
    pose_to_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()