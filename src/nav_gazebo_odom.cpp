// Copyright (c) 2018 Project SRS
// Released under the MIT license
// https://github.com/project-srs/ros_lecture/blob/master/LICENSE

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/timer.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "math.h"
#include <string>
#include <random>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

std::string odom_frame="odom";
std::string base_frame="base_link";
float publish_rate=20.0;
float noise=0.0;
bool tf_enable=false;
bool enable_odom=false;
nav_msgs::msg::Odometry last_odom;

class NavModelBasePublisher : public rclcpp::Node{
  
  public:
    NavModelBasePublisher(): Node("nav_model_base_publisher"){
      this->declare_parameter<std::string>("publish_rate");
      publish_rate = this->get_parameter("publish_rate").as_double();
      rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub = this->create_subscription<gazebo_msgs::msg::ModelStates>("gazebo/model_states", 10, std::bind(&NavModelBasePublisher::model_callback,this,_1));
      rclcpp::TimerBase::SharedPtr odom_timer = this->create_wall_timer(500ms, std::bind(&NavModelBasePublisher::timer_callback, this));
    }

    double sdlab_uniform(){
      double ret = ((double) rand() + 1.0) / ((double) RAND_MAX + 2.0);
      return ret;
    };
      
    double sdlab_normal(double mu, double sigma){
      double  z = sqrt(-2.0 * log(sdlab_uniform())) * sin(2.0 * M_PI * sdlab_uniform());
      return mu + sigma * z;
    };

    void tf_publish(geometry_msgs::msg::Pose pose0){
      this->declare_parameter<std::string>("odom_frame");
      odom_frame = this->get_parameter("odom_frame").as_string();
      this->declare_parameter<std::string>("base_frame");
      base_frame = this->get_parameter("base_frame").as_string();
      std::shared_ptr<tf2_ros::TransformBroadcaster> br;
      geometry_msgs::msg::Transform transform;
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->now();
      t.header.frame_id = odom_frame;
      t.child_frame_id = base_frame;
      t.transform.translation.x = pose0.position.x;
      t.transform.translation.y = pose0.position.y;
      t.transform.translation.z = pose0.position.z;
      t.transform.rotation = pose0.orientation;
      br->sendTransform(t);
    };

    void timer_callback(){
      this->declare_parameter<std::string>("tf_enable");
      tf_enable = this->get_parameter("tf_enable").as_bool();
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
      odom_pub->publish(last_odom);
      if(tf_enable){
        tf_publish(last_odom.pose.pose);
      }
    };

    void model_callback(const gazebo_msgs::msg::ModelStates::SharedPtr model_msg){
      this->declare_parameter<std::string>("noise");
      noise = this->get_parameter("noise").as_double();
      int model_size=model_msg->name.size();
      for(int i=0;i<model_size;i++){
          last_odom.pose.pose=model_msg->pose[i];
          last_odom.pose.pose.position.x+=sdlab_normal(0.0, noise);
          last_odom.pose.pose.position.y+=sdlab_normal(0.0, noise);
          last_odom.pose.covariance = {
          0.5, 0, 0, 0, 0, 0,  // covariance on gps_x
          0, 0.5, 0, 0, 0, 0,  // covariance on gps_y
          0, 0, 0.5, 0, 0, 0,  // covariance on gps_z
          0, 0, 0, 0.1, 0, 0,  // large covariance on rot x
          0, 0, 0, 0, 0.1, 0,  // large covariance on rot y
          0, 0, 0, 0, 0, 0.1}; // large covariance on rot z

    /*
          last_odom.twist.twist=model_msg.twist[i];
          last_odom.twist.covariance = {
          1000, 0, 0, 0, 0, 0,  // covariance on gps_x
          0, 1000, 0, 0, 0, 0,  // covariance on gps_y
          0, 0, 1000, 0, 0, 0,  // covariance on gps_z
          0, 0, 0, 1000, 0, 0,  // large covariance on rot x
          0, 0, 0, 0, 1000, 0,  // large covariance on rot y
          0, 0, 0, 0, 0, 1000}; // large covariance on rot z
    */
          enable_odom=true;
        }
      }
    };



int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavModelBasePublisher>());
  rclcpp::shutdown();
  return 0;
}