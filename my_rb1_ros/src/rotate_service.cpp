#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include <geometry_msgs/Twist.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// Import the service message header file generated from the rotate message
class RotateService {
public:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  ros::ServiceServer my_service;
  nav_msgs::Odometry odom;
  geometry_msgs::Twist vel;
  double roll, pitch, yaw;
  RotateService() {
    sub = nh.subscribe("/odom", 1000, &RotateService::odomCallback, this);
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    my_service =
        nh.advertiseService("/rotate_robot", &RotateService::my_callback, this);
    this->yaw = 0.0;
  }
  bool my_callback(my_rb1_ros::Rotate::Request &req,
                   my_rb1_ros::Rotate::Response &res) {
    res.result = "Failed to Rotate!";
    // res.some_variable = req.some_variable + req.other_variable;
    // We print an string whenever the Service gets called
    double rotate_goal = req.degrees * 0.0174533;
    double yaw;

    ros::Rate rate(10);
    int i = 0;
    double start_angle = this->yaw;
    while (abs((this->yaw) - start_angle) < abs(rotate_goal)) {
      float speed = 0.2;
      if (rotate_goal < 0) {
        speed = -0.2;
      }
      ROS_INFO("current speed is %f", speed);
      vel.angular.z = speed;
      pub.publish(vel);
      ROS_INFO("rotating goal %f", rotate_goal);
      ROS_INFO("resulting angle yaw  %f", this->yaw);
      ROS_INFO("start angle  %f", start_angle);
      ros::spinOnce();

      i++;
    }
    vel.angular.z = 0.0;
    pub.publish(vel);

    res.result = "Succesfully Rotated!";
    return true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    //("x: %f", msg->pose.pose.orientation.x);
    // ROS_INFO("y: %f", msg->pose.pose.orientation.y);
    // ROS_INFO("z: %f", msg->pose.pose.orientation.z);
    // ROS_INFO("w: %f", msg->pose.pose.orientation.w);
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double qy, qx, qz, qw;
    qx = msg->pose.pose.orientation.x;
    qy = msg->pose.pose.orientation.y;
    qz = msg->pose.pose.orientation.z;
    qw = msg->pose.pose.orientation.w;
    tf2::Matrix3x3 m(q);

    double yaw =
        atan2(2.0 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz);
    double pitch = asin(-2.0 * (qx * qz - qw * qy));
    double roll =
        atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);

    this->yaw = roll;
    // ROS_INFO("resulting angle yaw callback %f", yaw);
    ROS_INFO("resulting angle roll callback %f", this->yaw);
    // ROS_INFO("resulting angle pitch callback %f", pitch);
  }
};

// We define the callback function of the service

int main(int argc, char **argv) {
  ros::init(argc, argv, "service_server");
  RotateService spinme;
  ros::spin(); // mantain the service open.

  return 0;
}