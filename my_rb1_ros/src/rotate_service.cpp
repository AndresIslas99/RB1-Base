#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Scalar.h>

// Global variable to store the robot's current yaw
double current_yaw = 0;

// Callback to update the robot's current yaw based on Odometry data
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  double roll, pitch, yaw;
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  current_yaw = yaw;
}

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool rotate(my_rb1_ros::Rotate::Request &req,
            my_rb1_ros::Rotate::Response &res) {
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  double target_yaw = normalize_angle(current_yaw + req.degrees * M_PI / 180.0);

  geometry_msgs::Twist twist_msg;

  while (ros::ok()) {
    double error = normalize_angle(target_yaw - current_yaw);


    // Simple proportional control to minimize the error
    twist_msg.angular.z = 0.5 * error;

    // Send the velocity command
    pub.publish(twist_msg);

    if (fabs(error) < 0.01) { // Stop condition: error below a threshold
      twist_msg.angular.z = 0;
      pub.publish(twist_msg); // Stop the robot
      res.result = "Rotation completed successfully";
      return true;
    }

    ros::spinOnce(); // Process incoming messages, including odometry updates
  }

  return false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback);
  ros::ServiceServer service = n.advertiseService("/rotate_robot", rotate);

  ros::spin();

  return 0;
}
