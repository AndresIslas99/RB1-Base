#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Scalar.h>

// Global variables
double current_yaw = 0;
double previous_error = 0;
double integral_error = 0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  double roll, pitch, yaw;
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  current_yaw = yaw;
}

double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double pid_control(double error, double dt) {
  //double Kp = 1.5;  // Proportional gain
  //double Ki = 0.045; // Integral gain
  //double Kd = 1.3;  // Derivative gain for critical damping
  double Kp = 3;  // Proportional gain
  double Ki = 0.05; // Integral gain
  double Kd = 0.1;  // Derivative gain for critical damping
  integral_error += error * dt;
  double derivative_error = (error - previous_error) / dt;
  previous_error = error;
  double control_output =
      Kp * error + Ki * integral_error + Kd * derivative_error;
  control_output = std::max(std::min(control_output, 1.5),
                            -1.5); // Saturation limits between -1 and 1
  return control_output;
}

ros::Publisher pub; // Global variable

bool rotate(my_rb1_ros::Rotate::Request &req, my_rb1_ros::Rotate::Response &res,
            ros::Publisher &pub) {
  previous_error = 0; // Reset controller state
  integral_error = 0; // Reset controller state
  double target_yaw = normalize_angle(current_yaw + req.degrees * M_PI / 180.0);
  geometry_msgs::Twist twist_msg;
  ros::Rate rate(10); // Assuming a loop rate of 10 Hz for the PID
  ros::Time start_time = ros::Time::now(); // record the start time
  double timeout = 10.0;                   // set a timeout of 10 seconds

  while (ros::ok()) {
    double error = normalize_angle(target_yaw - current_yaw);

    // PID control
    twist_msg.angular.z = pid_control(error, rate.expectedCycleTime().toSec());

    if (fabs(error) < 0.01 ||
        (ros::Time::now() - start_time).toSec() > timeout) {
      twist_msg.angular.z = 0;
      pub.publish(twist_msg); // Stop the robot
      if ((ros::Time::now() - start_time).toSec() > timeout) {
        res.result = "Rotation timed out";
      } else {
        res.result = "Rotation completed successfully";
      }
      return true;
    }
    pub.publish(twist_msg);
    ros::spinOnce();
    rate.sleep();
  }

  return false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::Subscriber sub = n.subscribe("/odom", 1000, odomCallback);
  ros::ServiceServer service = n.advertiseService<my_rb1_ros::Rotate::Request,
                                                  my_rb1_ros::Rotate::Response>(
      "/rotate_robot",
      boost::bind(&rotate, _1, _2, pub)); // Pass publisher as argument
  ros::spin();

  return 0;
}
