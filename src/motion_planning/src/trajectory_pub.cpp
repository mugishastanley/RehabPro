#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"


#define _USE_MATH_DEFINES
#include <cmath>

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "trajectory_pub");

  ros::NodeHandle n;

  //ros::Publisher pub = n.advertise<geometry_msgs::Pose>("follow_traj", 1000);
  ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command", 1000);
  ros::Rate loop_rate(5);

  double angle_x = 0;
  double angle_y = 0;
  double angle_z = 0;
  double inc = 10;

  trajectory_msgs::JointTrajectory goal;
  trajectory_msgs::JointTrajectoryPoint point_goal;

  goal.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  
  point_goal.positions = {1.57, 0.0, 0.0, 0.0, 0.0, 0.0};
  ros::Duration tts(0.2);
  point_goal.time_from_start = {tts};

  goal.points.push_back(point_goal);

  std::cout << goal.points[0] << std::endl;
  int count = 0;
  while (n.ok())
  {
    /*
    count++;
    angle_x += inc;
    angle_y += inc;
    angle_z += inc;

    if(angle_x >= 360){
      inc = -inc;
    }
    else if(angle_x < 0){
      inc = -inc;
    }

    geometry_msgs::Pose goal;

    goal.position.x = 0.5*cos(angle_x*M_PI/180);
    goal.position.y = 0.5*sin(angle_y*M_PI/180);
    goal.position.z = 1+0.5*sin(angle_z*M_PI/180);

    ROS_INFO("x: %f", goal.position.x);
    ROS_INFO("y: %f", goal.position.y);
    ROS_INFO("z: %f", goal.position.z);

    */
    pub.publish(goal);

    ros::spinOnce();

    loop_rate.sleep();
  }

  ros::shutdown();

  return 0;
}