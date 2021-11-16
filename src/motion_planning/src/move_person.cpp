#include <iostream>
#include <tf2/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <math.h>

using namespace std;

// code that publish a sinusoidal signal in the LShoulderPsi joint of the mannequin model, for testing purposes

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_person");

  ros::NodeHandle node_handle;

  ros::Publisher chatter_pub = node_handle.advertise<sensor_msgs::JointState>("/joint_states", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  ROS_INFO("Moving arm to a random point");
  sensor_msgs::JointState goal;
  vector<std::string> name {"LShoulderPsi"};
  goal.name = name;
  double value = 0.0;
  vector<double> joint_value;
  double t = 0.0;
  ros::Rate(10);
  while (ros::ok())
  {
    value = 1.57*sin(2*M_PI*2*t);
    t += 0.01;
    joint_value = {value};
    goal.position = joint_value;
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(goal);

    ros::spinOnce();

    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}
