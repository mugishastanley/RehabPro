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
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <rosbag/bag.h>

using namespace std;


// Code that generates a motion plan and rescale accordingly the velocities and accelerations depending on the scaling given.
// Intended to manipulate the computed trajectory for it to be executed faster or slower, even though the computations are being properly done, does not behave good.


int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_scaling");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group.setPlannerId("RRTConnect");

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("pedestal");

  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.25;
  visual_tools.publishText(text_pose, "UR5 robot", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO("Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
  std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  moveit_msgs::RobotTrajectory trajectory;

  std::vector<double> end_joints;

  // Trajectory initial->home

  std::vector<double> home = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  home[0] = 0.7897;
  home[1] = -2.3015;
  home[2] = 1.8849;
  home[3] = 0.2061;
  home[4] = 1.5631;
  home[5] = -0.00698132;


  move_group.setJointValueTarget(home);

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from initial to home  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 1 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(my_plan);
  }

  if(!success)
  return 0;


  // Second trajectory home->door_2

  std::vector<double> door_2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // door_2 coordinates
  door_2[0] = -0.23509;
  door_2[1] = -0.30717;
  door_2[2] = 0.53372;
  door_2[3] = 0.4527;
  door_2[4] = -0.7316;
  door_2[5] = 1.9249;


  move_group.setJointValueTarget(door_2);

  success = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from home to door_2  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 0_1 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "plan 0_1", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(my_plan2);
  }

  if(!success)
  return 0;


  // Relocate at home position
  move_group.setJointValueTarget(home);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from initial to home  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 1 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(my_plan);
  }

  if(!success)
  return 0;


  // Execute same trajectory but faster
  moveit_msgs::RobotTrajectory the_traj;
  ros::Duration dur;
  the_traj.joint_trajectory = my_plan2.trajectory_.joint_trajectory;

  //int n_joints = sizeof(my_plan2.trajectory_.joint_trajectory.joint_names);   // doesn't works
  int n_joints = 6;
  int n_points = sizeof(my_plan2.trajectory_.joint_trajectory.points);
  //cout << my_plan2.trajectory_.joint_trajectory << endl;
  double spd = 2.0;
  double inv_spd = 1.0/spd;

  for(int i = 0; i < n_points; ++i){
    dur = my_plan2.trajectory_.joint_trajectory.points[i].time_from_start;
    my_plan2.trajectory_.joint_trajectory.points[i].time_from_start = dur * inv_spd;
    cout << "times: " << my_plan2.trajectory_.joint_trajectory.points[i].time_from_start << " " << dur << endl;
    cout << " " << endl;
    for(int j = 0; j < n_joints; ++j){
      the_traj.joint_trajectory.points[i].velocities[j] = my_plan2.trajectory_.joint_trajectory.points[i].velocities[j] * spd;
      cout << "vels: "<< the_traj.joint_trajectory.points[i].velocities[j] << " " << my_plan2.trajectory_.joint_trajectory.points[i].velocities[j] << endl;
      the_traj.joint_trajectory.points[i].accelerations[j] = my_plan2.trajectory_.joint_trajectory.points[i].accelerations[j] * spd;
      cout << "accels: "<< the_traj.joint_trajectory.points[i].accelerations[j] << " " << my_plan2.trajectory_.joint_trajectory.points[i].accelerations[j] << endl;
      cout << " " << endl;
      the_traj.joint_trajectory.points[i].positions[j] = my_plan2.trajectory_.joint_trajectory.points[i].positions[j];
    }
  }
  my_plan2.trajectory_ = the_traj;
  move_group.execute(the_traj);


  ros::spinOnce();
  visual_tools.deleteAllMarkers();
  ros::shutdown();
  return 0;
};
