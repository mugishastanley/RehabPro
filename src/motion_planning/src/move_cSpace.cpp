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


using namespace std;

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->header.frame_id);
  ROS_INFO("x: %f", msg->pose.position.x);
  ROS_INFO("y: %f", msg->pose.position.y);
  ROS_INFO("z: %f", msg->pose.position.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_cSpace");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher chatter_pub = node_handle.advertise<geometry_msgs::Pose>("/ee_pose", 1000);
  ros::Subscriber sub = node_handle.subscribe("distance_left_hand", 1000, callback);

  static const std::string PLANNING_GROUP = "manipulator";
  
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("pedestal");
  
  //rviz_visual_tools::RvizVisualTools visual_tools("base_link","/rviz_visual_markers");
  //visual_tools.reset(new rviz_visual_tools::RvizVisualTools("base_link","/rviz_visual_markers"));
  
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.25;
  visual_tools.publishText(text_pose, "UR5 with person model", rvt::WHITE, rvt::XLARGE);

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

  //Cartesian goal

  geometry_msgs::Pose desired;
  desired.orientation.w = 1;
  desired.position.x = 0.5;
  desired.position.y = 0.5;
  desired.position.z = 1.0;
  move_group.setPoseTarget(desired);
  ROS_INFO("desired pose1:");
  ROS_INFO("x: %f ", desired.position.x);
  ROS_INFO("y: %f ", desired.position.y);
  ROS_INFO("z: %f ", desired.position.z);

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(desired, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(my_plan);
  }

  if(!success)
  return 0;
  //move_group.asyncMove();
  

  geometry_msgs::PoseStamped ee_pose;
  ee_pose = move_group.getCurrentPose();
  ROS_INFO("ee_pose: ");
  ROS_INFO("x: %f ", ee_pose.pose.position.x);
  ROS_INFO("y: %f ", ee_pose.pose.position.y);
  ROS_INFO("z: %f ", ee_pose.pose.position.z);



  //Second surface

  desired.orientation.w = 1;
  desired.position.x = 0.0;
  desired.position.y = 0.6;
  desired.position.z = 1.2;
  move_group.setPoseTarget(desired);
  ROS_INFO("desired pose2:");
  ROS_INFO("x: %f ", desired.position.x);
  ROS_INFO("y: %f ", desired.position.y);
  ROS_INFO("z: %f ", desired.position.z);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 2 as trajectory line");
    visual_tools.publishAxisLabeled(desired, "pose2");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(my_plan);
  }

  if(!success)
  return 0;
  //move_group.asyncMove();
  
  ee_pose = move_group.getCurrentPose();
  ROS_INFO("ee_pose: ");
  ROS_INFO("x: %f ", ee_pose.pose.position.x);
  ROS_INFO("y: %f ", ee_pose.pose.position.y);
  ROS_INFO("z: %f ", ee_pose.pose.position.z);



  //Third surface
  desired.orientation.w = 1;
  desired.position.x = -0.6;
  desired.position.y = 0.1;
  desired.position.z = 1.0;
  move_group.setPoseTarget(desired);
  ROS_INFO("desired pose:");
  ROS_INFO("x: %f ", desired.position.x);
  ROS_INFO("y: %f ", desired.position.y);
  ROS_INFO("z: %f ", desired.position.z);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(desired, "pose1");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(my_plan);
  }

  if(!success)
  return 0;
  //move_group.asyncMove();
  
  ee_pose = move_group.getCurrentPose();
  ROS_INFO("ee_pose: ");
  ROS_INFO("x: %f ", ee_pose.pose.position.x);
  ROS_INFO("y: %f ", ee_pose.pose.position.y);
  ROS_INFO("z: %f ", ee_pose.pose.position.z);

  //Joint state goal

  // //Execute a joint-space goal to a known predefined position from Home position
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, modify the joints to the desired position, plan to the new joint space goal
  // Start at positive x and y


  joint_group_positions[0] = -1.74; // radians
  joint_group_positions[1] = -0.916; // radians
  joint_group_positions[2] = -1.66; // radians
  joint_group_positions[3] = -3.66;//-3.4; // radians
  joint_group_positions[4] = -1.71; // radians
  joint_group_positions[5] = -0.006; // radians
  
/*
  joint_group_positions[0] = 0.0; // radians
  joint_group_positions[1] = 0.0; // radians
  joint_group_positions[2] = 0.0; // radians
  joint_group_positions[3] = 0.0;//-3.4; // radians
  joint_group_positions[4] = 0.0; // radians
  joint_group_positions[5] = 0.0; // radians
*/
  // Start at negative x and y
  // joint_group_positions[0] = 1.69; // radians
  // joint_group_positions[1] = -1.42; // radians
  // joint_group_positions[2] = -2.26; // radians
  // joint_group_positions[3] = -2.56; // radians
  // joint_group_positions[4] = -1.40; // radians
  // joint_group_positions[5] = -0.006; // radians


  move_group.setMaxVelocityScalingFactor(0.08);
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");
  if(success){
    visual_tools.deleteAllMarkers();
  
    ROS_INFO("Visualizing plan 1 as trajectory line");
    //visual_tools.publishAxisLabeled(desired, "pose1");
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(my_plan);
  }
  
  if(!success)
  return 0;


  // Cartesian Paths
  // //Execute a joint-space goal to a known predefined position from Home position
  current_state = move_group.getCurrentState();
  joint_group_positions[0] = 1.57; // radians
  joint_group_positions[1] = 0.0; // radians
  joint_group_positions[2] = 0.0; // radians
  joint_group_positions[3] = 0.0;//-3.4; // radians
  joint_group_positions[4] = 0.0; // radians
  joint_group_positions[5] = 0.0; // radians

  move_group.setMaxVelocityScalingFactor(0.08);
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //move_group.asyncMove();
  if(success){
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal 2", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(my_plan);
  }
  
  if(!success)
  return 0;


  ros::shutdown();
  return 0;
};
