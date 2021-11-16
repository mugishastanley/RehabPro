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



using namespace std;

geometry_msgs::Pose desired, initial;
std::vector<double> joint_value = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
bool i_arrived = 0;
bool g_arrived = 0;

void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  if(goal->header.frame_id == "des_init"){
    initial.position.x = goal->pose.position.x;
    initial.position.y = goal->pose.position.y;
    initial.position.z = goal->pose.position.z;
    //initial.orientation.w = 1;
    i_arrived = 1;

  }

  if(goal->header.frame_id == "des_goal"){
    desired.position.x = goal->pose.position.x;
    desired.position.y = goal->pose.position.y;
    desired.position.z = goal->pose.position.z;
    //desired.orientation.w = 1;
    g_arrived = 1;
  }
  
  
}

void j_cb(const std_msgs::Float64MultiArray::ConstPtr& jvalue)
{

  joint_value[0] = jvalue->data[0];
  joint_value[1] = jvalue->data[1];
  joint_value[2] = jvalue->data[2];
  joint_value[3] = jvalue->data[3];
  joint_value[4] = jvalue->data[4];
  joint_value[5] = jvalue->data[5];

  cout << "joints: " << endl;
  cout << "j0 = " << joint_value[0] << endl;
  cout << "j1 = " << joint_value[1] << endl;
  cout << "j2 = " << joint_value[2] << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_cartesian");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::Subscriber sub = node_handle.subscribe("des_goal", 1000, chatterCallback);
  ros::Subscriber sub_joint = node_handle.subscribe("jvalue", 1000, j_cb);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<geometry_msgs::Pose> waypoints;


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
  moveit_msgs::RobotTrajectory trajectory;

  geometry_msgs::Pose midpoint, first_pose;
  int flag = 0;

  geometry_msgs::PoseStamped ee_pose;
  ee_pose = move_group.getCurrentPose();

  first_pose.position.x = ee_pose.pose.position.x;
  first_pose.position.y = ee_pose.pose.position.y;
  first_pose.position.z = ee_pose.pose.position.z;
  //first_pose.orientation.w = 1;

  waypoints.push_back(first_pose);
  waypoints.push_back(initial);
  //waypoints.push_back(desired);

  midpoint = initial;
  

  
  // TO GENERATE THE WAYPOINTS
  

  while(midpoint != desired){
    if (midpoint.position.x > desired.position.x){
      midpoint.position.x -= 0.1;
      waypoints.push_back(midpoint);
    }

    else if ((midpoint.position.x <= desired.position.x) && 
              (midpoint.position.z < desired.position.z)){
      midpoint.position.x = desired.position.x;
      midpoint.position.z += 0.1;
      waypoints.push_back(midpoint);

      if (midpoint.position.z >= desired.position.z){
        midpoint = desired;
        waypoints.push_back(desired);
        break;
      }
    }

  }
  
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  
  sleep(15.0);
  my_plan.trajectory_ = trajectory; 
  move_group.execute(my_plan);
  

  

  
  /*
  // TO MOVE WITH PLAN PROJECTION

  ros::Rate rate(10.0);
  while(node_handle.ok())
  {
    

    //Cartesian goal
    if(i_arrived ==1 && g_arrived == 1){

    move_group.setPoseTarget(initial);
    cout << "initial: " << endl;
    cout << "x = " << initial.position.x << endl;
    cout << "y = " << initial.position.y << endl;
    cout << "z = " << initial.position.z << endl;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");
    if(success){
      ROS_INFO("Visualizing plan 1 as trajectory line");
      visual_tools.publishAxisLabeled(initial, "initial");
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

      //std::cout << my_plan.trajectory_ << std::endl;

      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
      move_group.execute(my_plan);
    }

    if(!success)
    return 0;

    midpoint = initial;
    midpoint.position.y = 0.5; //0.7 normally (plane)
    move_group.setPoseTarget(midpoint);
    cout << "init_mid: " << endl;
    cout << "x = " << midpoint.position.x << endl;
    cout << "y = " << midpoint.position.y << endl;
    cout << "z = " << midpoint.position.z << endl;
    //move_group.setJointValueTarget(joint_value);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");
    if(success){
      ROS_INFO("Visualizing plan 2 as trajectory line");
      visual_tools.publishAxisLabeled(midpoint, "init_mid");
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

      //std::cout << my_plan.trajectory_ << std::endl;

      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
      move_group.execute(my_plan);
    }

    if(!success)
    return 0;


    midpoint = desired;
    midpoint.position.y = 0.5; //0.7 normally (plane)
    move_group.setPoseTarget(midpoint);
    cout << "des_mid: " << endl;
    cout << "x = " << midpoint.position.x << endl;
    cout << "y = " << midpoint.position.y << endl;
    cout << "z = " << midpoint.position.z << endl;
    //move_group.setJointValueTarget(joint_value);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");
    if(success){
      ROS_INFO("Visualizing plan 3 as trajectory line");
      visual_tools.publishAxisLabeled(midpoint, "des_mid");
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

      //std::cout << my_plan.trajectory_ << std::endl;

      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
      move_group.execute(my_plan);
    }

    if(!success)
    return 0;

    move_group.setPoseTarget(desired);
    cout << "desired: " << endl;
    cout << "x = " << desired.position.x << endl;
    cout << "y = " << desired.position.y << endl;
    cout << "z = " << desired.position.z << endl;
    //move_group.setJointValueTarget(joint_value);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");
    if(success){
      ROS_INFO("Visualizing plan 4 as trajectory line");
      visual_tools.publishAxisLabeled(desired, "desired");
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

      //std::cout << my_plan.trajectory_ << std::endl;

      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
      move_group.execute(my_plan);
      i_arrived = 0;
      g_arrived = 0;
    }

    if(!success)
    return 0;
    }

    //move_group.asyncMove();
    
    //std::vector<double> end_joints;

    //end_joints = move_group.getCurrentJointValues();

    //cout << "Current joints: " << endl;
    //cout << end_joints[0] << endl;
    //cout << end_joints[1] << endl;
    //cout << end_joints[2] << endl;

    //geometry_msgs::PoseStamped ee_pose;
    //ee_pose = move_group.getCurrentPose();
    //cout << "End effector: " << endl;
    //cout << "x = " << ee_pose.pose.position.x << endl;
    //cout << "y = " << ee_pose.pose.position.y << endl;
    //cout << "z = " << ee_pose.pose.position.z << endl;

      
    rate.sleep();
    visual_tools.deleteAllMarkers();
  }

  // UNTIL HERE
  */


    //move_group.setPoseTarget(desired);

    //bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");

    //move_group.asyncMove();
  ros::spinOnce();
  visual_tools.deleteAllMarkers();
  ros::shutdown();
  return 0;
};
