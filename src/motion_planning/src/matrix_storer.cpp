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
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>

#include <rosbag/bag.h>

using namespace std;

geometry_msgs::Pose desired, initial;

//Simpler version of matrix_storer_5 but for less trajectories

int main(int argc, char** argv)
{
  ros::init(argc, argv, "matrix_storer");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group.setPlannerId("BiTRRT");

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
  moveit::planning_interface::MoveGroupInterface::Plan plan_array[3][3];

  moveit_msgs::RobotTrajectory trajectory;

  std::vector<double> end_joints;

  std_msgs::String start_name[3], final_name[3];


  start_name[0].data = "home";
  start_name[1].data = "door_2";
  start_name[2].data = "seat_1";

  final_name[0].data = "home";
  final_name[1].data = "door_2";
  final_name[2].data = "seat_1";

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
  /* door_2 coordinates
  door_2[0] = -0.23509;
  door_2[1] = -0.30717;
  door_2[2] = 0.53372;
  door_2[3] = 0.4527;
  door_2[4] = -0.7316;
  door_2[5] = 1.9249;
  */
  // test point1
  door_2[0] = -0.0459;
  door_2[1] = -0.5494;
  door_2[2] = 0.7752;
  door_2[3] = -1.3576;
  door_2[4] = 1.2306;
  door_2[5] = 1.3617;

  std::vector<double> seat_1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  //seat_1 joint values
  seat_1[0] = 1.3629;
  seat_1[1] = 0.013;
  seat_1[2] = 0.7614;
  seat_1[3] = -0.8902;
  seat_1[4] = 1.6856;
  seat_1[5] = 1.3589;

  std::vector<double> sw_1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  //seat_1 joint values
  sw_1[0] = 0.3267;
  sw_1[1] = -1.1517;
  sw_1[2] = 2.1783;
  sw_1[3] = -1.6235;
  sw_1[4] = 1.1777;
  sw_1[5] = -0.00698132;

  move_group.allowReplanning(true);
  move_group.setJointValueTarget(door_2);

  success = (move_group.plan(plan_array[0][1]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from home to door_2  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 0_1 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "plan 0_1", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(plan_array[0][1].trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.asyncExecute(plan_array[0][1]);
  }

  if(!success)
  return 0;

  sleep(1.0);
  // Third trajectory door_2->seat_1
  move_group.stop();
  move_group.setJointValueTarget(seat_1);

  success = (move_group.plan(plan_array[1][2]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from door_2 to seat_1  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 1_2 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "plan 1_2", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(plan_array[1][2].trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.asyncExecute(plan_array[1][2]);
  }

  if(!success)
  return 0;

/*
  // Fourth trajectory seat_1->home

  move_group.setJointValueTarget(home);

  success = (move_group.plan(plan_array[2][0]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from seat_1 to home  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 2_0 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "plan 2_0", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(plan_array[2][0].trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(plan_array[2][0]);
  }

  if(!success)
  return 0;

  // Fifth trajectory home->seat_1

  move_group.setJointValueTarget(seat_1);

  success = (move_group.plan(plan_array[0][2]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from home to seat_1  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 0_2 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "plan 0_2", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(plan_array[0][2].trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(plan_array[0][2]);
  }

  // Sixth trajectory seat_1->door_2

  move_group.setJointValueTarget(door_2);

  success = (move_group.plan(plan_array[2][1]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from seat_1 to door_2  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 2_1 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "plan 2_1", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(plan_array[2][1].trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(plan_array[2][1]);
  }

  // Seventh trajectory door-2->home

  move_group.setJointValueTarget(home);

  success = (move_group.plan(plan_array[1][0]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from door_2 to home  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 1_0 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "plan 1_0", rvt::WHITE, rvt::XLARGE);

    //std::cout << my_plan.trajectory_ << std::endl;

    visual_tools.publishTrajectoryLine(plan_array[1][0].trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(plan_array[1][0]);
  }
*/

  end_joints = move_group.getCurrentJointValues();



  //planning times
  std_msgs::Float64 num[3][3];

  num[0][1].data = plan_array[0][1].planning_time_;
  //num[0][2].data = plan_array[0][2].planning_time_;
  //num[1][0].data = plan_array[1][0].planning_time_;
  num[1][2].data = plan_array[1][2].planning_time_;
  //num[2][0].data = plan_array[2][0].planning_time_;
  //num[2][1].data = plan_array[2][1].planning_time_;


  //robot states
  moveit_msgs::RobotState r_state[3][3];
  r_state[0][1] = plan_array[0][1].start_state_;
  //r_state[0][2] = plan_array[0][2].start_state_;
  //r_state[1][0] = plan_array[1][0].start_state_;
  r_state[1][2] = plan_array[1][2].start_state_;
  //r_state[2][0] = plan_array[2][0].start_state_;
  //r_state[2][1] = plan_array[2][1].start_state_;

  //trajectories
  moveit_msgs::RobotTrajectory traj[3][3];
  traj[0][1] = plan_array[0][1].trajectory_;
  //traj[0][2] = plan_array[0][2].trajectory_;
  //traj[1][0] = plan_array[1][0].trajectory_;
  traj[1][2] = plan_array[1][2].trajectory_;
  //traj[2][0] = plan_array[2][0].trajectory_;
  //traj[2][1] = plan_array[2][1].trajectory_;


  //int row_size = sizeof(num)/sizeof(num[0]);
  //int col_size = sizeof(num[0])/sizeof(num[0][0]);

  rosbag::Bag bag;
  bag.open("move_data.bag", rosbag::bagmode::Write);


  //for(int i=0; i < row_size; i++){

  //  for(int j=0; j < col_size; j++){
  //    if(i != j){
  //      cout << i << endl;
  //      cout << j << endl;
  //      bag.write("start_names", ros::Time::now(), start_name[i]);
  //      cout << "start: " << start_name[i] << endl;

  //      bag.write("final_names", ros::Time::now(), final_name[j]);
  //      cout << "final: " << final_name[j] << endl;

        bag.write("time", ros::Time::now(), num[0][1]);
        bag.write("time", ros::Time::now(), num[1][2]);
  //      cout << "" << num[i][j] << endl;
        bag.write("state", ros::Time::now(), r_state[0][1]);
        bag.write("state", ros::Time::now(), r_state[1][2]);
  //      cout << "" << r_state[i][j] << endl;
        bag.write("trajectory", ros::Time::now(), traj[0][1]);
        bag.write("trajectory", ros::Time::now(), traj[1][2]);
        //bag.write("head", ros::Time::now(), head);
        //bag.write("positions", ros::Time::now(), positions);
        //bag.write("vels", ros::Time::now(), vels);
  //    }
  //  }
  //}


  bag.close();



  ros::spinOnce();
  visual_tools.deleteAllMarkers();
  ros::shutdown();
  return 0;
};
