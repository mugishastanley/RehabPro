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

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace std;

//short version of matrix_reader_5 just for the trajcetories from home to each one of the safe positoins
// the trajectories are recorded in matrix_storer

std::string des_frame;

void cb_frame(const std_msgs::String::ConstPtr& msg_frame)
{
  des_frame = msg_frame->data;
  cout << "msg: " << des_frame << endl;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "matrix_reader");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber sub = node_handle.subscribe("des_frame", 1000, cb_frame);

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  //Select out planner
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
  moveit::planning_interface::MoveGroupInterface::Plan plan_array[6];
  moveit_msgs::RobotState state;
  moveit_msgs::RobotTrajectory trajectory;



  // Go towards the home pose at initialization
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

  //Read the rosbag file
  rosbag::Bag bag;
  bag.open("matrix_test.bag", rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("start_names"));
  topics.push_back(std::string("final_names"));
  topics.push_back(std::string("time"));
  topics.push_back(std::string("state"));
  topics.push_back(std::string("trajectory"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  int i_sn = 0, i_fn = 0, i_t = 0, i_state = 0, i_traj = 0;
  int noe = 6;
  std::string start_name[6];
  std::string final_name[6];

  ROS_INFO("Start to read");

  //read the time stamp and write them to .txt
  foreach(rosbag::MessageInstance const m, view)
  {

      if(m.getTopic() == "start_names"){
        std_msgs::String::ConstPtr sn = m.instantiate<std_msgs::String>();
        if (sn != NULL){
        //std::cout << "start_name " << i_sn << ": " << sn->data << std::endl;
        start_name[i_sn] = sn->data;

        i_sn += 1;
        if(i_sn >= noe)
          i_sn = 0;
        }
      }

      if(m.getTopic() == "final_names"){
        std_msgs::String::ConstPtr fn = m.instantiate<std_msgs::String>();
        if (fn != NULL){
        //std::cout << "final_name " << i_fn << ": " << fn->data << std::endl;
        final_name[i_fn] = fn->data;

        i_fn += 1;
        if(i_fn >= noe)
          i_fn = 0;
        }
      }

      if(m.getTopic() == "trajectory"){
        moveit_msgs::RobotTrajectory::ConstPtr traj = m.instantiate<moveit_msgs::RobotTrajectory>();
        if (traj != NULL){
          //std::cout << "traj" << std::endl;
          trajectory.joint_trajectory = traj->joint_trajectory;
          trajectory.multi_dof_joint_trajectory = traj->multi_dof_joint_trajectory;

          my_plan.trajectory_ = trajectory;
          plan_array[i_traj].trajectory_ = my_plan.trajectory_;

          i_traj += 1;
          if(i_traj >= noe)
            i_traj = 0;
          }
      }

      if(m.getTopic() == "state"){
        moveit_msgs::RobotState::ConstPtr s = m.instantiate<moveit_msgs::RobotState>();
        if (s != NULL){
          //std::cout << s->data << std::endl;
          //std::cout << s->joint_state << std::endl;
          //std::cout << s->multi_dof_joint_state << std::endl;
          //std::cout << s->attached_collision_objects << std::endl;

          state.joint_state = s->joint_state;
          state.multi_dof_joint_state = s->multi_dof_joint_state;

          my_plan.start_state_ = state;
          plan_array[i_state].start_state_ = my_plan.start_state_;

          i_state += 1;
          if(i_state >= noe)
            i_state = 0;
        }
      }

      if(m.getTopic() == "time"){
        std_msgs::Float64::ConstPtr t = m.instantiate<std_msgs::Float64>();

        if(t!=NULL){
          my_plan.planning_time_ = t->data;

          plan_array[i_t].planning_time_ = my_plan.planning_time_;
          //cout << plan_array[i_t].planning_time_ << endl;
          i_t += 1;
          if(i_t >= noe)
            i_t = 0;
        }
      }
  }

  bag.close();

  cout << "File loaded" << endl;

  std::string init_frame = "home";

  cout << start_name[0] << endl;
  cout << final_name[0] << endl;
  //move_group.execute(plan_array[0]);

  cout << start_name[1] << endl;
  cout << final_name[1] << endl;
  //move_group.execute(plan_array[1]);

  cout << start_name[2] << endl;
  cout << final_name[2] << endl;
  //move_group.execute(plan_array[2]);

  cout << start_name[3] << endl;
  cout << final_name[3] << endl;
  //move_group.execute(plan_array[3]);

  cout << start_name[4] << endl;
  cout << final_name[4] << endl;
  //move_group.execute(plan_array[4]);

  cout << start_name[5] << endl;
  cout << final_name[5] << endl;
  //move_group.execute(plan_array[5]);

  //Constantly look to move towards the desired point
  while(ros::ok()){
    //If we are in position
    if(des_frame == init_frame){
      cout << "We are in position" << endl;
    }

    // If not, search for the rspective trajectory
    for(int i = 0; i < noe; i++){

      if(start_name[i] == init_frame && final_name[i] == des_frame){

        cout << start_name[i] << endl;
        cout << final_name[i] << endl;
        cout << "index: " << i << endl;

        move_group.execute(plan_array[i]);
        init_frame = des_frame;

      }
    }

  }

  ros::spinOnce();
  visual_tools.deleteAllMarkers();
  ros::shutdown();
  return 0;
};
