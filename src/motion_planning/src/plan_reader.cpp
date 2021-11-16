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

//simplest version of read and execute pre recorded trajectories, just for 2 trajectories and to execute them, not reading any id information

geometry_msgs::Pose desired, initial;
std::vector<double> joint_value = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_reader");

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
  moveit::planning_interface::MoveGroupInterface::Plan plan_array[2];
  moveit_msgs::RobotState state;
  moveit_msgs::RobotTrajectory trajectory;


  //load the plan from the rosbag file
  /*
  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Read);
  //double ttt;
  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    //ttt = m.instantiate<double>();

    //std::cout << "time: " << ttt << std::endl;
    //move_group.execute(load_plan);

    double::ConstPtr ttt = m.instantiate<double>();
      //if (ttt != nullptr)
      //  std::cout << ttt->data << std::endl;
  }

  bag.close();
*/


  //Topics name in the rosbag
  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(std::string("time"));
  topics.push_back(std::string("state"));
  topics.push_back(std::string("trajectory"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  int i = 0;
  int noe = 2;
  ROS_INFO("Start to read");
  //read the time stamp and write them to .txt
  foreach(rosbag::MessageInstance const m, view)
  {
      moveit_msgs::RobotTrajectory::ConstPtr traj = m.instantiate<moveit_msgs::RobotTrajectory>();
      if (traj != NULL){
        std::cout << "traj" << std::endl;
        trajectory.joint_trajectory = traj->joint_trajectory;
        trajectory.multi_dof_joint_trajectory = traj->multi_dof_joint_trajectory;

        my_plan.trajectory_ = trajectory;
        plan_array[i].trajectory_ = my_plan.trajectory_;

        cout << "executing" << endl;
        move_group.execute(my_plan);
        i += 1;
        if(i >= noe)
          i = 0;
      }


      moveit_msgs::RobotState::ConstPtr s = m.instantiate<moveit_msgs::RobotState>();
      if (s != NULL){
        //std::cout << s->data << std::endl;
        //std::cout << s->joint_state << std::endl;
        //std::cout << s->multi_dof_joint_state << std::endl;
        //std::cout << s->attached_collision_objects << std::endl;

        state.joint_state = s->joint_state;
        state.multi_dof_joint_state = s->multi_dof_joint_state;
        cout << "state"  << endl;
        my_plan.start_state_ = state;
        plan_array[i].start_state_ = my_plan.start_state_;

        i += 1;
        if(i >= noe)
          i = 0;
      }

      std_msgs::Float64::ConstPtr t = m.instantiate<std_msgs::Float64>();

      if(t!=NULL){
        //
        std::cout << "t: " << t->data << std::endl;
        my_plan.planning_time_ = t->data;

        plan_array[i].planning_time_ = my_plan.planning_time_;
        cout << plan_array[i].planning_time_ << endl;
        i += 1;
        if(i >= noe)
          i = 0;
      }

  }

  bag.close();

  cout << plan_array[0].planning_time_ << endl;
  cout << plan_array[0].start_state_ << endl;
  //cout << plan_array[0].trajectory_ << endl;
  cout << "" << endl;
  cout << plan_array[1].planning_time_ << endl;
  cout << plan_array[1].start_state_ << endl;
  //cout << plan_array[1].trajectory_ << endl;

  ros::spinOnce();
  visual_tools.deleteAllMarkers();
  ros::shutdown();
  return 0;
};
