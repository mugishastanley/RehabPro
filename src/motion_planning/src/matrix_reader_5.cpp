#include <iostream>

#include <tf/transform_listener.h>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace std;


// Code that takes care of uploading the pre-recorded trajectories and calling the respective trajectory corresponding to the necesary motion
// from current position to desired position based on the ID received in */des_frame* topic

std::string des_frame;


//void cb_frame(const std_msgs::String::ConstPtr& msg_frame)
//{
  /*
  Callback function for the desired 'goal frame' to reached
  - Params:  msg_frame (std_msgs::String)
  */
//  des_frame = msg_frame->data;
//}


geometry_msgs::Pose pose_unity;
std_msgs::String msg;
void cb_pose(geometry_msgs::PoseStamped pose_msg){

  msg.data = pose_msg.header.frame_id;
  des_frame = msg.data;

  pose_unity.position.x = -pose_msg.pose.position.y; //from unity this is the transformation we want
  pose_unity.position.y = pose_msg.pose.position.x;
  pose_unity.position.z = pose_msg.pose.position.z;


  pose_unity.orientation.x = -pose_msg.pose.orientation.y;
  pose_unity.orientation.y = pose_msg.pose.orientation.x;
  pose_unity.orientation.z = pose_msg.pose.orientation.z;
  pose_unity.orientation.w = pose_msg.pose.orientation.w;

  cout << "received pose: " << endl;
  cout << "frame: " << pose_msg.header.frame_id << endl;
  cout << "pos x: " << pose_msg.pose.position.x << endl;
  cout << "pos y: " << pose_msg.pose.position.y << endl;
  cout << "pos z: " << pose_msg.pose.position.z << endl;

  cout << "rot x: " << pose_msg.pose.orientation.x << endl;
  cout << "rot y: " << pose_msg.pose.orientation.y << endl;
  cout << "rot z: " << pose_msg.pose.orientation.z << endl;
  cout << "rot w: " << pose_msg.pose.orientation.w << endl;

}


int main(int argc, char** argv)
{
  /*
  Main function which takes care of reading the bagfile with the trajectories.

  Executes the corresponding trajectory based on the current position and the one desired.
  */
  ros::init(argc, argv, "matrix_reader_5");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Subscriber to the topic which gives the desired position to reach
  //ros::Subscriber sub = node_handle.subscribe("des_frame", 1000, cb_frame);

  ros::Subscriber sub = node_handle.subscribe("listener", 1000, cb_pose);

  // initialization of the robot/moving group
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  //Select the planner
  move_group.setPlannerId("BiTRRT");

  // Visualization in Rviz
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

  // Define number of ellements (trajectories) stored    ex: 3 points -> 6 possible trajectories within them
  //                                                         4 points -> 12 possible trajectories within them
  //                                                         5 points -> 20 possible trajectories within them
  //
  int noe = 20;   //(no of points)^2 - (no of points)

  // Needed variables for storing rosbag data
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveGroupInterface::Plan plan_array[noe];
  moveit_msgs::RobotState state;
  moveit_msgs::RobotTrajectory trajectory;

  int i_sn = 0, i_fn = 0, i_ti = 0, i_state = 0, i_traj = 0;
  std::string start_name[noe];
  std::string final_name[noe];

  //Read the rosbag file
  rosbag::Bag bag;
  bag.open("matrix5_test.bag", rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("start_names"));
  topics.push_back(std::string("final_names"));
  topics.push_back(std::string("time"));
  topics.push_back(std::string("state"));
  topics.push_back(std::string("trajectory"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ROS_INFO("Start to read");

  //read the time stamp and write them to .txt
  foreach(rosbag::MessageInstance const m, view)
  {
      // Store starting points of each trajectory
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

      // Store finishing points of each trajectory
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

      // Store trajectories data
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

      // Store initial states of each trajectory
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

      // Store executrion time for each trajectory
      if(m.getTopic() == "time"){
        std_msgs::Float64::ConstPtr t = m.instantiate<std_msgs::Float64>();

        if(t!=NULL){
          my_plan.planning_time_ = t->data;
          plan_array[i_ti].planning_time_ = my_plan.planning_time_;
          //std::cout << "time " << i_ti << ": " << plan_array[i_ti].planning_time_ << endl;
          i_ti += 1;
          if(i_ti >= noe)
            i_ti = 0;
        }
      }
  }

  // Close the file
  bag.close();
  cout << "File loaded" << endl;

  // Go towards the home pose at initialization
  std::vector<double> SP3 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  SP3[0] = 0.0778;
  SP3[1] = -2.2259;
  SP3[2] = 1.7936;
  SP3[3] = 0.3459;
  SP3[4] = 0.7407;
  SP3[5] = 0.5323;

  move_group.setJointValueTarget(SP3);

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from initial to home  %s", success ? "" : "FAILED");
  if(success){
    ROS_INFO("Visualizing plan 1 as trajectory line");
    //visual_tools.publishAxisLabeled(initial, "initial");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    move_group.execute(my_plan);
  }

  if(!success)
  return 0;

  // Define current position after moving
  std::string init_frame = "SP3";
  std::string aux_des_frame = "SP3";

  //Constantly look to move towards the desired point
  while(ros::ok()){
    //If we are in position
    if(des_frame == init_frame){
      cout << "We are in position" << endl;
      cout << init_frame << ", " << des_frame << endl;
      cout << " " << endl;
    }

    // If not, search for the respective trajectory
    else{
      //Store the obtained desired position
      aux_des_frame = des_frame;
      for(int i = 0; i < noe; i++){
        if(start_name[i] == init_frame && final_name[i] == aux_des_frame){

          cout << start_name[i] << " --> " << final_name[i] << endl;
          cout << "index: " << i << endl;
          cout << " " <<endl;

          //move to desired position
          move_group.execute(plan_array[i]);
          //refresh current position
          init_frame = aux_des_frame;

          // publish last reached position
          // pub String
          // pub current joints?

        }
      }
    }
  }

  ros::spinOnce();
  visual_tools.deleteAllMarkers();
  ros::shutdown();
  return 0;
};
