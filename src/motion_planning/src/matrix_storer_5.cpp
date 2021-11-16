#include <iostream>
#include <tf/transform_listener.h>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <rosbag/bag.h>

using namespace std;

// Plans and performs all the possible trajectories between the diferent sagepoints, and stores them into a bag file for them to be loaded in matrix_reader_5
// intended for off-line trajectories computations

int main(int argc, char** argv)
{
  ros::init(argc, argv, "matrix_storer_5");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";

  
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Select the preferred planner to use
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

  int no_p = 5; // Number of points for the trajectories
  moveit::planning_interface::MoveGroupInterface::Plan plan_array[no_p][no_p];
  moveit_msgs::RobotTrajectory trajectory;

  // Store the respective names of the safe points
  std_msgs::String start_name[no_p], final_name[no_p];

  start_name[0].data = "SP1";
  start_name[1].data = "SP2";
  start_name[2].data = "SP3";
  start_name[3].data = "SP4";
  start_name[4].data = "SP5";


  final_name[0].data = "SP1";
  final_name[1].data = "SP2";
  final_name[2].data = "SP3";
  final_name[3].data = "SP4";
  final_name[4].data = "SP5";

  // Define points

  //Create the matrix of points
  typedef std::vector<double> vector_p;
  typedef std::vector<vector_p> matrix_p;

  matrix_p points(no_p, vector_p(6,0.0));

  //SP1
  points[0][0] = -0.6342; //0.7897;
  points[0][1] = -1.7154; //-2.3015;
  points[0][2] = 1.4734; //1.8849;
  points[0][3] = 0.2045; //0.2061;
  points[0][4] = 0.0856; //1.5631;
  points[0][5] = 0.5323; //-0.00698132;

  //SP2
  points[1][0] = -0.5773;  //-0.3915; //-0.7049;
  points[1][1] = -2.0228;  //-0.4053; //-1.1981;
  points[1][2] = 1.6968;  //0.7602; //1.6260;
  points[1][3] = 0.2043;  //-1.9794; //-1.4598;
  points[1][4] = 0.1391;  //1.6488; //0.5285;
  points[1][5] = 0.5323;  //1.2789; //1.0093;

  //SP3
  points[2][0] = 0.0778;  //-0.4245; // -0.7052;
  points[2][1] = -2.2259;  //-0.9263; // -1.6972;
  points[2][2] = 1.7936;  //1.7180; // 1.9266;
  points[2][3] = 0.3459;  //-2.2628; // -0.2927;
  points[2][4] = 0.7407;  //1.6489; // -0.0038;
  points[2][5] = 0.5323;  //1.2789; // 1.0093;

  //SP4
  points[3][0] = 1.2761;  //1.8596; // 1.5790;
  points[3][1] = -1.9106;  //-0.7896; // -1.6432;
  points[3][2] = 1.6156;  //1.6217; // 1.8536;
  points[3][3] = 0.2946;  //-2.1693; // 0.3076;
  points[3][4] = 1.9865;  //1.6487; // 1.4263;
  points[3][5] = 0.5323;  //1.2789;// 1.5521;

  //SP5
  points[4][0] = 1.6175;  //1.8722; // 2.0299;
  points[4][1] = -1.6168;  //-0.2958; // -0.9607;
  points[4][2] = 1.3721;  //0.8744; // 1.3326;
  points[4][3] = 0.2701;  //-2.1691; // -1.471;
  points[4][4] = 2.0832;  //1.6487; // 1.9863;
  points[4][5] = 0.5323;  //1.2789; // 2.1216;

  bool success;

  //Trajectories loop
  for(int i = 0; i < no_p; ++i){
    for(int j = 0; j < no_p; ++j){
      if(i!= j){
        // Go to starting position
        move_group.setJointValueTarget(points[i]);

        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO( "Completed the plan from initial to SP1  %s", success ? "" : "FAILED");
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

        //Store plan
        move_group.setJointValueTarget(points[j]);

        success = (move_group.plan(plan_array[i][j]) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO( "Completed the plan from home to door_1  %s", success ? "" : "FAILED");
        if(success){
          ROS_INFO("Visualizing plan 0_1 as trajectory line");
          //visual_tools.publishAxisLabeled(initial, "initial");
          visual_tools.publishText(text_pose, "plan 0_1", rvt::WHITE, rvt::XLARGE);

          //std::cout << my_plan.trajectory_ << std::endl;

          visual_tools.publishTrajectoryLine(plan_array[i][j].trajectory_, joint_model_group);
          visual_tools.trigger();
          move_group.execute(plan_array[i][j]);
        }

        if(!success)
        return 0;

        cout << "STORING PLAN" << endl;
        cout << " " << endl;
      }
    }
  }


  //planning times
  std_msgs::Float64 num[no_p][no_p];
  //robot states
  moveit_msgs::RobotState r_state[no_p][no_p];
  //trajectories
  moveit_msgs::RobotTrajectory traj[no_p][no_p];

  for(int i = 0; i < no_p; ++i){
    for(int j = 0; j < no_p; ++j){
      if(i!= j){
        num[i][j].data = plan_array[i][j].planning_time_;
        r_state[i][j] = plan_array[i][j].start_state_;
        traj[i][j] = plan_array[i][j].trajectory_;
      }
    }
  }

  // Store computed data in bagfile
  rosbag::Bag bag;
  bag.open("matrix5_test.bag", rosbag::bagmode::Write);


  for(int i=0; i < no_p; i++){

    for(int j=0; j < no_p; j++){
      if(i != j){
        cout << i << endl;
        cout << j << endl;
        bag.write("start_names", ros::Time::now(), start_name[i]);
        cout << "start: " << start_name[i] << endl;

        bag.write("final_names", ros::Time::now(), final_name[j]);
        cout << "final: " << final_name[j] << endl;

        bag.write("time", ros::Time::now(), num[i][j]);
        cout << "" << num[i][j] << endl;
        bag.write("state", ros::Time::now(), r_state[i][j]);
        cout << "" << r_state[i][j] << endl;
        bag.write("trajectory", ros::Time::now(), traj[i][j]);
      }
    }
  }

  bag.close();

  ros::spinOnce();
  visual_tools.deleteAllMarkers();
  ros::shutdown();
  return 0;
};
