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
  ros::init(argc, argv, "matrix_storer_18");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
//  moveit::planning_interface::MoveGroup::setMaxVelocityScalingFactor	(1.0);
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

  int no_p = 23; // Number of safe points

  moveit::planning_interface::MoveGroupInterface::Plan plan_array[no_p][no_p];
  moveit_msgs::RobotTrajectory trajectory;

  // Store the respective names of the safe points
  std_msgs::String start_name[no_p], final_name[no_p];

  start_name[0].data = "SP1";
  start_name[1].data = "SP2";
  start_name[2].data = "SP3";
  start_name[3].data = "SP4";
  start_name[4].data = "SP5";
  start_name[5].data = "1";
  start_name[6].data = "2";
  start_name[7].data = "3";
  start_name[8].data = "4";
  start_name[9].data = "5";
  start_name[10].data = "6";
  start_name[11].data = "7";
  start_name[12].data = "8";
  start_name[13].data = "9";
  start_name[14].data = "10";
  start_name[15].data = "11";
  start_name[16].data = "12";
  start_name[17].data = "13";
  start_name[18].data = "14";
  start_name[19].data = "15";
  start_name[20].data = "16";
  start_name[21].data = "17";
  start_name[22].data = "18";


  final_name[0].data = "SP1";
  final_name[1].data = "SP2";
  final_name[2].data = "SP3";
  final_name[3].data = "SP4";
  final_name[4].data = "SP5";
  final_name[5].data = "1";
  final_name[6].data = "2";
  final_name[7].data = "3";
  final_name[8].data = "4";
  final_name[9].data = "5";
  final_name[10].data = "6";
  final_name[11].data = "7";
  final_name[12].data = "8";
  final_name[13].data = "9";
  final_name[14].data = "10";
  final_name[15].data = "11";
  final_name[16].data = "12";
  final_name[17].data = "13";
  final_name[18].data = "14";
  final_name[19].data = "15";
  final_name[20].data = "16";
  final_name[21].data = "17";
  final_name[22].data = "18";

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

  //1
  points[5][0] = 0.0403; //0.7897;
  points[5][1] = 0.0804; //-2.3015;
  points[5][2] = 0.3008; //1.8849;
  points[5][3] = -1.2697; //0.2061;
  points[5][4] = 0.5504; //1.5631;
  points[5][5] = 3.2183; //-0.00698132;

  //2
  points[6][0] = -0.0877; //0.7897;
  points[6][1] = -0.5986; //-2.3015;
  points[6][2] = 1.1327; //1.8849;
  points[6][3] = -1.2580; //0.2061;
  points[6][4] = 0.8267; //1.5631;
  points[6][5] = -3.4512; //-0.00698132;

  //3
  points[7][0] = 0.2836; //0.7897;
  points[7][1] = -0.7794; //-2.3015;
  points[7][2] = 1.5610; //1.8849;
  points[7][3] = -1.7069; //0.2061;
  points[7][4] = 0.6126; //1.5631;
  points[7][5] = -3.2180; //-0.00698132;

  //4
  points[8][0] = -0.5265; //0.7897;
  points[8][1] = -1.1260; //-2.3015;
  points[8][2] = 1.8839; //1.8849;
  points[8][3] = -0.5080; //0.2061;
  points[8][4] = 0.1825; //1.5631;
  points[8][5] = -4.1856; //-0.00698132;

  //5
  points[9][0] = -0.0338;//-0.6342; //0.7897;
  points[9][1] = -1.8887;//-1.7154; //-2.3015;
  points[9][2] = 2.3017;//1.4734; //1.8849;
  points[9][3] = -0.3399;//0.2045; //0.2061;
  points[9][4] = 1.9519;//0.0856; //1.5631;
  points[9][5] = 1.3477;//0.5323; //-0.00698132;

  //6
  points[10][0] = 0.3611;//-0.6342; //0.7897;
  points[10][1] = -1.7908; //-2.3015;
  points[10][2] = 2.3813; //1.8849;
  points[10][3] = -0.9775; //0.2061;
  points[10][4] = 1.9277; //1.5631;
  points[10][5] = 1,1335; //-0.00698132;

  //7 robot really close
  points[11][0] = 0.0188; //0.7897;
  points[11][1] = -2.6904; //-2.3015;
  points[11][2] = 2.8251; //1.8849;
  points[11][3] = -1.2610; //0.2061;
  points[11][4] = 0.3083; //1.5631;
  points[11][5] = -3.8849; //-0.00698132;

  //8
  points[12][0] = 1.2500; //0.7897;
  points[12][1] = -1.8053; //-2.3015;
  points[12][2] = 2.4307; //1.8849;
  points[12][3] = -0.8562; //0.2061;
  points[12][4] = 1.4008; //1.5631;
  points[12][5] = 1.4861; //-0.00698132;

  //9
  points[13][0] = 1.6912; //0.7897;
  points[13][1] = -1.3969; //-2.3015;
  points[13][2] = 2.0507; //1.8849;
  points[13][3] = -0.9623; //0.2061;
  points[13][4] = 1.9786; //1.5631;
  points[13][5] = 1.2953; //-0.00698132;

  //10
  points[14][0] = 1.8046; //0.7897;
  points[14][1] = -1.2074; //-2.3015;
  points[14][2] = 1.7762; //1.8849;
  points[14][3] = -0.8223; //0.2061;
  points[14][4] = 1.9484; //1.5631;
  points[14][5] = 1.2955; //-0.00698132;

  //11
  points[15][0] = 1.9263; //0.7897;
  points[15][1] = -0.9398; //-2.3015;
  points[15][2] = 1.3789; //1.8849;
  points[15][3] = -0.7389; //0.2061;
  points[15][4] = 2.2026; //1.5631;
  points[15][5] = 1.1459; //-0.00698132;

  //12
  points[16][0] = 0.4213; //0.7897;
  points[16][1] = -1.1875; //-2.3015;
  points[16][2] = 1.7446; //1.8849;
  points[16][3] = 0.0694; //0.2061;
  points[16][4] = 1.5664; //1.5631;
  points[16][5] = -2.8658; //-0.00698132;

  //13
  points[17][0] = 1.3336; //0.7897;
  points[17][1] = -1.0896; //-2.3015;
  points[17][2] = 2.1703; //1.8849;
  points[17][3] = -1.5266; //0.2061;
  points[17][4] = 2.2319; //1.5631;
  points[17][5] = -1.2091; //-0.00698132;

  //14
  points[18][0] = 1.6425; //0.7897;
  points[18][1] = -0.7571; //-2.3015;
  points[18][2] = 1.1369; //1.8849;
  points[18][3] = 0.6108; //0.2061;
  points[18][4] = 2.1556; //1.5631;
  points[18][5] = 0.6775; //-0.00698132;

  //15
  points[19][0] = 1.6996; //0.7897;
  points[19][1] = -0.5593; //-2.3015;
  points[19][2] = 0.7698; //1.8849;
  points[19][3] = 0.8251; //0.2061;
  points[19][4] = 2.0073; //1.5631;
  points[19][5] = 0.6946; //-0.00698132;

  //16
  points[20][0] = 1.7743; //0.7897;
  points[20][1] = -0.4064; //-2.3015;
  points[20][2] = 0.4082; //1.8849;
  points[20][3] = 0.7468; //0.2061;
  points[20][4] = 1.8216; //1.5631;
  points[20][5] = 0.8436; //-0.00698132;

  //17
  points[21][0] = 1.3702; //0.7897;
  points[21][1] = -0.0635; //-2.3015;
  points[21][2] = 0.9608; //1.8849;
  points[21][3] = -1.0107; //0.2061;
  points[21][4] = 2.0699; //1.5631;
  points[21][5] = -1.2027; //-0.00698132;

  //18
  points[22][0] = 1.4599; //0.7897;
  points[22][1] = 0.1296; //-2.3015;
  points[22][2] = 0.5197; //1.8849;
  points[22][3] = -0.8360; //0.2061;
  points[22][4] = 1.9416; //1.5631;
  points[22][5] = -1.2030; //-0.00698132;

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
  bag.open("matrix18_test.bag", rosbag::bagmode::Write);


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
