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
// stores slow trajectories

int main(int argc, char** argv)
{
  ros::init(argc, argv, "matrix_storer_slow28");

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

  int no_p = 28; // Number of safe points
  int no_sp = 12; // Number of safe points
  int no_ip = 16; // Number of int points

  moveit::planning_interface::MoveGroupInterface::Plan plan_array[no_p][no_p];
  moveit_msgs::RobotTrajectory trajectory;

  // Store the respective names of the safe points
  std_msgs::String start_name[no_p], final_name[no_p];

start_name[0].data ="SP1";
start_name[1].data ="SP2";
start_name[2].data ="SP3";
start_name[3].data ="SP4";
start_name[4].data ="SP5";
start_name[5].data ="SP6";
start_name[6].data ="SP7";
start_name[7].data ="SP8";
start_name[8].data ="SP9";
start_name[9].data ="SP10";
start_name[10].data ="SP11";
start_name[11].data ="SP12";
start_name[12].data ="1";
start_name[13].data ="2";
start_name[14].data ="3";
start_name[15].data ="4";
start_name[16].data ="5";
start_name[17].data ="6";
start_name[18].data ="7";
start_name[19].data ="8";
start_name[20].data ="9";
start_name[21].data ="10";
start_name[22].data ="11";
start_name[23].data ="12";
start_name[24].data ="13";
start_name[25].data ="14";
start_name[26].data ="15";
start_name[27].data ="16";



final_name[0].data ="SP1";
final_name[1].data ="SP2";
final_name[2].data ="SP3";
final_name[3].data ="SP4";
final_name[4].data ="SP5";
final_name[5].data ="SP6";
final_name[6].data ="SP7";
final_name[7].data ="SP8";
final_name[8].data ="SP9";
final_name[9].data ="SP10";
final_name[10].data ="SP11";
final_name[11].data ="SP12";
final_name[12].data ="1";
final_name[13].data ="2";
final_name[14].data ="3";
final_name[15].data ="4";
final_name[16].data ="5";
final_name[17].data ="6";
final_name[18].data ="7";
final_name[19].data ="8";
final_name[20].data ="9";
final_name[21].data ="10";
final_name[22].data ="11";
final_name[23].data ="12";
final_name[24].data ="13";
final_name[25].data ="14";
final_name[26].data ="15";
final_name[27].data ="16";


  // Define points

  //Create the matrix of points
  typedef std::vector<double> vector_p;
  typedef std::vector<vector_p> matrix_p;

  matrix_p points(no_p, vector_p(6,0.0));

  //sp1
  points[0][0]=1.8315485441;
  points[0][1]=-0.978780658977778;
  points[0][2]=1.55142319501667;
  points[0][3]=-0.471937036711111;
  points[0][4]=1.98618471811111;
  points[0][5]=0.533547160216667;
  //sp2=0;
  points[1][0]=1.56817835608333;
  points[1][1]=-1.38736224290556;
  points[1][2]=2.11254655782222;
  points[1][3]=-0.472111569638889;
  points[1][4]=1.98618471811111;
  points[1][5]=0.533547160216667;
  //sp3=0;
  points[2][0]=1.02625361533333;
  points[2][1]=-1.76051364249444;
  points[2][2]=2.48988674767778;
  points[2][3]=-0.471937036711111;
  points[2][4]=1.98618471811111;
  points[2][5]=0.533547160216667;
  //sp4=0;
  points[3][0]=-0.500036838083333;
  points[3][1]=-1.76278257055556;
  points[3][2]=2.59006864822222;
  points[3][3]=-0.701273303811111;
  points[3][4]=0.2450442306;
  points[3][5]=0.533547160216667;
  //sp5=0;
  points[4][0]=-0.6660176524;
  points[4][1]=-1.27845369597222;
  points[4][2]=2.07467291249444;
  points[4][3]=-0.8293804728;
  points[4][4]=0.0719075662444444;
  points[4][5]=0.533547160216667;
  //sp6=0;
  points[5][0]=-0.716108602672222;
  points[5][1]=-0.882613015772222;
  points[5][2]=1.47043991652778;
  points[5][3]=-1.06098566796111;
  points[5][4]=0.0752236918722222;
  points[5][5]=0.533547160216667;
  //sp7=0;
  points[6][0]=1.63467540156667;
  points[6][1]=-1.65195416141667;
  points[6][2]=1.80170341345;
  points[6][3]=-0.240157308622222;
  points[6][4]=2.23559227190556;
  points[6][5]=0.533547160216667;
  //sp8=0;
  points[7][0]=0.9330530319;
  points[7][1]=-2.12336759934444;
  points[7][2]=2.10277271386667;
  points[7][3]=-0.0150098317888889;
  points[7][4]=1.79664195854444;
  points[7][5]=0.533547160216667;
  //sp9=0;
  points[8][0]=-0.461115995188889;
  points[8][1]=-2.14134449090556;
  points[8][2]=2.11324468953333;
  points[8][3]=-0.106988684727778;
  points[8][4]=0.308399683383333;
  points[8][5]=0.533547160216667;
  //sp10=0;
  points[9][0]=-0.660083532855556;
  points[9][1]=-1.61233518681111;
  points[9][2]=1.73398463747222;
  points[9][3]=-0.0724311650277778;
  points[9][4]=0.298276773572222;
  points[9][5]=0.533547160216667;
  //sp11=0;
  points[10][0]=0.776496995683333;
  points[10][1]=-2.010619328;
  points[10][2]=1.42279242724444;
  points[10][3]=0.4649557196;
  points[10][4]=1.46031700671667;
  points[10][5]=0.533547160216667;
  //sp12=0;
  points[11][0]=-0.261101259955556;
  points[11][1]=-1.90520143962222;
  points[11][2]=1.24354711041667;
  points[11][3]=0.877900626722222;
  points[11][4]=0.898146446344445;
  points[11][5]=0.533372627288889;
  //p1=;
  points[12][0]=1.77465080964444;
  points[12][1]=-0.368264477611111;
  points[12][2]=0.952426186883333;
  points[12][3]=-1.08140602051111;
  points[12][4]=2.13453770672222;
  points[12][5]=0.533547160216667;
  //p2=;
  points[13][0]=1.57166901463889;
  points[13][1]=-0.774926199333333;
  points[13][2]=1.70064884826667;
  points[13][3]=-1.08140602051111;
  points[13][4]=2.13453770672222;
  points[13][5]=0.533547160216667;
  //p3=;
  points[14][0]=1.22557021885556;
  points[14][1]=-1.01089471768889;
  points[14][2]=2.12982531767222;
  points[14][3]=-1.08158055343889;
  points[14][4]=2.13453770672222;
  points[14][5]=0.533547160216667;
  //p4=;
  points[15][0]=0.452214815872222;
  points[15][1]=-1.16657808926667;
  points[15][2]=2.38726138614444;
  points[15][3]=-1.08175508636667;
  points[15][4]=1.31772360472222;
  points[15][5]=0.533547160216667;
  //p5=0;
  points[16][0]=-0.169296939944444;
  points[16][1]=-1.02468281898333;
  points[16][2]=2.18689758505556;
  points[16][3]=-1.18560217839444;
  points[16][4]=0.654149413311111;
  points[16][5]=0.533547160216667;
  //p6=0;
  points[17][0]=-0.390081093583333;
  points[17][1]=-0.782431115227778;
  points[17][2]=1.73136664355556;
  points[17][3]=-1.18507857961111;
  points[17][4]=0.654149413311111;
  points[17][5]=0.533547160216667;
  //p7=0;
  points[18][0]=-0.504400161277778;
  points[18][1]=-0.386590435027778;
  points[18][2]=0.958534839355556;
  points[18][3]=-1.10933128895556;
  points[18][4]=0.654149413311111;
  points[18][5]=0.533547160216667;
  //p8=0;
  points[19][0]=1.56241876946667;
  points[19][1]=-1.15017199405556;
  points[19][2]=1.56206970361111;
  points[19][3]=-0.587128769044444;
  points[19][4]=2.09020634306667;
  points[19][5]=0.533547160216667;
  //p9=0;
  points[20][0]=1.21684357246667;
  points[20][1]=-1.47375604215556;
  points[20][2]=1.86907312357222;
  points[20][3]=-0.231605195161111;
  points[20][4]=2.09055540892222;
  points[20][5]=0.533547160216667;
  //p10=0;
  points[21][0]=0.627271342433333;
  points[21][1]=-1.6776105018;
  points[21][2]=2.08636661865556;
  points[21][3]=-0.305956222394444;
  points[21][4]=1.85319062714444;
  points[21][5]=0.533547160216667;
  //p11=0;
  points[22][0]=-0.168947874088889;
  points[22][1]=-1.51826193873889;
  points[22][2]=1.98793004738889;
  points[22][3]=-0.534245291927778;
  points[22][4]=0.650658754755556;
  points[22][5]=0.533547160216667;
  //p12=0;
  points[23][0]=-0.3989822729;
  points[23][1]=-1.14144534766667;
  points[23][2]=1.54130028520556;
  points[23][3]=-0.628493072927778;
  points[23][4]=0.576133194594444;
  points[23][5]=0.533547160216667;
  //p13=0;
  points[24][0]=1.04946649472778;
  points[24][1]=-1.67778503472778;
  points[24][2]=1.53693696201111;
  points[24][3]=-0.0268780708777778;
  points[24][4]=1.49260559835556;
  points[24][5]=0.533547160216667;
  //p14=0;
  points[25][0]=0.500385903938889;
  points[25][1]=-1.78826437801111;
  points[25][2]=1.55177226087222;
  points[25][3]=0.278380019805556;
  points[25][4]=1.45909527622222;
  points[25][5]=0.533547160216667;
  //p15=0;
  points[26][0]=-0.0743510272333333;
  points[26][1]=-1.66347333465;
  points[26][2]=1.48457708367778;
  points[26][3]=0.112922804272222;
  points[26][4]=1.10165184013333;
  points[26][5]=0.533547160216667;
  //p16=0;
  points[27][0]=0.429874601116667;
  points[27][1]=-1.59645269038333;
  points[27][2]=0.808087455611111;
  points[27][3]=0.470889839144444;
  points[27][4]=1.22923541033889;
  points[27][5]=0.533547160216667;


  //fast 1 i=0..no_sp-1, j= 0...no_p-1 //from point to sp
  //fast 2 i=no_sp...no_p-1, j= 0...no_p-1 //from point to point
  //slow 1 i=0..no_sp-1, j= no_sp... no_p //from sp to points
  //slow 2 i=no_sp... no_p j=no_sp... no_p //from point to point

  //optimisation
  //fast i=0...nop j= 0...no_p-1
  //slow i=0...nop j=no_sp... no_p

  bool success;

  //Trajectories loop
  for(int i = 0; i < no_p; ++i){
    for(int j = no_sp; j < no_p; ++j){
      if(i!= j){
        // Go to starting position
        move_group.setJointValueTarget(points[i]);

        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        //ROS_INFO( "Completed the plan from initial to SP1  %s", success ? "" : "FAILED");
        ROS_INFO( "Completed the plan from %d to %d  %s",i,j, success ? "" : "FAILED");
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
    for(int j = no_sp; j < no_p; ++j){
      if(i!= j){
        num[i][j].data = plan_array[i][j].planning_time_;
        r_state[i][j] = plan_array[i][j].start_state_;
        traj[i][j] = plan_array[i][j].trajectory_;
      }
    }
  }

  // Store computed data in bagfile
  rosbag::Bag bag;
  bag.open("matrixslow12_test.bag", rosbag::bagmode::Write);

  for(int i = 0; i < no_p; ++i){
    for(int j = no_sp; j < no_p; ++j){
      if(i!= j){
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
