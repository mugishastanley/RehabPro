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
  ros::init(argc, argv, "matrix_storer_28");

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
  points[0][0]=1.80868470384184;
  points[0][1]=-0.896052037973948;
  points[0][2]=1.48719505562447;
  points[0][3]=-0.714886861616924;
  points[0][4]=2.17764730771347;
  points[0][5]=0.532674487708704;


  //sp2
  points[1][0]=1.54810704651907;
  points[1][1]=-1.28229340144032;
  points[1][2]=2.05041280524307;
  points[1][3]=-0.715061394542124;
  points[1][4]=2.17747277478827;
  points[1][5]=0.532674487708704;
  //sp3
  points[2][0]=1.05138634140145;
  points[2][1]=-1.57987203890537;
  points[2][2]=2.39127560815759;
  points[2][3]=-0.700051562974972;
  points[2][4]=2.17764730771347;
  points[2][5]=0.532674487708704;
  //sp4
  points[3][0]=-0.139800873084755;
  points[3][1]=-1.68092660259585;
  points[3][2]=2.43019645047706;
  points[3][3]=-0.442440965380592;
  points[3][4]=0.821002880138187;
  points[3][5]=0.532849020633904;
  //sp5
  points[4][0]=-0.526391302401524;
  points[4][1]=-1.25838239068799;
  points[4][2]=2.06228104415664;
  points[4][3]=-0.915774258521485;
  points[4][4]=0.282917871748299;
  points[4][5]=0.532849020633904;
  //sp6
  points[5][0]=-0.622558944186418;
  points[5][1]=-0.863414380961652;
  points[5][2]=1.43553330976543;
  points[5][3]=-0.915948791446685;
  points[5][4]=0.282917871748299;
  points[5][5]=0.532849020633904;
  //sp7
  points[6][0]=1.6011650557797;
  points[6][1]=-1.51337499440438;
  points[6][2]=1.62856672503602;
  points[6][3]=-0.132470490226378;
  points[6][4]=2.42513499564628;
  points[6][5]=0.454483737219353;
  //sp8
  points[7][0]=0.806342114421433;
  points[7][1]=-2.03540297367592;
  points[7][2]=2.06193197830624;
  points[7][3]=-0.089535390627315;
  points[7][4]=1.54444185508988;
  points[7][5]=0.454483737219353;
  //sp9
  points[8][0]=-0.255690735417186;
  points[8][1]=-1.98827908387207;
  points[8][2]=2.00835037027001;
  points[8][3]=-0.0291469985083072;
  points[8][4]=0.469493568786506;
  points[8][5]=0.454483737219353;
  //sp10
  points[9][0]=-0.552047642405843;
  points[9][1]=-1.52960655644793;
  points[9][2]=1.65561932844193;
  points[9][3]=-0.178198116628633;
  points[9][4]=0.273842159637928;
  points[9][5]=0.454483737219353;
  //sp11
  points[10][0]=1.04021623418869;
  points[10][1]=-1.9161969857647;
  points[10][2]=1.48998758242766;
  points[10][3]=-0.0514872129338361;
  points[10][4]=2.10713600593289;
  points[10][5]=0.533023553559103;
  //sp12
  points[11][0]=0.272794962086732;
  points[11][1]=-1.94272599039502;
  points[11][2]=1.52192710773916;
  points[11][3]=-0.0511381470834372;
  points[11][4]=1.91392805773711;
  points[11][5]=0.533023553559103;
  //p1
  points[12][0]=1.73468274355728;
  points[12][1]=-0.305258086173828;
  points[12][2]=0.696735437396182;
  points[12][3]=-0.632158255072388;
  points[12][4]=2.17293491873308;
  points[12][5]=0.532499954783505;
  //p2
  points[13][0]=1.52140350896356;
  points[13][1]=-0.742113997948038;
  points[13][2]=1.52576683209354;
  points[13][3]=-0.732340154136869;
  points[13][4]=2.13104701668522;
  points[13][5]=0.532499954783505;


  //p3
  points[14][0]=1.19048908278541;
  points[14][1]=-0.978606111593285;
  points[14][2]=1.96960406087573;
  points[14][3]=-0.840026968984926;
  points[14][4]=2.13191968131121;
  points[14][5]=0.532499954783505;
  //p4

/***
  points[15][0]=0.5770;
  points[15][1]=-1.1041;
  points[15][2]=2.0019;
  points[15][3]=-0.2180;
  points[15][4]=1.6132;
  points[15][5]=0.5327;

  points[15][0]=0.5287;
  points[15][1]=-1.0879;
  points[15][2]=2.2560;
  points[15][3]=-1.1023;
  points[15][4]=1.4476;
  points[15][5]=0.5327;
**/


  points[15][0]=0.5145;
  points[15][1]=-1.1153;
  points[15][2]=2.2237;
  points[15][3]=-0.8992;
  points[15][4]=1.4476;
  points[15][5]=0.5327;


/**
  points[15][0]=0.4775;
  points[15][1]=-1.083;
  points[15][2]=2.2571;
  points[15][3]=-1.1252;
  points[15][4]=1.2551;
  points[15][5]=0.5327;

  points[15][0]=-0.0912807198793095;
  points[15][1]=-0.977733446967288;
  points[15][2]=2.00677957394321;
  points[15][3]=-0.886976325863577;
  points[15][4]=0.66601764256108;
  points[15][5]=0.532499954783505;//0.881565805182394;
  **/

  //p5
  points[16][0]=-0.0912807198793095;
  points[16][1]=-0.977733446967288;
  points[16][2]=2.00677957394321;
  points[16][3]=-0.886976325863577;
  points[16][4]=0.66601764256108;
  points[16][5]=0.532499954783505;//0.881565805182394;
  //p6
  points[17][0]=-0.350985712576083;
  points[17][1]=-0.705287550730955;
  points[17][2]=1.48824225317566;
  points[17][3]=-0.897448301375543;
  points[17][4]=0.391651884147553;
  points[17][5]=0.532499954783505;
  //p7
  points[18][0]=-0.4712388980385;
  points[18][1]=-0.258308729295178;
  points[18][2]=0.688532389911808;
  points[18][3]=-1.15750235992272;
  points[18][4]=0.38955748904516;
  points[18][5]=0.532499954783505;
  //p8
  points[19][0]=1.53170095155032;
  points[19][1]=-1.05156087432665;
  points[19][2]=1.36118228363047;
  points[19][3]=-0.338419341961723;
  points[19][4]=2.19352980390662;
  points[19][5]=0.532499954783505;
  //p9
  points[20][0]=1.20008839367138;
  points[20][1]=-1.37881010907561;
  points[20][2]=1.821949206157;
  points[20][3]=-0.471937029739298;
  points[20][4]=2.19422793560742;
  points[20][5]=0.532499954783505;
  //p10
  points[21][0]=0.554665636283834;
  points[21][1]=-1.58004657183057;
  points[21][2]=1.99334053870285;
  points[21][3]=0.295484242362659;
  points[21][4]=1.56538580611382;
  points[21][5]=0.532499954783505;
  //p11
  points[22][0]=-0.0352556508902878;
  points[22][1]=-1.4451326206514;
  points[22][2]=1.90240888467394;
  points[22][3]=-0.489041256408843;
  points[22][4]=0.952949771588967;
  points[22][5]=0.532499954783505;
  //p12
  points[23][0]=-0.335975881008931;
  points[23][1]=-1.066745238819;
  points[23][2]=1.43500971098983;
  points[23][3]=-0.604756585816075;
  points[23][4]=0.588525023772527;
  points[23][5]=0.532499954783505;
  //p13
  points[24][0]=1.13481307964679;
  points[24][1]=-1.52454510161715;
  points[24][2]=1.36554560676045;
  points[24][3]=0.0453785605518556;
  points[24][4]=1.9174187162411;
  points[24][5]=0.532499954783505;
  //p14
  points[25][0]=0.649262481741933;
  points[25][1]=-1.60395758258289;
  points[25][2]=1.36537107383525;
  points[25][3]=0.35814156250926;
  points[25][4]=1.88111586779961;
  points[25][5]=0.532499954783505;
  //p15
  points[26][0]=-0.0680678408277833;
  points[26][1]=-1.52943202352273;
  points[26][2]=1.35472456539809;
  points[26][3]=0.109606677025251;
  points[26][4]=0.845961088441707;
  points[26][5]=0.532499954783505;
  //p16
  points[27][0]=0.491484717361636;
  points[27][1]=-1.52210164066435;
  points[27][2]=0.782954702444708;
  points[27][3]=0.35814156250926;
  points[27][4]=1.35891335560287;
  points[27][5]=0.532499954783505;


  bool success;

  //Trajectories loop
  for(int i = 0; i < no_p; ++i){
    for(int j = 0; j < no_p; ++j){
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
    for(int j = 0; j < no_p; ++j){
      if(i!= j){
        num[i][j].data = plan_array[i][j].planning_time_;
        //num[i][j].data = plan_array[i][j].execution_time_;
        r_state[i][j] = plan_array[i][j].start_state_;
        traj[i][j] = plan_array[i][j].trajectory_;
      }
    }
  }

  // Store computed data in bagfile
  rosbag::Bag bag;
  bag.open("matrix28_test.bag", rosbag::bagmode::Write);


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
