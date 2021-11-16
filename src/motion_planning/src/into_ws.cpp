#include <iostream>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>


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

#include <rosbag/bag.h>

using namespace std;



geometry_msgs::Pose pose_unity;
std_msgs::String msg;
void cb_pose(geometry_msgs::PoseStamped pose_msg){

  msg.data = pose_msg.header.frame_id;

  pose_unity.position.x = -pose_msg.pose.position.y; //from unity this is the transformation we want
  pose_unity.position.y = pose_msg.pose.position.x;
  pose_unity.position.z = pose_msg.pose.position.z;


  pose_unity.orientation.x = -pose_msg.pose.orientation.y;
  pose_unity.orientation.y = pose_msg.pose.orientation.x;
  pose_unity.orientation.z = pose_msg.pose.orientation.z;
  pose_unity.orientation.w = pose_msg.pose.orientation.w;

/*
  cout << "received pose: " << endl;
  cout << "frame: " << pose_msg.header.frame_id << endl;
  cout << "pos x: " << pose_msg.pose.position.x << endl;
  cout << "pos y: " << pose_msg.pose.position.y << endl;
  cout << "pos z: " << pose_msg.pose.position.z << endl;

  cout << "rot x: " << pose_msg.pose.orientation.x << endl;
  cout << "rot y: " << pose_msg.pose.orientation.y << endl;
  cout << "rot z: " << pose_msg.pose.orientation.z << endl;
  cout << "rot w: " << pose_msg.pose.orientation.w << endl;
*/
}




std::vector<double> joint_value = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
void cb_point(const std_msgs::Float64MultiArray::ConstPtr& msg_point)
{
  joint_value[0] = msg_point->data[0];
  joint_value[1] = msg_point->data[1];
  joint_value[2] = msg_point->data[2];
  joint_value[3] = msg_point->data[3];
  joint_value[4] = msg_point->data[4];
  joint_value[5] = msg_point->data[5];
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "into_ws");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber sub_ = node_handle.subscribe("des_point", 1000, cb_point);

  ros::Subscriber sub = node_handle.subscribe("listener", 1000, cb_pose);
  ros::Publisher pub = node_handle.advertise<std_msgs::String>("/des_frame", 1000);




  std::shared_ptr<tf2_ros::Buffer> tf_buffer = std::make_shared<tf2_ros::Buffer>();

  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string ROBOT_DESCRIPTION = "robot_description";

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //planning_scene::PlanningScene *planning_scene_;
  //planning_scene_monitor::PlanningSceneMonitor *planning_scene_monitor;

  robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  //Load planning scene
  //planning_scene_ = new planning_scene::PlanningScene(kinematic_model);
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(kinematic_model));


  //const planning_scene::PlanningScenePtr ps_ptr = planning_scene_->diff();
  //Load planning scene monitor
  //planning_scene_monitor = new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION);
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(planning_scene, ROBOT_DESCRIPTION));



  // For safe acces to the scene
  //planning_scene_monitor::LockedPlanningSceneRO *locked_planning_scene_;
  //planning_scene_monitor::PlanningSceneMonitorPtr psm_ptr = locked_planning_scene_->getPlanningSceneMonitor();

  //locked_planning_scene_ = new planning_scene_monitor::LockedPlanningSceneRO(psm_ptr);





  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group.setPlannerId("RRTConnect");

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("pedestal");





  //start planning scene monitor
  planning_scene_monitor->startSceneMonitor("move_group/monitored_planning_scene");

  static const std::string JOINT_STATES = "/joint_states";
  static const std::string ATT_COL_OBJECTS = "/attached_collision_objects";
  planning_scene_monitor->startStateMonitor(JOINT_STATES, ATT_COL_OBJECTS);



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
  moveit_msgs::RobotTrajectory trajectory_;
  moveit_msgs::RobotState start_state_;

  std::vector<double> init_pos, check_pose;

  bool success;
  bool outside = true;

  //while(ros::ok()){

    // store current position (make sure to switch states when at a point)
    if(outside == true){
      init_pos = move_group.getCurrentJointValues();
    }

    //move to desired position
    move_group.setJointValueTarget(joint_value);
    move_group.allowReplanning(true);
    // slow down velocity
    move_group.setMaxVelocityScalingFactor(0.08);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO( "Completed the plan to inside ws %s", success ? "" : "FAILED");
    if(success){
      ROS_INFO("Visualizing plan as trajectory line");
      //visual_tools.publishAxisLabeled(initial, "initial");
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

      //std::cout << my_plan.trajectory_ << std::endl;

      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();

    }

    if(!success)
    return 0;
    outside = false;


    trajectory_ = my_plan.trajectory_;
    start_state_ = my_plan.start_state_;

    sleep(2.0);


    // spawn object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "chair_base";

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.20;
    primitive.dimensions[1] = 1.00;
    primitive.dimensions[2] = 0.20;

    // A pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.60;
    box_pose.position.y = 0.19;
    box_pose.position.z =  1.22;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    ROS_INFO("Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
    sleep(2.0);


    //planning_scene_monitor::PlanningSceneMonitorPtr psm_ptr = visual_tools.getPlanningSceneMonitor();
    //planning_scene_monitor::LockedPlanningSceneRO planning_scene_RO(psm_ptr);
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRO planning_scene_RO(planning_scene_monitor);



    double ps_pub_freq = planning_scene_monitor->getPlanningScenePublishingFrequency();
    cout << "ps_pub_freq: " << ps_pub_freq << endl;
    double su_freq = planning_scene_monitor->getStateUpdateFrequency();
    cout << "su_freq: " << su_freq << endl;

    planning_scene_monitor->setPlanningScenePublishingFrequency(20);
    ps_pub_freq = planning_scene_monitor->getPlanningScenePublishingFrequency();
    cout << "ps_pub_freq: " << ps_pub_freq << endl;

    bool us = planning_scene_monitor->updatesScene(planning_scene);
    cout << "us: " << us << endl;



    move_group.asyncExecute(my_plan);
    sleep(3.0);


    bool arrived = false;

    // Prints the objects present in the scene
    std::vector<std::string> objects;
    objects = planning_scene_interface.getKnownObjectNames();

    for (std::vector<std::string>::const_iterator i = objects.begin(); i != objects.end(); ++i)
    std::cout << *i << ' ' << endl;


    //collision_detection::AllowedCollisionMatrix::AllowedCollisionMatrix();

    while ( ros::ok() ){
       //process collision objects in scene
       moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
       std::map<std::string, moveit_msgs::CollisionObject> c_objects_map = planning_scene_interface_.getObjects();
       for(auto& kv : c_objects_map){
         planning_scene->processCollisionObjectMsg(kv.second);
       }
       planning_scene::PlanningScenePtr ptr = planning_scene->diff();
       planning_scene->pushDiffs( ptr);
       robot_state::RobotState robot_state = planning_scene->getCurrentState();
       //planning_scene_monitor->updateSceneWithCurrentState();

       //print distance
       ROS_INFO_STREAM("Distance to Collision: " << planning_scene->distanceToCollision(robot_state));

       ros::Duration(1.).sleep();
     }




    // Spin for path validity
/*
    bool ispathvalid;
    while(ros::ok()){
      // need to find a condition to break the loop when arriving to the desired position

      moveit_msgs::CollisionObject sphere_object;
      sphere_object.header.frame_id = move_group.getPlanningFrame();
      // The id of the object is used to identify it.
      sphere_object.id = "unity_sphere";

      // Define a box to add to the world.
      shape_msgs::SolidPrimitive sphere_shape;
      sphere_shape.type = sphere_shape.CYLINDER;
      sphere_shape.dimensions.resize(2);
      sphere_shape.dimensions[0] = 1;
      sphere_shape.dimensions[1] = 0.1;

      geometry_msgs::Pose sphere_pose;

      sphere_pose.position.x =  pose_unity.position.x; // before 1.65
      sphere_pose.position.y = pose_unity.position.y; // before 1.65
      sphere_pose.position.z =  pose_unity.position.z; // before 0.55
      sphere_pose.orientation.x = pose_unity.orientation.x;
      sphere_pose.orientation.y = pose_unity.orientation.y;
      sphere_pose.orientation.z = pose_unity.orientation.z;
      sphere_pose.orientation.w = pose_unity.orientation.w;

      tf2::Quaternion qs_orig, qs_rot, qs_new;

      // Get the original orientation of 'commanded_pose'
      tf2::convert(sphere_pose.orientation , qs_orig);

      double r=0.0, p=0, y=0.0;  // Rotate the previous pose by 180* about X
      qs_rot.setRPY(r, p, y);

      qs_new = qs_rot*qs_orig;  // Calculate the new orientation
      qs_new.normalize();

      // Stuff the new rotation back into the pose. This requires conversion into a msg type
      tf2::convert(qs_new, sphere_pose.orientation);


      sphere_object.primitives.push_back(sphere_shape);
      sphere_object.primitive_poses.push_back(sphere_pose);
      sphere_object.operation = sphere_object.ADD;

      std::vector<moveit_msgs::CollisionObject> collision_objects;
      collision_objects.push_back(sphere_object);

      // Now, let's add the collision object into the world
      //ROS_INFO("Add an object into the world");
      planning_scene_interface.addCollisionObjects(collision_objects);
      sleep(0.2);

      planning_scene_monitor->requestPlanningSceneState();
      planning_scene_monitor::LockedPlanningSceneRO planning_scene_RO(planning_scene_monitor);

      pub.publish(msg);








      ispathvalid = planning_scene_RO->isPathValid(start_state_,trajectory_,"manipulator");

      if (!ispathvalid){
          ROS_INFO("Invalid path");

          move_group.stop();

          success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
          ROS_INFO( "Completed the plan to inside ws %s", success ? "" : "FAILED");
          if(success){
            ROS_INFO("Visualizing plan as trajectory line");
            //visual_tools.publishAxisLabeled(initial, "initial");
            visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);


            visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

            visual_tools.trigger();
            move_group.asyncExecute(my_plan);

            // Update data for new trajectory
            trajectory_ = my_plan.trajectory_;
            start_state_ = my_plan.start_state_;
          }

          if(!success)
          return 0;

      }

      else{
        ROS_INFO("Path is valid");
      }
      sleep(1.0);
      // this stucks the loop????
      //planning_scene_monitor->requestPlanningSceneState();
      //planning_scene_monitor::LockedPlanningSceneRO planning_scene_RO(planning_scene_monitor);

  }
*/



    sleep(1);

    /*
    // (set up condition to go back to safe point)
    // cartesian distance? think about it...
    // if does not comply, just execute above part to constantly go to desired point

    // go to initial position
    move_group.setJointValueTarget(init_pos);

    move_group.setMaxVelocityScalingFactor(0.08);


    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO( "Completed the plan to outside ws  %s", success ? "" : "FAILED");
    if(success){
      ROS_INFO("Visualizing plan as trajectory line");
      //visual_tools.publishAxisLabeled(initial, "initial");
      visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);

      //std::cout << my_plan.trajectory_ << std::endl;

      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();
      move_group.execute(my_plan);
    }

    if(!success)
    return 0;
    // we came back outside ws
    outside = true;
    */
  //}


  ros::spinOnce();
  visual_tools.deleteAllMarkers();
  ros::shutdown();
  return 0;
};
