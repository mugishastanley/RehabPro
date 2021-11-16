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

//Spawns different object types linked to the mannequin frames. Constantly updates MoveIt planning scene to track the moving objects (mannequin) in real time

void spawn_sphere(std::string obj_id, std::string des_frame, std::string ref_frame, double radius, tf::TransformListener* listener, moveit::planning_interface::MoveGroupInterface* move_group, moveit::planning_interface::PlanningSceneInterface* planning_scene_interface){

    tf::StampedTransform transform;
    try{
      listener->lookupTransform(ref_frame, des_frame,
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // spawn object
    moveit_msgs::CollisionObject sphere_object;
    sphere_object.header.frame_id = move_group->getPlanningFrame();
    // The id of the object is used to identify it.
    sphere_object.id = obj_id;

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive sphere_shape;
    sphere_shape.type = sphere_shape.SPHERE;
    sphere_shape.dimensions.resize(1);
    sphere_shape.dimensions[0] = radius;

    geometry_msgs::Pose sphere_pose;

    sphere_pose.position.x =  transform.getOrigin().x(); // before 1.65
    sphere_pose.position.y = transform.getOrigin().y(); // before 1.65
    sphere_pose.position.z =  transform.getOrigin().z(); // before 0.55
    sphere_pose.orientation.w = 1.0;

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
    ROS_INFO("Add an object into the world");
    planning_scene_interface->addCollisionObjects(collision_objects);
    sleep(0.2);

}

void spawn_cylinder(std::string obj_id, std::string des_frame, std::string ref_frame, double height, double radius, tf::TransformListener* listener, moveit::planning_interface::MoveGroupInterface* move_group, moveit::planning_interface::PlanningSceneInterface* planning_scene_interface){

  tf::StampedTransform transform;
  try{
    listener->lookupTransform(ref_frame, des_frame,
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // spawn object
  moveit_msgs::CollisionObject cylinder_object;
  cylinder_object.header.frame_id = move_group->getPlanningFrame();
  // The id of the object is used to identify it.
  cylinder_object.id = obj_id;

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive cylinder_shape;
  cylinder_shape.type = cylinder_shape.CYLINDER;
  cylinder_shape.dimensions.resize(2);
  cylinder_shape.dimensions[0] = height;
  cylinder_shape.dimensions[1] = radius;


  geometry_msgs::Pose cylinder_pose;

  cylinder_pose.position.x =  transform.getOrigin().x(); // before 1.65
  cylinder_pose.position.y = transform.getOrigin().y(); // before 1.65
  cylinder_pose.position.z =  transform.getOrigin().z(); // before 0.55
  cylinder_pose.orientation.x = transform.getRotation().x();
  cylinder_pose.orientation.y = transform.getRotation().y();
  cylinder_pose.orientation.z = transform.getRotation().z();
  cylinder_pose.orientation.w = transform.getRotation().w();

  cylinder_object.primitives.push_back(cylinder_shape);
  cylinder_object.primitive_poses.push_back(cylinder_pose);
  cylinder_object.operation = cylinder_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(cylinder_object);

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");
  planning_scene_interface->addCollisionObjects(collision_objects);
  sleep(0.2);
}


void spawn_box(std::string obj_id, std::string des_frame, std::string ref_frame, double width, double length, double height, tf::TransformListener* listener, moveit::planning_interface::MoveGroupInterface* move_group, moveit::planning_interface::PlanningSceneInterface* planning_scene_interface){

  tf::StampedTransform transform;
  try{
    listener->lookupTransform(ref_frame, des_frame,
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // spawn object
  moveit_msgs::CollisionObject box_object;
  box_object.header.frame_id = move_group->getPlanningFrame();
  // The id of the object is used to identify it.
  box_object.id = obj_id;

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive box_shape;
  box_shape.type = box_shape.BOX;
  box_shape.dimensions.resize(3);
  box_shape.dimensions[0] = width;
  box_shape.dimensions[1] = height;
  box_shape.dimensions[2] = length;

  geometry_msgs::Pose box_pose;

  box_pose.position.x =  transform.getOrigin().x(); // before 1.65
  box_pose.position.y = transform.getOrigin().y(); // before 1.65
  box_pose.position.z =  transform.getOrigin().z(); // before 0.55
  box_pose.orientation.x = transform.getRotation().x();
  box_pose.orientation.y = transform.getRotation().y();
  box_pose.orientation.z = transform.getRotation().z();
  box_pose.orientation.w = transform.getRotation().w();

  box_object.primitives.push_back(box_shape);
  box_object.primitive_poses.push_back(box_pose);
  box_object.operation = box_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(box_object);

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");
  planning_scene_interface->addCollisionObjects(collision_objects);
  sleep(0.2);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obj_tf");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

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


  while(ros::ok()){

    spawn_sphere("left_wrist", "/left_wrist", "/world", 0.051, &listener, &move_group, &planning_scene_interface);
    spawn_sphere("right_wrist", "/right_wrist", "/world", 0.051, &listener, &move_group, &planning_scene_interface);
    //spawn_sphere("head", "/body_head", "/world", 0.117, &listener, &move_group, &planning_scene_interface);
    spawn_cylinder("left_humerus", "/left_humerus", "/world", 0.3384, 0.051, &listener, &move_group, &planning_scene_interface);
    spawn_cylinder("right_humerus", "/right_humerus", "/world", 0.3384, 0.051, &listener, &move_group, &planning_scene_interface);
    spawn_cylinder("left_fore_arm", "/left_fore_arm", "/world", 0.261, 0.051, &listener, &move_group, &planning_scene_interface);
    spawn_cylinder("right_fore_arm", "/right_fore_arm", "/world", 0.261, 0.051, &listener, &move_group, &planning_scene_interface);
    spawn_box("left_palm", "/left_palm", "/world", 0.09, 0.1944, 0.03996, &listener, &move_group, &planning_scene_interface);
    spawn_box("right_palm", "/right_palm", "/world", 0.09, 0.1944, 0.03996, &listener, &move_group, &planning_scene_interface);

    // UPDATE SCENE
    //planning_scene_monitor::PlanningSceneMonitorPtr psm_ptr = visual_tools.getPlanningSceneMonitor();
    //planning_scene_monitor::LockedPlanningSceneRO planning_scene_RO(psm_ptr);
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRO planning_scene_RO(planning_scene_monitor);



  }


  ros::spinOnce();
  visual_tools.deleteAllMarkers();
  ros::shutdown();
  return 0;
};
