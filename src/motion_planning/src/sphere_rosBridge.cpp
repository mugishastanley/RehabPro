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

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

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
  ros::init(argc, argv, "sphere_rosBridge");

  ros::NodeHandle node_handle;

  ros::Publisher pub = node_handle.advertise<std_msgs::String>("/des_frame", 1000);

  ros::Subscriber sub = node_handle.subscribe("listener", 1000, cb_pose);

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




  while (ros::ok()){
    // spawn object
    /*
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
    */
    pub.publish(msg);
  }


  ros::spinOnce();
  visual_tools.deleteAllMarkers();
  ros::shutdown();
  return 0;
}
