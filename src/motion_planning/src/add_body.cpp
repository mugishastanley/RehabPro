#include <iostream>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/Pose.h>
#include <geometric_shapes/shape_operations.h>


using namespace std;


// This code is meant to spawn the desired static objects in the scene, being the plane, the spherical plane and themesh corresponding for car the environment

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_body");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("manipulator");

  moveit::planning_interface::PlanningSceneInterface current_scene;
  sleep(2.0);


  // Add the PLANE
/*
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "plane";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive plane_shape;
  plane_shape.type = plane_shape.BOX;
  plane_shape.dimensions.resize(3);
  plane_shape.dimensions[0] = 2.0;
  plane_shape.dimensions[1] = 0.02;
  plane_shape.dimensions[2] = 1.4;

  // A pose for the box (specified relative to frame_id)
  geometry_msgs::Pose plane_pose;

  plane_pose.position.x =  0.2687;
  plane_pose.position.y = 0.2687;
  plane_pose.position.z =  1.0;
  plane_pose.orientation.w = 1.0;

  tf2::Quaternion q_orig, q_rot, q_new;

  // Get the original orientation of 'commanded_pose'
  tf2::convert(plane_pose.orientation , q_orig);

  double r=0.0, p=0, y=-0.785;  // Rotate the previous pose by 180* about X
  q_rot.setRPY(r, p, y);

  q_new = q_rot*q_orig;  // Calculate the new orientation
  q_new.normalize();

  // Stuff the new rotation back into the pose. This requires conversion into a msg type
  tf2::convert(q_new, plane_pose.orientation);

  collision_object.primitives.push_back(plane_shape);
  collision_object.primitive_poses.push_back(plane_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");
  current_scene.addCollisionObjects(collision_objects);
  sleep(2.0);
*/




  // Add the Sphere workspace

  moveit_msgs::CollisionObject sphere_object;
  sphere_object.header.frame_id = group.getPlanningFrame();
  // The id of the object is used to identify it.
  //sphere_object.id = "sphere";
  sphere_object.id = "box";
  // Define a box to add to the world.
  shape_msgs::SolidPrimitive sphere_shape;
  sphere_shape.type = sphere_shape.BOX;
  sphere_shape.dimensions.resize(3);
  sphere_shape.dimensions[0] = 2.0;
  sphere_shape.dimensions[1] = 0.0;
  sphere_shape.dimensions[2] = 2.0;

  // A pose for the box (specified relative to frame_id)

  geometry_msgs::Pose sphere_pose;

  sphere_pose.position.x =  0.575; // before 1.65
  sphere_pose.position.y = 0.575; // before 1.65
  sphere_pose.position.z =  1.0; // before 0.55
  sphere_pose.orientation.x = 0.0;
  sphere_pose.orientation.y = 0.0;
  sphere_pose.orientation.z = 0.0;
  sphere_pose.orientation.w =   1;




  tf2::Quaternion qs_orig, qs_rot, qs_new;

  // Get the original orientation of 'commanded_pose'
  tf2::convert(sphere_pose.orientation , qs_orig);

  double r=0.0, p=0, y=-0.785;  // Rotate the previous pose by 180* about X
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
  current_scene.addCollisionObjects(collision_objects);
  sleep(2.0);





// ADD MESH BODIES
  /*
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "chair_base";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.60;
  primitive.dimensions[1] = 1.00;
  primitive.dimensions[2] = 0.30;

  // A pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.0;
  box_pose.position.y = 0.78;
  box_pose.position.z =  0.15;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  //std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");
  current_scene.addCollisionObjects(collision_objects);
  sleep(2.0);



  // MESH obstacle
  moveit_msgs::CollisionObject co;
  co.header.frame_id = group.getPlanningFrame();

  // Id
  co.id = "chair";

  // Obstacle definition
  //Eigen::Vector3d b(0.001, 0.001, 0.001);
  //package://myur5_description/meshes/chair.stl
  shapes::Mesh* m = shapes::createMeshFromResource("package://ur_description/meshes/ur5/collision/chair.stl");
  ROS_INFO("Chair mesh loaded");
  sleep(2.0);

  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  co.meshes.resize(1);
  co.mesh_poses.resize(1);
  co.meshes[0] = mesh;
  co.header.frame_id = group.getPlanningFrame();
  co.mesh_poses[0].position.x = 0.0;
  co.mesh_poses[0].position.y = 0.0;
  co.mesh_poses[0].position.z = 0.0;
  co.mesh_poses[0].orientation.w= 1.0;
  co.mesh_poses[0].orientation.x= 0.0;
  co.mesh_poses[0].orientation.y= 0.0;
  co.mesh_poses[0].orientation.z= 0.0;

  co.meshes.push_back(mesh);
  co.mesh_poses.push_back(co.mesh_poses[0]);
  co.operation = co.ADD;

  collision_objects.push_back(co);
  ROS_INFO("Chair added into the world");
  sleep(5.0);
  current_scene.addCollisionObjects(collision_objects);
  sleep(5.0);
*/
  ros::shutdown();
  return 0;
}
