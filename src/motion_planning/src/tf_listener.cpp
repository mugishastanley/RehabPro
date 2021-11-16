#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <turtlesim/Spawn.h>
#include <math.h>

//read a computes the distance between the end effector ee_link and different frames

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("/distance_left_hand", 1000);

  tf::TransformListener list_ee_bf, list_ee_lp, list_ee_rp, list_ee_lh, list_ee_rh;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transf_ee_bf, transf_lp, transf_rp, transf_lh, transf_rh;
    try{
      list_ee_bf.waitForTransform("/ee_link", "/base_link", ros::Time(0), ros::Duration(3.0));
      list_ee_bf.lookupTransform("/ee_link", "/base_link",
                               ros::Time(0), transf_ee_bf);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    try{
      list_ee_lp.waitForTransform("/ee_link", "/left_palm_sensor", ros::Time(0), ros::Duration(3.0));
      list_ee_lp.lookupTransform("/ee_link", "/left_palm_sensor",
                               ros::Time(0), transf_lp);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    try{
      list_ee_rp.waitForTransform("/ee_link", "/right_palm_sensor", ros::Time(0), ros::Duration(3.0));
      list_ee_rp.lookupTransform("/ee_link", "/right_palm_sensor",
                               ros::Time(0), transf_rp);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    try{
      list_ee_lh.waitForTransform("/ee_link", "/left_humerus_sensor", ros::Time(0), ros::Duration(3.0));
      list_ee_lh.lookupTransform("/ee_link", "/left_humerus_sensor",
                               ros::Time(0), transf_lh);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    try{
      list_ee_rh.waitForTransform("/ee_link", "/right_humerus_sensor", ros::Time(0), ros::Duration(3.0));
      list_ee_rh.lookupTransform("/ee_link", "/right_humerus_sensor",
                               ros::Time(0), transf_rh);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    /*
    geometry_msgs::PoseStamped ee_pose;

    sensor_pose.header.frame_id = "left_palm_sensor";
    sensor_pose.pose.position.x = transf_lh.getOrigin().x();
    sensor_pose.pose.position.y = transf_lh.getOrigin().y();
    sensor_pose.pose.position.z = transf_lh.getOrigin().z();
    */

    double lp_dist, rp_dist, lh_dist, rh_dist;

    lp_dist = sqrt( pow(transf_lp.getOrigin().x(),2) + pow(transf_lp.getOrigin().y(),2)
      + pow(transf_lp.getOrigin().z(),2) );
    rp_dist = sqrt( pow(transf_rp.getOrigin().x(),2) + pow(transf_rp.getOrigin().y(),2)
      + pow(transf_rp.getOrigin().z(),2) );
    lh_dist = sqrt( pow(transf_lh.getOrigin().x(),2) + pow(transf_lh.getOrigin().y(),2)
      + pow(transf_lh.getOrigin().z(),2) );
    rh_dist = sqrt( pow(transf_rh.getOrigin().x(),2) + pow(transf_rh.getOrigin().y(),2)
      + pow(transf_rh.getOrigin().z(),2) );

    ROS_INFO("lp_dist: %f", lp_dist);
    ROS_INFO("rp_dist: %f", rp_dist);
    ROS_INFO("lh_dist: %f", lh_dist);
    ROS_INFO("rh_dist: %f", rh_dist);

    rate.sleep();
  }
  return 0;
};
