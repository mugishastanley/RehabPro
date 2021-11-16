#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

using namespace std;

geometry_msgs::Pose pose_unity;
string des_frame;

void cb_ball(geometry_msgs::PoseStamped pose_msg){

  des_frame = pose_msg.header.frame_id;
  des_frame = "hello";
  pose_unity.position.x = -pose_msg.pose.position.y; //from unity this is the transformation we want
  pose_unity.position.y = pose_msg.pose.position.x;
  pose_unity.position.z = pose_msg.pose.position.z;


  pose_unity.orientation.x = -pose_msg.pose.orientation.y;
  pose_unity.orientation.y = pose_msg.pose.orientation.x;
  pose_unity.orientation.z = pose_msg.pose.orientation.z;
  pose_unity.orientation.w = pose_msg.pose.orientation.w;

  cout << "received pose: " << endl;
  cout << "pos x: " << pose_msg.pose.position.x << endl;
  cout << "pos y: " << pose_msg.pose.position.y << endl;
  cout << "pos z: " << pose_msg.pose.position.z << endl;

  cout << "rot x: " << pose_msg.pose.orientation.x << endl;
  cout << "rot y: " << pose_msg.pose.orientation.y << endl;
  cout << "rot z: " << pose_msg.pose.orientation.z << endl;
  cout << "rot w: " << pose_msg.pose.orientation.w << endl;

}

// Simple publisher for giving the ID of the desired point to reach using matrix_reader_5 core (desired point to reach using the prerecorder trajectories)

int main(int argc, char** argv){
  ros::init(argc, argv, "des_frame_pub");

  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("listener", 1000, cb_ball);
  ros::Publisher pub = node.advertise<std_msgs::String>("/des_frame", 1000);


  ros::Rate rate(10.0);

  std_msgs::String msg;
  while (ros::ok()){

    //cout << "Enter the desired frame: " << endl;
    //getline(cin, des_frame);
    //cout << " " << endl;
    cout << des_frame << endl;
    msg.data = des_frame;
    pub.publish(msg);
    rate.sleep();
  }

  ros::shutdown();
  return 0;
};
