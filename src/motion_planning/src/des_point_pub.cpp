#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
int main(int argc, char** argv){
  ros::init(argc, argv, "des_point_pub");

  ros::NodeHandle node;

  ros::Publisher pub_joint = node.advertise<std_msgs::Float64MultiArray>("/des_point",1000);

  ros::Rate rate(10.0);

  std::vector<double> p1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  p1[0] = -2.8632;
  p1[1] = -2.4939;
  p1[2] = -1.1904;
  p1[3] = 4.1403;
  p1[4] = -1.8930;
  p1[5] = 0.2505;

  std::vector<double> p2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  p2[0] = -1.5948;
  p2[1] = -1.7758;
  p2[2] = -2.1383;
  p2[3] = 4.2711;
  p2[4] = -0.8611;
  p2[5] = 0.2504;


  std::vector<double> p3 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  p3[0] = -1.5445;
  p3[1] = -3.2230;
  p3[2] = -0.8028;
  p3[3] = 4.6511;
  p3[4] = -0.8804;
  p3[5] = -0.1896;


  string des_frame;
  std_msgs::Float64MultiArray picked_point;
  while (node.ok()){

    cout << "Enter the desired point: " << endl;
    getline(cin, des_frame);
    cout << " " << endl;

    if(des_frame == "p1"){
      cout << "p1" << endl;
      picked_point.data = {p1[0], p1[1], p1[2], p1[3], p1[4], p1[5]};
    }

    else if(des_frame == "p2"){
      cout << "p2" << endl;
      picked_point.data = {p2[0], p2[1], p2[2], p2[3], p2[4], p2[5]};
    }

    else if(des_frame == "p3"){
      cout << "p3" << endl;
      picked_point.data = {p3[0], p3[1], p3[2], p3[3], p3[4], p3[5]};
    }

    // desired goal
    pub_joint.publish(picked_point);

    rate.sleep();
  }

  ros::shutdown();
  return 0;
};
