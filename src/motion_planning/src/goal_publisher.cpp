#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <turtlesim/Spawn.h>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <chrono>

using namespace std;
int main(int argc, char** argv){
  ros::init(argc, argv, "goal_publisher");

  ros::NodeHandle node;

  ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>("/des_goal", 1000);
  
  ros::Publisher pub_joint = node.advertise<std_msgs::Float64MultiArray>("/jvalue",1000);
  
  geometry_msgs::PoseStamped init;
  init.header.frame_id = "des_init";
  init.pose.position.x = 0.5; //for projection and goal iter 0.0      0.5 for waypoints
  init.pose.position.y = 0.5; //for projection 0.7     0.5 normally
  init.pose.position.z = 1.0;
  cout << "init: " << endl;
  cout << "x: " << init.pose.position.x << endl;
  cout << "y: " << init.pose.position.y << endl;
  cout << "z: " << init.pose.position.z << endl;
  double step_size = 0.1;  

  geometry_msgs::PoseStamped goal, aux;

  goal.header.frame_id = "des_goal";
  goal.pose.position.x = -0.5; //for projection 0.5     -0.5 for waypoints
  goal.pose.position.y = 0.5; //for projection 0.7     0.5 normally
  goal.pose.position.z = 1.5; //for projection 1.0     1.5 for waypoints
  cout << "goal: " << endl;
  cout << "x: " << goal.pose.position.x << endl;
  cout << "y: " << goal.pose.position.y << endl;
  cout << "z: " << goal.pose.position.z << endl;
  cout << "" << endl;
  
  aux = init;
  aux.header.frame_id = "des_goal";

  std_msgs::Float64MultiArray joint_value;

  joint_value.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  int count = 0;
  int type_goal = 0;

  ros::Rate rate(10.0);
  
  sleep(5);
  pub.publish(init);
    
  pub.publish(goal);
  
  auto start = std::chrono::high_resolution_clock::now();
  while (node.ok()){

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;

    //std::cout << "Elapsed time: " << elapsed.count() << endl;


    //cout << "here" << endl;
    // for cartesian goals
    
    /*
    // THIS BLOCK FOR CONSTANTLY CHANGING GOAL

    goal.pose.position.x = 0.5;
    goal.pose.position.y = 0.5;
    goal.pose.position.z = 1.5;
    
    
    if (count >= 20){
      if(type_goal==0){
        cout << "case 0" << endl;
        if (aux.pose.position.x < goal.pose.position.x){
          aux.pose.position.x += step_size;
        }

        else if (aux.pose.position.x >= goal.pose.position.x){
          aux.pose.position.x = goal.pose.position.x;
          aux.pose.position.z += step_size;
          if (aux.pose.position.z >= goal.pose.position.z){
            aux.pose.position.z = goal.pose.position.z;
          }
        }

        else{
          cout << "Final goal reached" << endl;
          break;
        }

        cout << "Goal: " << endl;
        cout << "x = " << aux.pose.position.x << endl;
        cout << "y = " << aux.pose.position.y << endl;
        cout << "z = " << aux.pose.position.z << endl;
        pub.publish(aux);
      }

      if(type_goal==1){
        cout << "case 1" << endl;
        if(joint_value.data[0] < 1.57){
          joint_value.data[0] += step_size;
        }
        else if (joint_value.data[0] >= 1.57){
          joint_value.data[0] = 1.57;
          joint_value.data[1] -= step_size;
          if(joint_value.data[1] <= -1.57){
            joint_value.data[1] = -1.57;
          }
        }
        else{
          cout << "Final goal reached" << endl;
          break;
        }

        cout << "Joints: " << endl;
        cout << "joint 0: " << joint_value.data[0] << endl;
        cout << "joint 1: " << joint_value.data[1] << endl;
        cout << "joint 2: " << joint_value.data[2] << endl;

        pub_joint.publish(joint_value);
      }
      
      count = 0;
    }
    count += 1;

    //cout << "Goal: " << endl;
    //cout << "x = " << goal.pose.position.x << endl;
    //cout << "y = " << goal.pose.position.y << endl;
    //cout << "z = " << goal.pose.position.z << endl;
    //pub.publish(goal);

    //cout << "Joints: " << endl;
    //cout << "joint 0: " << joint_value.data[0] << endl;
    //cout << "joint 1: " << joint_value.data[1] << endl;
    //cout << "joint 2: " << joint_value.data[2] << endl;

    //pub_joint.publish(joint_value);
    
    
    */
    // UNTIL HERE

    /*
    
    // FOR PROJECTION
    if(elapsed.count() >= 10){
      //store current goal as new init
      init = goal;
      init.header.frame_id = "des_init";
      cout << "init: " << endl;
      cout << "x: " << init.pose.position.x << endl;
      cout << "y: " << init.pose.position.y << endl;
      cout << "z: " << init.pose.position.z << endl;

      //store current init as new goal
      goal = aux;
      goal.header.frame_id = "des_goal";
      cout << "goal: " << endl;
      cout << "x: " << goal.pose.position.x << endl;
      cout << "y: " << goal.pose.position.y << endl;
      cout << "z: " << goal.pose.position.z << endl;
      cout << "" << endl;
      //update following goal to come back to it next iteration
      aux = init;
      start = std::chrono::high_resolution_clock::now();
      pub.publish(init);
    
      pub.publish(goal);
    }
    
    // END PROJECTION
    */
    
    //FOR WAYPOINTS
    pub.publish(init);
    
    pub.publish(goal);
    
    rate.sleep();
  }
  
  ros::shutdown();
  return 0;
};