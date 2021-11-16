#include <iostream>
#include <tf2/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <math.h>
#include <stdio.h>
#include <vector>

using namespace std;

namespace ur_kinematics {

  namespace {
    const double ZERO_THRESH = 0.00000001;
    int SIGN(double x) {
      return (x > 0) - (x < 0);
    }
    const double PI = M_PI;

    //#define UR10_PARAMS
    #ifdef UR10_PARAMS
    const double d1 =  0.1273;
    const double a2 = -0.612;
    const double a3 = -0.5723;
    const double d4 =  0.163941;
    const double d5 =  0.1157;
    const double d6 =  0.0922;
    #endif

    #define UR5_PARAMS
    #ifdef UR5_PARAMS
    const double d1 =  0.089159;
    const double a2 = -0.42500;
    const double a3 = -0.39225;
    const double d4 =  0.10915;
    const double d5 =  0.09465;
    const double d6 =  0.0823;
    #endif

    //#define UR3_PARAMS
    #ifdef UR3_PARAMS
    const double d1 =  0.1519;
    const double a2 = -0.24365;
    const double a3 = -0.21325;
    const double d4 =  0.11235;
    const double d5 =  0.08535;
    const double d6 =  0.0819;
    #endif
    }

   //std::vector< IKREAL_TYPE > vfree(num_free_parameters);
  void forward(const double* q, double* T) {
    double s1 = sin(*q), c1 = cos(*q); q++;
    double q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
    double s3 = sin(*q), c3 = cos(*q); q234 += *q; q++;
    q234 += *q; q++;
    double s5 = sin(*q), c5 = cos(*q); q++;
    double s6 = sin(*q), c6 = cos(*q);
    double s234 = sin(q234), c234 = cos(q234);
    *T = ((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0; T++;
    *T = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) -
          (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0); T++;
    *T = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 -
          s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0)); T++;
    *T = ((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 -
          d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 -
          a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3); T++;
    *T = c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0; T++;
    *T = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) +
          s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0)); T++;
    *T = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) -
          s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0)); T++;
    *T = ((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 +
          (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 -
          a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3); T++;
    *T = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0); T++;
    *T = ((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6); T++;
    *T = (s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0); T++;
    *T = (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 -
         (d6*(c234*c5+s234*s5))/2.0 - d5*c234); T++;
    *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
  }

  void forward_all(const double* q, double* T1, double* T2, double* T3,
                                    double* T4, double* T5, double* T6) {
    double s1 = sin(*q), c1 = cos(*q); q++; // q1
    double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++; // q2
    double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++; // q3
    q234 += *q; q++; // q4
    double s5 = sin(*q), c5 = cos(*q); q++; // q5
    double s6 = sin(*q), c6 = cos(*q); // q6
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);

    if(T1 != NULL) {
      *T1 = c1; T1++;
      *T1 = 0; T1++;
      *T1 = s1; T1++;
      *T1 = 0; T1++;
      *T1 = s1; T1++;
      *T1 = 0; T1++;
      *T1 = -c1; T1++;
      *T1 = 0; T1++;
      *T1 =       0; T1++;
      *T1 = 1; T1++;
      *T1 = 0; T1++;
      *T1 =d1; T1++;
      *T1 =       0; T1++;
      *T1 = 0; T1++;
      *T1 = 0; T1++;
      *T1 = 1; T1++;
    }

    if(T2 != NULL) {
      *T2 = c1*c2; T2++;
      *T2 = -c1*s2; T2++;
      *T2 = s1; T2++;
      *T2 =a2*c1*c2; T2++;
      *T2 = c2*s1; T2++;
      *T2 = -s1*s2; T2++;
      *T2 = -c1; T2++;
      *T2 =a2*c2*s1; T2++;
      *T2 =         s2; T2++;
      *T2 = c2; T2++;
      *T2 = 0; T2++;
      *T2 =   d1 + a2*s2; T2++;
      *T2 =               0; T2++;
      *T2 = 0; T2++;
      *T2 = 0; T2++;
      *T2 =                 1; T2++;
    }

    if(T3 != NULL) {
      *T3 = c23*c1; T3++;
      *T3 = -s23*c1; T3++;
      *T3 = s1; T3++;
      *T3 =c1*(a3*c23 + a2*c2); T3++;
      *T3 = c23*s1; T3++;
      *T3 = -s23*s1; T3++;
      *T3 = -c1; T3++;
      *T3 =s1*(a3*c23 + a2*c2); T3++;
      *T3 =         s23; T3++;
      *T3 = c23; T3++;
      *T3 = 0; T3++;
      *T3 =     d1 + a3*s23 + a2*s2; T3++;
      *T3 =                    0; T3++;
      *T3 = 0; T3++;
      *T3 = 0; T3++;
      *T3 =                                     1; T3++;
    }

    if(T4 != NULL) {
      *T4 = c234*c1; T4++;
      *T4 = s1; T4++;
      *T4 = s234*c1; T4++;
      *T4 =c1*(a3*c23 + a2*c2) + d4*s1; T4++;
      *T4 = c234*s1; T4++;
      *T4 = -c1; T4++;
      *T4 = s234*s1; T4++;
      *T4 =s1*(a3*c23 + a2*c2) - d4*c1; T4++;
      *T4 =         s234; T4++;
      *T4 = 0; T4++;
      *T4 = -c234; T4++;
      *T4 =                  d1 + a3*s23 + a2*s2; T4++;
      *T4 =                         0; T4++;
      *T4 = 0; T4++;
      *T4 = 0; T4++;
      *T4 =                                                  1; T4++;
    }

    if(T5 != NULL) {
      *T5 = s1*s5 + c234*c1*c5; T5++;
      *T5 = -s234*c1; T5++;
      *T5 = c5*s1 - c234*c1*s5; T5++;
      *T5 =c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T5++;
      *T5 = c234*c5*s1 - c1*s5; T5++;
      *T5 = -s234*s1; T5++;
      *T5 = - c1*c5 - c234*s1*s5; T5++;
      *T5 =s1*(a3*c23 + a2*c2) - d4*c1 + d5*s234*s1; T5++;
      *T5 =                           s234*c5; T5++;
      *T5 = c234; T5++;
      *T5 = -s234*s5; T5++;
      *T5 =                          d1 + a3*s23 + a2*s2 - d5*c234; T5++;
      *T5 =                                                   0; T5++;
      *T5 = 0; T5++;
      *T5 = 0; T5++;
      *T5 =                                                                                 1; T5++;
    }

    if(T6 != NULL) {
      *T6 =   c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T6++;
      *T6 = - s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T6++;
      *T6 = c5*s1 - c234*c1*s5; T6++;
      *T6 =d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1; T6++;
      *T6 = - c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T6++;
      *T6 = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T6++;
      *T6 = - c1*c5 - c234*s1*s5; T6++;
      *T6 =s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1; T6++;
      *T6 =                                       c234*s6 + s234*c5*c6; T6++;
      *T6 = c234*c6 - s234*c5*s6; T6++;
      *T6 = -s234*s5; T6++;
      *T6 =                                                      d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5; T6++;
      *T6 =                                                                                                   0; T6++;
      *T6 = 0; T6++;
      *T6 = 0; T6++;
      *T6 =                                                                                                                                            1; T6++;
    }
  }

  void convert(const double* pose, double* T)
  {
   std::vector<double> eetrans(3);
   std::vector<double> eerot(9);
   eetrans[0] = (*pose); pose++;
   eetrans[1] = (*pose); pose++;
   eetrans[2] = (*pose); pose++;

  double qx = (*pose); pose++;
  double qy = (*pose); pose++;
  double qz = (*pose); pose++;
  double qw = (*pose);
  const double n = 1.0f / sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  qw *= n;
  qx *= n;
  qy *= n;
  qz *= n;
  eerot[0] = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
  eerot[1] = 2.0f * qx * qy - 2.0f * qz * qw;
  eerot[2] = 2.0f * qx * qz + 2.0f * qy * qw;
  eerot[3] = 2.0f * qx * qy + 2.0f * qz * qw;
  eerot[4] = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
  eerot[5] = 2.0f * qy * qz - 2.0f * qx * qw;
  eerot[6] = 2.0f * qx * qz - 2.0f * qy * qw;
  eerot[7] = 2.0f * qy * qz + 2.0f * qx * qw;
  eerot[8] = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;

//  *T = eerot[2];T++; *T = eerot[0];T++; *T = eerot[1];T++; *T = eetrans[0];T++;
//  *T = eerot[5];T++; *T = eerot[3];T++; *T = eerot[4];T++; *T = eetrans[1];T++;
//  *T = eerot[8];T++; *T = eerot[6];T++; *T = eerot[7];T++; *T = eetrans[2];T++;
//  *T = 0;T++; *T = 0;T++; *T = 0;T++; *T = 1;
  *T = eerot[0];T++; *T = eerot[1];T++; *T = eerot[2];T++; *T = eetrans[0];T++;
  *T = eerot[3];T++; *T = eerot[4];T++; *T = eerot[5];T++; *T = eetrans[1];T++;
  *T = eerot[6];T++; *T = eerot[7];T++; *T = eerot[8];T++; *T = eetrans[2];T++;
  *T = 0;T++; *T = 0;T++; *T = 0;T++; *T = 1;
  }
  int inverse(const double* T, double* q_sols, double q6_des) {
      int num_sols = 0;
      // THIS IS ONLY NEEDED IF forward FUNCTION IS USED.
      //    double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++;
      //    double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++;
      //    double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;

      double T00 = T[0];
      double T01 = T[1];
      double T02 = T[2];
      double T03 = T[3];
      double T10 = T[4];
      double T11 = T[5];
      double T12 = T[6];
      double T13 = T[7];
      double T20 = T[8];
      double T21 = T[9];
      double T22 = T[10];
      double T23 = T[11];
      ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
      double q1[2];
      {
          double A = d6*T12 - T13;
          double B = d6*T02 - T03;
          double R = A*A + B*B;
          if(fabs(A) < ZERO_THRESH) {
              double div;
              if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
                  div = -SIGN(d4)*SIGN(B);
              else
                  div = -d4/B;
              double arcsin = asin(div);
              if(fabs(arcsin) < ZERO_THRESH)
                  arcsin = 0.0;
              if(arcsin < 0.0)
                  q1[0] = arcsin + 2.0*PI;
              else
                  q1[0] = arcsin;
              q1[1] = PI - arcsin;
          }
          else if(fabs(B) < ZERO_THRESH) {
              double div;
              if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
                  div = SIGN(d4)*SIGN(A);
              else
                  div = d4/A;
              double arccos = acos(div);
              q1[0] = arccos;
              q1[1] = 2.0*PI - arccos;
          }
          else if(d4*d4 > R) {
              return num_sols;
          }
          else {
              double arccos = acos(d4 / sqrt(R)) ;
              double arctan = atan2(-B, A);
              double pos = arccos + arctan;
              double neg = -arccos + arctan;
              if(fabs(pos) < ZERO_THRESH)
                  pos = 0.0;
              if(fabs(neg) < ZERO_THRESH)
                  neg = 0.0;
              if(pos >= 0.0)
                  q1[0] = pos;
              else
                  q1[0] = 2.0*PI + pos;
              if(neg >= 0.0)
                  q1[1] = neg;
              else
                  q1[1] = 2.0*PI + neg;
          }
      }
      ////////////////////////////////////////////////////////////////////////////////

      ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
      double q5[2][2];
      {
          for(int i=0;i<2;i++) {
              double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
              double div;
              if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
                  div = SIGN(numer) * SIGN(d6);
              else
                  div = numer / d6;
              double arccos = acos(div);
              q5[i][0] = arccos;
              q5[i][1] = 2.0*PI - arccos;
          }
      }
      ////////////////////////////////////////////////////////////////////////////////

      {
          for(int i=0;i<2;i++) {
              for(int j=0;j<2;j++) {
                  double c1 = cos(q1[i]), s1 = sin(q1[i]);
                  double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
                  double q6;
                  ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
                  if(fabs(s5) < ZERO_THRESH)
                      q6 = q6_des;
                  else {
                      q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1),
                                 SIGN(s5)*(T00*s1 - T10*c1));
                      if(fabs(q6) < ZERO_THRESH)
                          q6 = 0.0;
                      if(q6 < 0.0)
                          q6 += 2.0*PI;
                  }
                  ////////////////////////////////////////////////////////////////////////////////

                  double q2[2], q3[2], q4[2];
                  ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
                  double c6 = cos(q6), s6 = sin(q6);
                  double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
                  double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
                  double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) +
                          T03*c1 + T13*s1;
                  double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

                  double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
                  if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
                      c3 = SIGN(c3);
                  else if(fabs(c3) > 1.0) {
                      // TODO NO SOLUTION
                      continue;
                  }
                  double arccos = acos(c3);
                  q3[0] = arccos;
                  q3[1] = 2.0*PI - arccos;
                  double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
                  double s3 = sin(arccos);
                  double A = (a2 + a3*c3), B = a3*s3;
                  q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
                  q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
                  double c23_0 = cos(q2[0]+q3[0]);
                  double s23_0 = sin(q2[0]+q3[0]);
                  double c23_1 = cos(q2[1]+q3[1]);
                  double s23_1 = sin(q2[1]+q3[1]);
                  q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
                  q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
                  ////////////////////////////////////////////////////////////////////////////////
                  for(int k=0;k<2;k++) {
                      if(fabs(q2[k]) < ZERO_THRESH)
                          q2[k] = 0.0;
                      else if(q2[k] < 0.0) q2[k] += 2.0*PI;
                      if(fabs(q4[k]) < ZERO_THRESH)
                          q4[k] = 0.0;
                      else if(q4[k] < 0.0) q4[k] += 2.0*PI;
                      q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k];
                      q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k];
                      q_sols[num_sols*6+4] = q5[i][j]; q_sols[num_sols*6+5] = q6;
                      num_sols++;
                  }

              }
          }
      }
      return num_sols;
  }


// This part is giving problems with GetNumJoints vfree vInfo IkReal IkSingleDOFSolutionBase /////////////
/*
bool inverse_sol(const std::vector< double >& pose, std::vector< double >& joints, int& numOfSolns,  const pfree){

    if(!pfree) return false;


    double T[16];
    double q_sols[8*6];
    int n = GetNumJoints();
    convert(pose,T);
    //to_mat44(T, eetrans, eerot);
    //inverse(T,q_sols);
    int num_sols = ur_kinematics::inverse(T, q_sols,pfree[0]);


    std::vector<int> vfree(0);

    for (int i=0; i < num_sols; ++i){
        std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(n);
        for (int j=0; j < n; ++j) vinfos[j].foffset = q_sols[i*n+j];
        solutions.AddSolution(vinfos,vfree);
    }
    return num_sols > 0;
}
*/
}

using namespace ur_kinematics;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_jSpace");

  ros::NodeHandle node_handle;

  tf::TransformListener listener;


  ros::AsyncSpinner spinner(1);
  spinner.start();


  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


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
  // //Execute a joint-space goal to a known predefined position from Home position
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, modify the joints to the desired position, plan to the new joint space goal
  // Start at positive x and y
  // joint_group_positions[0] = 1.49; // radians
  // joint_group_positions[1] = -1.22; // radians
  // joint_group_positions[2] = 1.94; // radians
  // joint_group_positions[3] = -0.77; // radians
  // joint_group_positions[4] = 1.77; // radians
  // joint_group_positions[5] = -0.006; // radians

/*
  joint_group_positions[0] = -1.74; // radians
  joint_group_positions[1] = -0.916; // radians
  joint_group_positions[2] = -1.66; // radians
  joint_group_positions[3] = -3.66;//-3.4; // radians
  joint_group_positions[4] = -1.71; // radians
  joint_group_positions[5] = -0.006; // radians
*/
  joint_group_positions[0] = 0.461531; // radians
  joint_group_positions[1] = 5.523255; // radians
  joint_group_positions[2] = 1.742068; // radians
  joint_group_positions[3] = 3.730251;//-3.4; // radians
  joint_group_positions[4] = 1.570796; // radians
  joint_group_positions[5] = 1.109265; // radians

//0.461531, 5.523255, 1.742068, 3.730251, 1.570796, 1.109265


  // Start at negative x and y
  // joint_group_positions[0] = 1.69; // radians
  // joint_group_positions[1] = -1.42; // radians
  // joint_group_positions[2] = -2.26; // radians
  // joint_group_positions[3] = -2.56; // radians
  // joint_group_positions[4] = -1.40; // radians
  // joint_group_positions[5] = -0.006; // radians
  move_group.allowReplanning(true);

  move_group.setMaxVelocityScalingFactor(0.08);
  move_group.setJointValueTarget(joint_group_positions);
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");
  move_group.execute(my_plan);
/*
  ROS_INFO("Trying new plan now");

  double* bTh = new double[16];
  //bTh[0] = rot_T_bh[0][0], bTh[1] = rot_T_bh[0][1], bTh[2] = rot_T_bh[0][2], bTh[3] = T_bh.getOrigin().x();
  //bTh[4] = rot_T_bh[1][0], bTh[5] = rot_T_bh[1][1], bTh[6] = rot_T_bh[1][2], bTh[7] = T_bh.getOrigin().y();
  //bTh[8] = rot_T_bh[2][0], bTh[9] = rot_T_bh[2][1], bTh[10] = rot_T_bh[2][2], bTh[11] = T_bh.getOrigin().z();    
  //bTh[12] = 0, bTh[13] = 0, bTh[14] = 0, bTh[15] = 1;

  //bTh[0] = 0.70711, bTh[1] = 0.0, bTh[2] = 0.70711, bTh[3] = -0.50699;
  //bTh[4] = 0.70711, bTh[5] = 0.0, bTh[6] = -0.70711, bTh[7] = -0.37406;
  //bTh[8] = 0.0, bTh[9] = 1.0, bTh[10] = 0.0, bTh[11] = 0.138;
  //bTh[12] = 0, bTh[13] = 0, bTh[14] = 0, bTh[15] = 1;

  bTh[0] = 1.0, bTh[1] = 0.0, bTh[2] = 0.0, bTh[3] = -0.50699;
  bTh[4] = 0.0, bTh[5] = 1.0, bTh[6] = -0.0, bTh[7] = -0.37406;
  bTh[8] = 0.0, bTh[9] = 0.0, bTh[10] = 1.0, bTh[11] = 0.138;
  bTh[12] = 0, bTh[13] = 0, bTh[14] = 0, bTh[15] = 1;

  double qp_sols[8*6];
  int num_sols;

  //What does the 3rd parameter do? He used it with 2 in uf_kin.cpp//////////////////////////////////////////////////////
  num_sols = inverse(bTh, qp_sols, 1);
  printf("num sols: %d\n", num_sols);
  // Get joint values
  for(int j = 0; j< num_sols; j++){
    printf("%1.6f, %1.6f, %1.6f, %1.6f, %1.6f, %1.6f\n", qp_sols[6*j],qp_sols[6*j+1],qp_sols[6*j+2],qp_sols[6*j+3],qp_sols[6*j+4],qp_sols[6*j+5]);
  }

  joint_group_positions[0] = qp_sols[6*0];
  joint_group_positions[1] = qp_sols[6*0+1];
  joint_group_positions[2] = qp_sols[6*0+2];
  joint_group_positions[3] = qp_sols[6*0+3];
  joint_group_positions[4] = qp_sols[6*0+4];
  joint_group_positions[5] = qp_sols[6*0+5];

  

  // get current state
  current_state = move_group.getCurrentState();

  move_group.setJointValueTarget(joint_group_positions);
  // Try this for uninterrupted movement?
  // move_group.asyncMove();

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO( "Completed the plan from home  %s", success ? "" : "FAILED");

  move_group.move();
*/ 
  ros::shutdown();
  return 0;
};
