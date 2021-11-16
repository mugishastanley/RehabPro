#!/usr/bin/env python

import sys
import numpy as np
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix



"""
Solve the kinematics for the robot, it subscribes (if listener() is called in main) to the /listener topic that comes from unity
It processes the PoseStamped msg and obtains the T matrix xhich the can be used to find the inverse kinematics in the main function 
"""



IKPI = 3.14159265358979
IK2PI = IKPI * 2

ZERO_THRESH = 0.00000001
#define SIGN(x) ( ( (x) > 0 ) - ( (x) < 0 ) )
def SIGN(x):
    #print(sign(4.0))
    if(x >= 0.0):
        a=1.0
    else:
        a=-1.0
    return a
#define PI M_PI


#define UR5_PARAMS
d1 = 0.089159
a2 = -0.42500
a3 = -0.39225
d4 = 0.10915
d5 = 0.09465
d6 = 0.0823

def forward(q,T):
    """
    @param q       The 6 joint values
    @param T       The 4x4 end effector pose in row-major ordering

    Test Example
    q = [0.0, 0.0, 1.0, 0.0, 1.0, 0.0]
    T = [None] * 16
    forward(q, T)
    for i in range(0,4):
        for j in range(i*4,(i+1)*4):
            print("%1.3f ", T[j])
        print("\n")
    """
    qcounter = 0
    Tcounter = 0
    s1 = np.sin(q[qcounter])
    c1 = np.cos(q[qcounter])
    qcounter += 1

    q234 = q[qcounter]
    s2= np.sin(q[qcounter])
    c2= np.cos(q[qcounter])
    qcounter +=1

    s3 = np.sin(q[qcounter])
    c3= np.cos(q[qcounter])
    q234 += q[qcounter]
    qcounter +=1

    q234 += q[qcounter]
    qcounter +=1

    s5 = np.sin(q[qcounter])
    c5= np.cos(q[qcounter])
    qcounter +=1

    s6= np.sin(q[qcounter])
    c6= np.cos(q[qcounter])
    s234= np.sin(q234)
    c234=np.cos(q234)

    q= ((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2

    T[Tcounter] = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) -\
          (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0)
    Tcounter+=1 #//nx

    T[Tcounter] = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 -\
          s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0))
    Tcounter+=1 #//ox


    T[Tcounter]=   c5*s1 - ((c1*c234-s1*s234)*s5)/2.0 -((c1*c234-s1*s234)*s5)/2.0
    Tcounter+=1 #//ax

    T[Tcounter] = -(d5*(s1*c234-c1*s234))/2.0 + (d5*(s1*c234+c1*s234))/2.0 +\
    d4*s1 - (d6*(c1*c234-s1*s234)*s5)/2.0 - (d6*(c1*c234+s1*s234)*s5)/2.0 +\
    a2*c1*c2 + d6*c5*s1 + a3*c1*c2*c3 - a3*c1*s2*s3
    Tcounter+=1 # //px

    T[Tcounter] = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) +\
          s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0))
    Tcounter+=1 #//ny

    T[Tcounter] = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) -\
          s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0))
    Tcounter+=1# //oy

    T[Tcounter] = -c1*c5 -((s1*c234+c1*s234)*s5)/2.0 + ((c1*s234-s1*c234)*s5)/2.0
    Tcounter+=1 #//ay

    T[Tcounter] = -(d5*(c1*c234-s1*s234))/2.0 + (d5*(c1*c234+s1*s234))/2.0 - d4*c1 -\
    (d6*(s1*c234+c1*s234)*s5)/2.0 - (d6*(s1*c234-c1*s234)*s5)/2.0 - d6*c1*c5 +\
    a2*c2*s1 + a3*c2*c3*s1 - a3*s1*s2*s3
    Tcounter+=1 # //py


    T[Tcounter] = ((s234*c6+c234*s6)/2.0 + s234*c5*c6-(s234*c6-c234*s6)/2.0)
    Tcounter+=1 #//nz

    T[Tcounter] = ((c234*c6+s234*s6)/2.0 + (c234*c6-s234*s6)/2.0 - s234*c5*s6 )
    Tcounter+=1 # oz

    T[Tcounter] = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0)
    Tcounter+=1 # //az

    T[Tcounter] = (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 -\
    (d6*(c234*c5+s234*s5))/2.0 - d5*c234)
    Tcounter+=1; #//pz
    T[Tcounter] = 0.0
    Tcounter+=1
    T[Tcounter] = 0.0
    Tcounter+=1
    T[Tcounter] = 0.0
    Tcounter+=1
    T[Tcounter] = 1.0

def forward_all(q, T1, T2, T3, T4, T5, T6):
    """
    @param q       The 6 joint values
    @param Ti      The 4x4 link i pose in row-major ordering. If NULL, nothing is stored.
    """
    qcounter = 0
    s1 = np.sin(q[qcounter])
    c1 = np.cos(q[qcounter])
    qcounter+=1 #// q1
    q23 = q[qcounter]
    q234 = q[qcounter]
    s2 = np.sin(q[qcounter])
    c2 = np.cos(q[qcounter])
    qcounter+=1 #// q2
    s3 = np.sin(q[qcounter])
    c3 = np.cos(q[qcounter])
    q23 += q[qcounter]
    q234 += q[qcounter]
    qcounter+=1 #// q3
    q234 += q[qcounter]
    qcounter+=1#// q4
    s5 = np.sin(q[qcounter])
    c5 = np.cos(q[qcounter])
    qcounter+=1; #// q5
    s6 = np.sin(q[qcounter])
    c6 = np.cos(q[qcounter]) #// q6
    s23 = np.sin(q23)
    c23 = np.cos(q23)
    s234 = np.sin(q234)
    c234 = np.cos(q234)

    if not T1 :
        Tcounter=0
        T1[Tcounter] = c1;
        Tcounter+=1;
        T1[Tcounter] = 0;
        Tcounter+=1;
        T1[Tcounter] = s1;
        Tcounter+=1;
        T1[Tcounter] = 0;
        Tcounter+=1;
        T1[Tcounter] = s1;
        Tcounter+=1;
        T1[Tcounter] = 0;
        Tcounter+=1;
        T1[Tcounter] = -c1;
        Tcounter+=1;
        T1[Tcounter] = 0;
        Tcounter+=1;
        T1[Tcounter] = 0;
        Tcounter+=1;
        T1[Tcounter] = 1;
        Tcounter+=1;
        T1[Tcounter] = 0;
        Tcounter+=1;
        T1[Tcounter] =d1;
        Tcounter+=1;
        T1[Tcounter] = 0;
        Tcounter+=1;
        T1[Tcounter] = 0;
        Tcounter+=1;
        T1[Tcounter] = 0;
        Tcounter+=1;
        T1[Tcounter] = 1;
        Tcounter+=1;


    if not T2:
      Tcounter=0
      T2[Tcounter] = c1*c2;
      Tcounter+=1;
      T2[Tcounter] = -c1*s2;
      Tcounter+=1;
      T2[Tcounter] = s1;
      Tcounter+=1;
      T2[Tcounter] =a2*c1*c2;
      Tcounter+=1;
      T2[Tcounter] = c2*s1;
      Tcounter+=1;
      T2[Tcounter] = -s1*s2;
      Tcounter+=1;
      T2[Tcounter] = -c1;
      Tcounter+=1;
      T2[Tcounter] =a2*c2*s1;
      Tcounter+=1;
      T2[Tcounter] = s2;
      Tcounter+=1;
      T2[Tcounter] = c2;
      Tcounter+=1;
      T2[Tcounter] = 0;
      Tcounter+=1;
      T2[Tcounter] = d1 + a2*s2;
      Tcounter+=1;
      T2[Tcounter] = 0;
      Tcounter+=1;
      T2[Tcounter] = 0;
      Tcounter+=1;
      T2[Tcounter] = 0;
      Tcounter+=1;
      T2[Tcounter] = 1;
      Tcounter+=1;


    if not T3:
      Tcounter=0
      T3[Tcounter] = c23*c1;
      Tcounter+=1;
      T3[Tcounter] = -s23*c1;
      Tcounter+=1;
      T3[Tcounter] = s1; Tcounter+=1;
      T3[Tcounter] =c1*(a3*c23 + a2*c2);
      Tcounter+=1;
      T3[Tcounter] = c23*s1;
      Tcounter+=1;
      T3[Tcounter] = -s23*s1;
      Tcounter+=1;
      T3[Tcounter] = -c1;
      Tcounter+=1;
      T3[Tcounter] = s1*(a3*c23 + a2*c2);
      Tcounter+=1;
      T3[Tcounter] = s23;
      Tcounter+=1;
      T3[Tcounter] = c23;
      Tcounter+=1;
      T3[Tcounter] = 0;
      Tcounter+=1;
      T3[Tcounter] = d1 + a3*s23 + a2*s2;
      Tcounter+=1;
      T3[Tcounter] = 0;
      Tcounter+=1;
      T3[Tcounter] = 0;
      Tcounter+=1;
      T3[Tcounter] = 0;
      Tcounter+=1;
      T3[Tcounter] = 1;
      Tcounter+=1;


    if not T4:
      Tcounter=0
      T4[Tcounter] = c234*c1;
      Tcounter+=1;
      T4[Tcounter] = s1;
      Tcounter+=1;
      T4[Tcounter] = s234*c1;
      Tcounter+=1;
      T4[Tcounter] =c1*(a3*c23 + a2*c2) + d4*s1;
      Tcounter+=1;
      T4[Tcounter] = c234*s1;
      Tcounter+=1;
      T4[Tcounter] = -c1;
      Tcounter+=1;
      T4[Tcounter] = s234*s1;
      Tcounter+=1;
      T4[Tcounter] =s1*(a3*c23 + a2*c2) - d4*c1;
      Tcounter+=1;
      T4[Tcounter] = s234;
      Tcounter+=1;
      T4[Tcounter] = 0;
      Tcounter+=1;
      T4[Tcounter] = -c234;
      Tcounter+=1;
      T4[Tcounter] = d1 + a3*s23 + a2*s2;
      Tcounter+=1;
      T4[Tcounter] = 0;
      Tcounter+=1;
      T4[Tcounter] = 0;
      Tcounter+=1;
      T4[Tcounter] = 0;
      Tcounter+=1;
      T4[Tcounter] = 1;
      Tcounter+=1;


    if not T5:
      Tcounter=0
      T5[Tcounter] = s1*s5 + c234*c1*c5;
      Tcounter+=1;
      T5[Tcounter] = -s234*c1;
      Tcounter+=1;
      T5[Tcounter] = c5*s1 - c234*c1*s5;
      Tcounter+=1;
      T5[Tcounter] =c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1;
      Tcounter+=1;
      T5[Tcounter] = c234*c5*s1 - c1*s5;
      Tcounter+=1;
      T5[Tcounter] = -s234*s1;
      Tcounter+=1;
      T5[Tcounter] = - c1*c5 - c234*s1*s5;
      Tcounter+=1;
      T5[Tcounter] =s1*(a3*c23 + a2*c2) - d4*c1 + d5*s234*s1;
      Tcounter+=1;
      T5[Tcounter] =  s234*c5;
      Tcounter+=1;
      T5[Tcounter] = c234;
      Tcounter+=1;
      T5[Tcounter] = -s234*s5;
      Tcounter+=1;
      T5[Tcounter] = d1 + a3*s23 + a2*s2 - d5*c234;
      Tcounter+=1;
      T5[Tcounter] = 0;
      Tcounter+=1;
      T5[Tcounter] = 0;
      Tcounter+=1;
      T5[Tcounter] = 0;
      Tcounter+=1;
      T5[Tcounter] = 1;
      Tcounter+=1;


    if not T6:
      Tcounter=0
      T6[Tcounter] = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6;
      Tcounter+=1;
      T6[Tcounter] = - s6*(s1*s5 + c234*c1*c5) - s234*c1*c6;
      Tcounter+=1;
      T6[Tcounter] = c5*s1 - c234*c1*s5;
      Tcounter+=1;
      T6[Tcounter] =d6*(c5*s1 - c234*c1*s5) + c1*(a3*c23 + a2*c2) + d4*s1 + d5*s234*c1;
      Tcounter+=1;
      T6[Tcounter] = - c6*(c1*s5 - c234*c5*s1) - s234*s1*s6;
      Tcounter+=1;
      T6[Tcounter] = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1;
      Tcounter+=1;
      T6[Tcounter] = - c1*c5 - c234*s1*s5;
      Tcounter+=1;
      T6[Tcounter] =s1*(a3*c23 + a2*c2) - d4*c1 - d6*(c1*c5 + c234*s1*s5) + d5*s234*s1;
      Tcounter+=1;
      T6[Tcounter] = c234*s6 + s234*c5*c6;
      Tcounter+=1;
      T6[Tcounter] = c234*c6 - s234*c5*s6;
      Tcounter+=1;
      T6[Tcounter] = -s234*s5;
      Tcounter+=1;
      T6[Tcounter] = d1 + a3*s23 + a2*s2 - d5*c234 - d6*s234*s5;
      Tcounter+=1;
      T6[Tcounter] = 0;
      Tcounter+=1;
      T6[Tcounter] = 0;
      Tcounter+=1;
      T6[Tcounter] = 0;
      Tcounter+=1;
      T6[Tcounter] = 1;
      Tcounter+=1;



def inverse(T, q_sols, q6_des=0.0):
    """
    @param T       The 4x4 end effector pose in row-major ordering
    @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
    @param q6_des  An optional parameter which designates what the q6 value should take
                  in case of an infinite solution on that joint.
    @return        Number of solutions found (maximum of 8)
    """
    num_sols = 0;
    Tcounter=0
    T00 =  T[Tcounter];
    Tcounter += 1;
    T01 =  T[Tcounter];
    Tcounter += 1;
    T02 = T[Tcounter];
    Tcounter += 1;
    T03 = T[Tcounter];
    Tcounter += 1;

    T10 =  T[Tcounter];
    Tcounter += 1;
    T11 =  T[Tcounter];
    Tcounter += 1;
    T12 = T[Tcounter];
    Tcounter += 1;
    T13 = T[Tcounter];
    Tcounter += 1;


    T20 = T[Tcounter];
    Tcounter += 1;
    T21 = T[Tcounter];
    Tcounter += 1;
    T22 =  T[Tcounter];
    Tcounter += 1;
    T23 =  T[Tcounter];

    #Rotate joint q1
     #double q1[2];
    q1 = [None] * 2
    A = d6*T12 - T13
    B = d6*T02 - T03
    R = A*A + B*B;
    if np.abs(A) < ZERO_THRESH:
        if np.abs(np.abs(d4) - np.abs(B)) < ZERO_THRESH:
          div = -SIGN(d4)*SIGN(B);
        else:
          div = -d4/B
        arcsin = np.arcsin(div)
        if np.abs(arcsin) < ZERO_THRESH:
          arcsin = 0.0
        if arcsin < 0.0:
          q1[0] = arcsin + 2.0*IKPI
        else :
          q1[0] = arcsin
        q1[1] = IKPI - arcsin

    elif np.abs(B) < ZERO_THRESH :
        if np.abs(np.abs(d4) - np.abs(A)) < ZERO_THRESH:
          div = SIGN(d4)*SIGN(A);
        else:
          div = d4/A;
        arccos = np.arccos(div)
        q1[0] = arccos
        q1[1] = 2.0*IKPI - arccos

    #elif(d4*d4 > R) :
       # return num_sols
    else:
        arccos = np.arccos(d4 / np.sqrt(R))
        arctan = np.arctan2(-B, A);
        pos = arccos + arctan;
        neg = -arccos + arctan;
        if np.abs(pos) < ZERO_THRESH:
          pos = 0.0;
        if np.abs(neg) < ZERO_THRESH:
          neg = 0.0;
        if(pos >= 0.0):
          q1[0] = pos
        else:
          q1[0] = 2.0*IKPI + pos
        if(neg >= 0.0):
          q1[1] = neg
        else:
          q1[1] = 2.0*IKPI + neg
    ##########################################################################

    ####################################### wrist 2 joint (q5) ###############
    q5=[[0]*2 for i in range(2)]
    for i in range (0,2):
        numer = (T03*np.sin(q1[i]) - T13*np.cos(q1[i])-d4)
        if(np.abs(np.abs(numer) - np.abs(d6)) < ZERO_THRESH):
          div = SIGN(numer) * SIGN(d6);
        else:
          div = numer / d6;
        arccos = np.arccos(div);
        q5[i][0] = arccos;
        q5[i][1] = 2.0*IKPI - arccos;
    ###########################################################################
    for i in range (0,2):
        for j in range(0,2):
          c1 = np.cos(q1[i])
          s1 = np.sin(q1[i])
          c5 = np.cos(q5[i][j])
          s5 = np.sin(q5[i][j])
          #////////////////////////////// wrist 3 joint (q6) //////////////////////////////
          if(np.abs(s5) < ZERO_THRESH):
            q6 = q6_des
          else:
            q6 = np.arctan2(SIGN(s5)*-(T01*s1 - T11*c1),\
                       SIGN(s5)*(T00*s1 - T10*c1))
            if(np.abs(q6) < ZERO_THRESH):
              q6 = 0.0;
            if(q6 < 0.0):
              q6 += 2.0*IKPI
          #print("q6 is %1.6f\n"%q6)
          #////////////////////////////////////////////////////////////////////////////////

          q2=[None]*2
          q3=[None]*2
          q4=[None]*2
          #///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
          c6 = np.cos(q6)
          s6 = np.sin(q6)
          x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1))
          x04y = c5*(T20*c6 - T21*s6) - T22*s5
          p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) +\
                        T03*c1 + T13*s1
          p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6)

          c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3)
          if(np.abs(np.abs(c3) - 1.0) < ZERO_THRESH):
            c3 = SIGN(c3)
          elif(np.abs(c3) > 1.0):
            #// TODO NO SOLUTION
            continue

          arccos = np.arccos(c3)
          q3[0] = arccos
          q3[1] = 2.0*IKPI - arccos
          denom = a2*a2 + a3*a3 + 2*a2*a3*c3
          s3 = np.sin(arccos)
          A = (a2 + a3*c3)
          B = a3*s3
          q2[0] = np.arctan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom)
          q2[1] = np.arctan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom)
          c23_0 = np.cos(q2[0]+q3[0]);
          s23_0 = np.sin(q2[0]+q3[0]);
          c23_1 = np.cos(q2[1]+q3[1]);
          s23_1 = np.sin(q2[1]+q3[1]);
          q4[0] = np.arctan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
          q4[1] = np.arctan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);

          #////////////////////////////////////////////////////////////////////////////////
          for k in range (0,2):
            if(np.abs(q2[k]) < ZERO_THRESH):
              q2[k] = 0.0
            elif(q2[k] < 0.0):
                q2[k] += 2.0*IKPI
            if(np.abs(q4[k]) < ZERO_THRESH):
              q4[k] = 0.0
            elif(q4[k] < 0.0):
                q4[k] += 2.0*IKPI
            q_sols[num_sols*6+0] = q1[i]
            q_sols[num_sols*6+1] = q2[k]
            q_sols[num_sols*6+2] = q3[k]
            q_sols[num_sols*6+3] = q4[k]
            q_sols[num_sols*6+4] = q5[i][j]
            q_sols[num_sols*6+5] = q6
            num_sols+=1
    return num_sols



def limitsol(t):
    if t > IKPI:
        t=t-IK2PI
    return t



def Test(T):
    q = [0.0, 0.0, 1.0, 0.0, 1.0, 0.0]
    #q = [-0.136, -0.392, 0.10, 0, 3.11, 0.04]
    q = [0.003, 0.353, 0.149, 1.57, 0, 0.00]
    #T = [None] * 16
    forward(q, T)
    # for i in range(0,4):
    #    for j in range(i*4,(i+1)*4):
    #        print("%1.5f " %T[j])
    #    print("\n")
#
    q_sols=[None]*48
    num_sols = inverse(T, q_sols)
    #inverse(T, q_sols)
    print("num sols :",num_sols)
    for i in range(0,num_sols):
        for j in range (0,6):
            q_sols[i*6+j]=limitsol(q_sols[i*6+j])
            #q_sols[i*6+j]=(q_sols[i*6+j])
    for i in range(0,num_sols):
        print("[%1.6f, %1.6f, %1.6f, %1.6f, %1.6f, %1.6f]\n"\
           %(q_sols[i*6+0] , q_sols[i*6+1] ,q_sols[i*6+2] ,q_sols[i*6+3] ,q_sols[i*6+4] ,q_sols[i*6+5]))
    print("\n")
#


def BestSol(T):
    """this is a naive algorithm to eliminate solutions with high values"""
    q_sols=[None]*48
    num_sols = inverse(T, q_sols)

    #if null, do nothing, else
    #create a list or dictionary to store solutions and sum of their absolute values
    for i in range(0,num_sols):
        for j in range (0,6):
            q_sols[i*6+j]=limitsol(q_sols[i*6+j])    #return solution with smallest
    i=0
    bestsol=[q_sols[i*6+0] , q_sols[i*6+1] ,q_sols[i*6+2] ,q_sols[i*6+3] ,q_sols[i*6+4] ,q_sols[i*6+5]]
    return bestsol


#ROS part
pose_unity = PoseStamped()
T_unity = np.zeros([4, 4])
t = np.zeros(4)
q = np.zeros(4)

def cb_pose_T(PS_msg):


    frame_id = PS_msg.header.frame_id
    #Build translation part of the matrix
    t[0] = -PS_msg.pose.position.y #Unity conventions
    t[1] = PS_msg.pose.position.x
    t[2] = PS_msg.pose.position.z
    t[3] = 1
    #Build quaternion from pose
    q[0] = -PS_msg.pose.orientation.y #Respect unity conventions
    q[1] = PS_msg.pose.orientation.x
    q[2] = PS_msg.pose.orientation.z
    q[3] = PS_msg.pose.orientation.w

    R = quaternion_matrix([q[0], q[1], q[2], q[3]])

    rospy.loginfo("I heard id: %s", frame_id)
    rospy.loginfo("pos x: %f", t[0])
    rospy.loginfo("pos y: %f", t[1])
    rospy.loginfo("pos z: %f", t[2])
    rospy.loginfo("orient x: %f", q[0])
    rospy.loginfo("orient y: %f", q[1])
    rospy.loginfo("orient z: %f", q[2])
    rospy.loginfo("orient w: %f", q[3])


    R[:,3] = t
    T_unity = R
    print(T_unity)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('UR5kine', anonymous=True)

    rospy.Subscriber("listener", PoseStamped, cb_pose_T)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()





def main():

    #calls the listener function
    listener()

    """
    Tp = np.zeros(16)
    Tp[0] =  0.70711
    Tp[1] = 0.00000
    Tp[2] = 0.70711
    Tp[3] = -0.50699
    Tp[4] = 0.70711
    Tp[5] = 0.0
    Tp[6] = -0.70711
    Tp[7] = -0.37406
    Tp[8] = 0.00000
    Tp[9] = 1.0
    Tp[10] = 0.00000
    Tp[11] = 0.1380
    Tp[12] = 0.00000
    Tp[13] = 0.00000
    Tp[14] = 0.00000
    Tp[15] = 1.00000

    print(Tp)

    q_sols=[None]*48
    num_sols = inverse(Tp, q_sols)

    print("num sols :",num_sols)
    for i in range(0,num_sols):
        for j in range (0,6):
            q_sols[i*6+j]=limitsol(q_sols[i*6+j])
            #q_sols[i*6+j]=(q_sols[i*6+j])

    for i in range(0,num_sols):
        print("[%1.6f, %1.6f, %1.6f, %1.6f, %1.6f, %1.6f]\n"\
            %(q_sols[i*6+0] , q_sols[i*6+1] ,q_sols[i*6+2] ,q_sols[i*6+3] ,q_sols[i*6+4] ,q_sols[i*6+5]))
    print("\n")
    T = np.zeros(16)
    q1 = [50.61,-74.32,128.3,-55.22,96.87,0.39]
    for i in range(0,6):
        q1[i]=q1[i]* IKPI/180

    forward(q_sols, T)

    print("forward matrix: \n")
    print(T)
    """
if __name__ == '__main__':
	main()
