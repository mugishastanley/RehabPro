#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import time

# used to plot real time data of the joinnt values, not fully implemented as it wasn't used in the end

def callback(data):
    global counter, joint_velocities, x
    #joint_velocities = np.vstack((joint_velocities, data.velocity))
    #print("Velocities: " + str(joint_velocities[:,0]))

    #fig = plt.figure(1)

    #samples = np.shape(joint_velocities[:,0])
        #x = np.append(x,samples[0])
    #x = np.arange(samples[0])
    #print(x)
    #print(samples[0])
    #print(joint_velocities[:,0])
    #print(joint_velocities[:,1])
    #print(joint_velocities[:,2])

    #plt.plot(x, joint_velocities[:,0])

    #plt.plot(x, joint_velocities[:,1])
    #plt.plot(x, joint_velocities[:,2])
    #fig.canvas.draw()
    #plt.close()

    if counter % 10 == 0:
        joint_velocities = np.vstack((joint_velocities, data.velocity))
        samples = np.shape(joint_velocities[:,0])
        x = np.arange(samples[0])

        stamp = data.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        plt.figure(1)

        plt.plot(x, joint_velocities[:,0])
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)
    counter += 1
"""
def listener():
    global joint_velocities,x
    rospy.init_node('plotter', anonymous=True)


    rate = rospy.Rate(50) # 10hz

    while not rospy.is_shutdown():
        #if(len(joint_velocities) > 7):
            #print(np.shape(joint_velocities[:,0]))
            #print(joint_velocities[:,0])
            #samples = np.shape(joint_velocities[:,0])
            #x = np.arange(samples[0])
            #print(x)
            #print(joint_velocities[:,0])
            #plt.figure(1)
            #plt.plot(x, joint_velocities[:,0])
            #fig.canvas.draw()
            #plt.show()

        rate.sleep()
"""
if __name__ == '__main__':
    counter = 0
    joint_velocities = np.zeros((6,))
    x = [0,1]

    rospy.init_node("plotter")
    rospy.Subscriber("joint_states", JointState, callback)

    plt.ion()
    plt.show()
    rospy.spin()
