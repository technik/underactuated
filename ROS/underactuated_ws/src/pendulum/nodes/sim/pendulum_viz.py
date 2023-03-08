#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
import math
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelState

pendulum_length = 1

ax = plt.subplot(111, polar=True)

def callback(data):
    # Uncomment the math below to treat theta as a quaternion. Currently just using w == theta 
    # x,y,z,w = data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
    # theta = 2.0*math.atan2(math.sqrt(x*x + y*y + z*z), w)
    theta = data.pose.orientation.w
    theta, r = [theta]*2, [0,pendulum_length]

    ax.cla()
    ax.set_theta_zero_location("S")
    ax.set_yticklabels([])
    ax.set_rmax(pendulum_length+1)

    ax.plot(theta, r, color='blue', lw=2)
    plt.pause(0.00001)

rospy.init_node('pendulum_viz', anonymous=True)
rospy.Subscriber("pendulum_x", ModelState, callback, queue_size=1)

plt.show(block=True)
rospy.spin()