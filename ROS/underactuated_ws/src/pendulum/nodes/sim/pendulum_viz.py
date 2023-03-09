#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
import math
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelState
import numpy as np
import time

class Pendulum:
    l1 = 1
    m1 = 1

    g = 9.81

class VizConsole:
    fig = plt.figure()
    ax0 = fig.add_subplot(211, polar=True)
    ax1 = fig.add_subplot(212)

    pendulum = Pendulum()

    count = 0

    prev_callback_time = 0

    def __init__(self, plot_E_every_n = 1, history_sec=1, data_rate_Hz=100):
        self.plot_E_every_n = plot_E_every_n
        self.data_rate_Hz = data_rate_Hz
        self.history_sec = history_sec
        self.history_len = history_sec*data_rate_Hz//plot_E_every_n

        self.time_ticks = np.linspace(0, 1, self.history_len)
        self.E_history = [0]*self.history_len
        self.T_history = [0]*self.history_len
        self.V_history = [0]*self.history_len


    def viz_pendulum(self, theta):
        theta, r = [theta]*2, [0,self.pendulum.l1]

        self.ax0.cla()
        self.ax0.set_theta_zero_location("S")
        self.ax0.set_yticklabels([])
        self.ax0.set_rmax(self.pendulum.l1+1)

        self.ax0.plot(theta, r, color='darkorange', lw=3)

    def viz_energy(self, theta, dTheta):
        if self.count % self.plot_E_every_n != 0:
            return

        T = 0.5 * self.pendulum.m1 * self.pendulum.l1 * self.pendulum.l1 * dTheta * dTheta
        V = self.pendulum.m1 * self.pendulum.g * self.pendulum.l1 * -math.cos(theta) + self.pendulum.m1 * self.pendulum.g * self.pendulum.l1
        E = T + V


        self.E_history = self.E_history[1:] + [E]
        self.T_history = self.T_history[1:] + [T]
        self.V_history = self.V_history[1:] + [V]

        self.ax1.cla()
        # self.ax1.set_ylim([0,10])
        self.ax1.plot(self.time_ticks, self.E_history, label='E', color='royalblue')
        self.ax1.plot(self.time_ticks, self.T_history, label='T', color='dodgerblue', alpha=0.6)
        self.ax1.plot(self.time_ticks, self.V_history, label='V', color='indigo', alpha=0.6)
        self.ax1.legend(loc='upper left')
    
    def draw(self):
        plt.pause(0.00001)

    def callback(self, data):

        # Uncomment the math below to treat theta as a quaternion. Currently just using w == theta 
        # x,y,z,w = data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
        # theta = 2.0*math.atan2(math.sqrt(x*x + y*y + z*z), w)
        theta = data.pose.orientation.w
        dTheta = data.twist.angular.z

        viz.viz_pendulum(theta)
        viz.viz_energy(theta, dTheta)
        viz.draw()

        self.count += 1
        
        current_time = time.time()
        print(f"Ran at {1/(current_time-self.prev_callback_time):.2f} Hz   ", end="\r")
        self.prev_callback_time = time.time()

    

viz = VizConsole()

rospy.init_node('pendulum_viz', anonymous=True)
rospy.Subscriber("pendulum_x", ModelState, viz.callback, queue_size=1)

plt.show(block=True)
rospy.spin()