import math
import matplotlib.pyplot as plt
import numpy as np
import time

# State: x, theta, x_d, theta_d

class SegwaySimulator:
    def __init__(self, state = [0,0,0,0], dt=0.01):
        self.state = state
        self.dt = dt

        # Physical constants
        self.m = 0.1
        self.M = 2.0
        self.l = 0.5
        self.r = 0.05
        self.g = 9.81

    def f(self, state, u):
        [x, theta, x_d, theta_d] = state
        Tau = u

        x_prime = x_d
        theta_prime = theta_d

        x_dd = (-self.M*self.g*self.l*math.sin(2*theta) - 2*self.M*self.l**2 *theta_d**2 *math.sin(theta) + 2*Tau*self.l*self.r - 2*Tau*math.cos(theta) ) / (self.l*(self.M*math.cos(2*theta) + 3*self.M + 2*self.m))
        x_d_prime = x_dd

        theta_dd_numerator = self.M**2 *self.g*self.l*math.sin(2*theta) - 0.5*self.M**2*self.l**2*theta_d**2*math.sin(theta) - 0.5*self.M**2*self.l**2*theta_d**2*math.sin(3*theta) + self.M*Tau*self.l*self.r*math.cos(2*theta) + self.M*Tau*self.l*self.r + 2*self.M*Tau*math.cos(theta) + self.M*self.g*self.l*self.m*math.sin(2*theta) + 2*Tau*self.m*math.cos(theta)
        theta_dd_denominator = self.M*self.l**2*(self.M*math.cos(2*theta) + 3*self.M + 2*self.m)*math.cos(theta)
        theta_dd = theta_dd_numerator / theta_dd_denominator

        theta_d_prime = theta_dd

        return np.array([x_prime, theta_prime, x_d_prime, theta_d_prime])

    def stepSimulation(self, u):
        dt = self.dt

        k1 = self.f(self.state, u)
        k2 = self.f(self.state + (k1*(dt/2.0)), u)
        k3 = self.f(self.state + (k2*(dt/2.0)), u)
        k4 = self.f(self.state + k3*dt, u)

        self.state = self.state + (k1 + k2*2.0 + k3*2.0 + k4) * (dt/6.0)

class PID:
    def ctrl(self, state, setpoint=np.array([0,0,0,0])):
        kp_theta = 100
        kd_theta = 10

        e = setpoint - state
        return e[1]*kp_theta + e[3]*kd_theta

class LQR:
    def __init__(self):
        self.K = np.array([1, 8.83817682570448, 0, 2.397848383185352])

    def ctrl(self, state, setpoint=np.array([0,0,0,0])):
        e = setpoint - state
        return np.matmul(self.K, e)

class VizConsole:
    fig = plt.figure()
    ax0 = fig.add_subplot(211, aspect='equal', autoscale_on=True, xlim=(-2, 2), ylim=(0,1))
    ax1 = fig.add_subplot(212)

    count = 0

    prev_callback_time = 0

    def __init__(self, sim, plot_U_every_n = 1, history_sec=1, data_rate_Hz=100):

        self.sim = sim

        self.plot_U_every_n = plot_U_every_n
        self.data_rate_Hz = data_rate_Hz
        self.history_sec = history_sec
        self.history_len = history_sec*data_rate_Hz//plot_U_every_n

        self.time_ticks = np.linspace(0, 1, self.history_len)
        self.U_history = [0]*self.history_len
        self.theta_history = [0]*self.history_len
        self.xd_history = [0]*self.history_len

        # Visual params of segway
        self.L = 0.5
        self.r = 0.1
        self.M = 0.25


    def viz_pendulum(self, x, theta):
        
        self.ax0.cla()
        self.ax0.set_yticklabels([])

        x0, y0 = x, 0
        x1, y1 = x + self.L*np.sin(theta), self.L*np.cos(theta)

        # Draw circle centered at x0, y0
        self.ax0.add_patch(plt.Circle((x0, y0), self.r, color='black', clip_on=False))

        # Draw line from x0, y0 to x1, y1
        self.ax0.plot([x0, x1], [y0, y1], color='black', lw=3)

        # Draw circle centered at x1, y1
        self.ax0.add_patch(plt.Circle((x1, y1), self.M, color='darkorange', clip_on=False))

        # Draw the ground
        self.ax0.plot([-2,2], [-self.r,-self.r], color='darkgreen', lw=1, linestyle='--')

        # Set the limits of the plot
        self.ax0.set_ylim(-0.01, 2*self.L)


    def viz_timeseries(self, u):
        if self.count % self.plot_U_every_n != 0:
            return

        self.U_history = self.U_history[1:] + [u]
        self.theta_history = self.theta_history[1:] + [self.sim.state[1]]
        self.xd_history = self.xd_history[1:] + [self.sim.state[2]]

        self.ax1.cla()
        self.ax1.plot(self.time_ticks, self.U_history, label='U', color='royalblue')
        self.ax1.plot(self.time_ticks, self.theta_history, label='theta', color='purple')
        self.ax1.plot(self.time_ticks, self.xd_history, label='x_d', color='darkgreen')
        self.ax1.legend(loc='upper left')
    
    def draw(self):
        plt.pause(0.00001)

    def callback(self, state, u):

        # Get the state
        x = state[0]
        theta = state[1]

        # Run visualizations
        viz.viz_pendulum(x, theta)
        viz.viz_timeseries(u)
        viz.draw()

        self.count += 1
        
        current_time = time.time()
        print(f"Ran at {1/(current_time-self.prev_callback_time):.2f} Hz   ", end="\r")
        self.prev_callback_time = time.time()

    
sim = SegwaySimulator([0, 0.1, 0, 0])
controller = PID()
viz = VizConsole(sim)

u = 0
while True:
    if abs(sim.state[1]) > math.pi/2:
        break # It fell down

    u = controller.ctrl(sim.state)
    viz.callback(sim.state, u)
    sim.stepSimulation(u)