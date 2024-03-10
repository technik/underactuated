from math import *
from matplotlib import pyplot as plt
import numpy as np

#model
g = 9.81

Q_covariance = 1.0
R_covariance = 1.0

def linearF(X, dt):
    return np.array([[1, dt], [0, 0]])

def f(X, dt):
    return np.array([X[0] + X[1]*dt, X[1]]).T

def simulateStep(X, dt):
	gaussian_noise = np.array([np.random.normal(0, Q_covariance), np.random.normal(0, Q_covariance)])
	return f(X, dt) + gaussian_noise # assume perfect simulation for now

def linearH(X):
    theta = X[0]
    H = np.array([[g*sin(theta), 0],[g*cos(theta), 0], [0, 1]])

    return H

def h(X):
    theta = X[0]
    dtheta = X[1]

    return np.array([-g*cos(theta), g*sin(theta), dtheta])

def sensor(X):
    theta = X[0]
    dtheta = X[1]

    perfect_measurement = np.array([-g*cos(theta), g*sin(theta), dtheta]).T
    gaussian_noise = np.array([np.random.normal(0, R_covariance), np.random.normal(0, R_covariance), np.random.normal(0, R_covariance)])

    return perfect_measurement + gaussian_noise


# Initial state x, y, theta, derivatives...
dt = 0.01
x1, dx1 = 0,1

X = np.array([x1, dx1]).T

F = linearF(X, dt)

Q = np.eye(2) * Q_covariance
R = np.eye(3) * R_covariance

P = np.eye(2) * 10

num_steps = 100
Xs = np.zeros((2, num_steps))
Xs_actual = np.zeros((2,num_steps))
Ps = np.zeros((2,num_steps))
ts = np.zeros(num_steps)

# Initial state estimate, might be wrong
X_k_1 = np.array([1, 0])
P_k_1 = P

print("starting...")
for i, t in enumerate(ts):
    # Simulation
    X = simulateStep(X, dt)

    # Predict
    X_k = f(X_k_1, dt)

    F_k = linearF(X_k_1, dt)
    P_k = F_k @ P_k_1 @ F_k.T + Q

    # Update
    y_k = sensor(X) - h(X_k)

    H_k = linearH(X_k)
    S_k = H_k @ P_k @ H_k.T + R

    K_k = P_k @ H_k.T @ np.linalg.inv(S_k)

    X_k_k = X_k + K_k @ y_k

    P_k_k = (np.eye(2) - K_k @ H_k) @ P_k

    # Update vars for next loop
    X_k_1 = X_k_k
    P_k_1 = P_k_k

    # Update values for plotting
    ts[i] = i*dt
    Xs_actual[:,i] = X
    Xs[:,i] = X_k
    Ps[:,i] = np.array([P_k_1[0,0], P_k_1[1,1]])

print("Done!")
print(Ps.shape)

plt.figure(1)
plt.title("Position")
plt.plot(ts, Xs[0], 'r' , label='est. theta', linewidth=1)
plt.plot(ts, [Xs[0,i] + sqrt(Ps[0,i]) for i in range(num_steps)], 'r', label='1 sigma')
plt.plot(ts, [Xs[0,i] - sqrt(Ps[0,i]) for i in range(num_steps)], 'r', label='1 sigma')
plt.plot(ts, Xs_actual[0], 'g', label='actual theta', linewidth=2)

plt.figure(2)
plt.title("Velocity")
plt.plot(ts, Xs[1], 'r-', label='est. dtheta')
plt.plot(ts, Xs_actual[1], 'g', label='actual dtheta')

plt.show()