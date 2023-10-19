import math

# Physical constants
m = 0.25
M = 1.0
l = 0.5
r = 0.05
g = 9.81

# State variables
theta = 0.01
theta_d = 0
Tau = 0

x_dd = 0
theta_dd = 0

# EoM
x_dd = (-M*g*l*math.sin(2*theta) - 2*M*l**2 *theta_d**2 *math.sin(theta) + 2*Tau*l*r - 2*Tau*math.cos(theta) ) / (l*(M*math.cos(2*theta) + 3*M + 2*m))

theta_dd_numerator = M**2 *g*l*math.sin(2*theta) - 0.5*M**2*l**2*theta_d**2*math.sin(theta) - 0.5*M**2*l**2*theta_d**2*math.sin(3*theta) + M*Tau*l*r*math.cos(2*theta) + M*Tau*l*r + 2*M*Tau*math.cos(theta) + M*g*l*m*math.sin(2*theta) + 2*Tau*m*math.cos(theta)
theta_dd_denominator = M*l**2*(M*math.cos(2*theta) + 3*M + 2*m)*math.cos(theta)

theta_dd = theta_dd_numerator / theta_dd_denominator