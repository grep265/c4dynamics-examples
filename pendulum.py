from matplotlib import pyplot as plt 
import c4dynamics as c4d
import numpy as np 
from scipy.integrate import solve_ivp

# simulation parameters
dt = 0.01   # s    
t_end = 5.0 # s
L = 1.0 # m
g = 9.8 # m/s^2
theta_init = 50 # deg

def pendulum_model(t, y, L, g):
    theta, q = y
    return [q, -(g/L) * c4d.sin(theta)]

pendulum = c4d.state(theta= theta_init * c4d.d2r, q=0)

for ti in np.arange(0, t_end, dt):
    pendulum.X = solve_ivp(pendulum_model, [ti, ti + dt], pendulum.X, args=(L, g)).y[:, -1]
    pendulum.store(ti)

pendulum.plot('theta', scale=c4d.r2d, darkmode=False)
plt.ylabel('θ [deg]')
plt.xlabel('Time [s]')
plt.title('Pendulum simulation')
plt.show()
