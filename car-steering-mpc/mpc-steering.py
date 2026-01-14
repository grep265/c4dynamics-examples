import numpy as np
import cvxpy as cp
import c4dynamics as c4d
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Model parameters
V = 10.0 # m/s

#Initial states
psi0 = 0 # rad
y0 = 1 # m

# Simulation parameters
dt = 0.1 # s
t_end = 5.0 # s

# Controller parameters
N = 5 # horizons
Q = np.diag([1.0, 10.0]) # [psi, y]
R = np.array([[0.1]])
u_max = 0.5 # rad/s

# states: [psi, y]
def vehicle_model(t, x, delta, V=V):
    psi, y = x
    return [delta, V * psi]

# Discrete model
A_d = np.array([
    [1.0,      0.0],
    [V * dt,   1.0]
])

B_d = np.array([
    [dt],
    [0.5 * V * dt**2]
])

# Initial state
vehicle = c4d.state(psi=psi0, y=y0)

def solve_mpc(current_x):
    x = cp.Variable((2, N + 1))
    u = cp.Variable((1, N))
    cost = 0
    constraints = [x[:, 0] == current_x]

    for k in range(N):
        cost += cp.quad_form(x[:, k], Q) + cp.quad_form(u[:, k], R)
        constraints += [
            x[:, k+1] == A_d @ x[:, k] + B_d @ u[:, k],
            cp.abs(u[:, k]) <= u_max
        ]

    cost += cp.quad_form(x[:, N], Q)
    cp.Problem(cp.Minimize(cost), constraints).solve(solver=cp.OSQP)

    return u.value[0, 0]

# Simulation loop
for ti in np.arange(0, t_end, dt):
    delta = solve_mpc(vehicle.X)
    sol = solve_ivp(vehicle_model, [ti, ti + dt], vehicle.X, args=(delta,))
    vehicle.X = sol.y[:, -1]
    vehicle.store(ti)

# Plots
vehicle.plot('y', darkmode=False)
plt.title('Lateral Position')
vehicle.plot('psi', darkmode=False)
plt.title('Heading Angle')
plt.show()