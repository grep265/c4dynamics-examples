# Vehicle steering control using MPC

This project implements a Model Predictive Controller (MPC) for the steering control of a vehicle using the [c4dynamics](https://github.com/c4dynamics/c4dynamics) library.

The goal is to control the lateral position **(y)** and heading angle **(ψ)** of a vehicle to track a straight line using steering input **(δ)**.

---

## System Model

We consider a simplified kinematic model of a vehicle with 2 states `[ψ ; y]`:

$\dot{\psi}(t) = \delta$(t)

$\dot{y}(t) = V\psi$(t)
  
where:  
- `y` = lateral position [m]  
- `ψ` = heading angle [rad]  
- `V` = velocity [m/s]  
- `δ` = steering input [rad/s]  

---

## Model Predictive Controller (MPC)

The vehicle steering is controlled using a Model Predictive Controller. The MPC computes the optimal steering input `δ` over a finite prediction horizon to regulate both the lateral position `y` and heading angle `ψ` of the vehicle.

### MPC Design

1. **Prediction Horizon:**  
   The controller looks ahead `N` steps to predict future vehicle states.

2. **Cost Function:**  
   The MPC minimizes a quadratic cost function combining state deviations and control effort:

   $J = \sum_{k=0}^{N-1} \left( x_k^\top Q x_k + u_k^\top R u_k \right) + x_N^\top Q x_N$
   
   where:  
   - $x_k = [\psi_k, y_k]^\top$ is the state vector at step $k$  
   - $u_k = \delta_k$ is the steering input  
   - $Q$ penalizes heading and lateral deviation  
   - $R$ penalizes control effort  

3. **Constraints:**  
   - Steering input is bounded: $|\delta| \le 0.2 \text{ rad/s}$ 

4. **Discrete Vehicle Model:**  

   The continuous-time kinematic model:

   $\dot{\psi}(t) = \delta(t), \quad \dot{y}(t) = V \psi(t)$
   
   is discretized with timestep `dt = 0.1 s`:

   $\begin{aligned}
   \psi_{k+1} &= \psi_k + \delta_k \, dt \\
   y_{k+1} &= y_k + V \psi_k \, dt + 0.5 V \, dt^2 \, \delta_k
   \end{aligned}$

### Implementation Details

- The MPC problem is formulated and solved using **CVXPY** with the **OSQP** solver.
- At each simulation step:
  1. The current state is measured.
  2. The MPC computes the optimal steering input for the next timestep.
  3. The input is applied to the vehicle using `solve_ivp` to integrate the dynamics.
  4. The state is stored for plotting and analysis.

---

## Pseudocode

```text
Import required libraries: c4dynamics, numpy, matplotlib, etc

Define model parameters: V
Define initial states: y0, ψ0
Define simulation parameters: dt, t_end
Define controller parameters: N, Q, R
Define constraints: delta_max

Define vehicle model function: vehicle_model()
Define MPC function: A & B matrices, solver, etc

Initialize vehicle state object: y0, ψ0

Main loop:
    Store current state: vehicle.store
    Compute optimal steering using MPC:
    Apply control input to vehicle: vehicle.X = solve_ivp or discrete model
    Update vehicle state:
        
Plot: lateral position (y), heading (ψ) and control input (δ)
```
---

## Results

- Lateral position (`y`) is driven from the initial offset (`y0 = 1.0 m`) toward zero; the controller reduces the lateral error within a few seconds and stabilizes near the reference.

![](images/lateral_position.png)

- Heading angle (`psi`) shows a transient response as the controller rotates the vehicle to correct lateral error, then converges toward small values once `y` is near zero.

![](images/heading_angle.png)

- Steering input (`δ`) exhibits an initial corrective action (bounded by `u_max`) and then settles close to zero as the vehicle reaches the desired lateral position.

![](images/mpc_steering_input.png)

These results indicate the MPC successfully commands the steering rate to remove the initial lateral offset while respecting the input bound.

---

## References

[MPC - Standford lecture](https://web.stanford.edu/class/archive/ee/ee392m/ee392m.1056/Lecture14_MPC.pdf)