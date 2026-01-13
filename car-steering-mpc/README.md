# Lateral MPC with c4dynamics

This project implements a Model Predictive Controller (MPC) for the steering control of a vehicle using the [c4dynamics](https://github.com/c4dynamics/c4dynamics) library.

The goal is to control the lateral position **(y)** and heading angle **(ψ)** of a vehicle to track a straight line using steering input **(δ)**.

---

## System Model

We consider a simplified kinematic model of a vehicle with 2 states `[y ; ψ]`:

`\[\dot{y} = V * \psi\]`

`\[\dot{\psi} = \delta\]`
  
where:  
- `\(y)` = lateral position [m]  
- `\(\psi\)` = heading angle [rad]  
- `\(V\)` = velocity [m/s]  
- `\(\delta\)` = steering input [rad/s]  

---

## Pseudocode

```text
Import required libraries: c4dynamics, numpy, matplotlib, etc

Define model parameters: V
Define initial states: y0, psi0
Define simulation parameters: dt, t_end
Define controller parameters: N, Q, R
Define constraints: delta_max

Define vehicle model function: vehicle_model(t,x,delta)
Define MPC function:

Initialize vehicle state object: y0, psi0

Main loop:
    Store current state: vehicle.store
    Compute optimal steering using MPC:
    Apply control input to vehicle: vehicle.X = solve_ivp
    Update vehicle state:
        
Plot: lateral position (y), heading (psi) and control input (delta)
```
---

## References

[MPC - Standford lecture](https://web.stanford.edu/class/archive/ee/ee392m/ee392m.1056/Lecture14_MPC.pdf)