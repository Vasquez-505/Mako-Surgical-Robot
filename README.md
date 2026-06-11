# Mako surgical robot

Lab project for the Robotic Manipulation course at Instituto Superior Técnico (IST), Master's in Mechanical Engineering, Specialization in Systems, Robotics and AI.

## Contributors
- Pedro Vasques ([@Vasquez-505](https://github.com/Vasquez-505))
- Abdul Hannan Chowdhury

## Project overview

Complete kinematic and dynamic modelling of the **Mako surgical manipulator by Stryker**, one of the leading orthopaedic surgical robots globally, implemented in MATLAB and Simulink.

## Part 1: Robot Kinematics (May 2025)

- Denavit-Hartenberg kinematic model: full DH parameter table and frame assignment for the 6-DOF manipulator
- Direct kinematics Simulink model, validated through known configurations and 3D VR visualization
- Closed-form inverse kinematics via kinematic decoupling, separating position and orientation subproblems
- Geometric Jacobian, computed symbolically and validated via finite differences (maximum error of 1×10⁻¹¹)
- Singularity detection: arm and wrist singularities identified via structured sampling across joint configurations
- Closed-Loop Inverse Kinematics (CLIK): iterative, numerically stable IK using Jacobian pseudo-inverse

## Part 2: Dynamics & Control (June 2025)

- Link physical parameter estimation: mass, center of mass, and inertia tensors for each link using geometric approximations
- Full dynamic model via Newton-Euler formulation, with symbolic torque computation in MATLAB exported as a numeric function and integrated as a Simulink block
- Worst-case inertia configuration analysis: joint space inertia matrix computed symbolically to identify maximum load conditions
- Decentralized PID joint controllers, individually tuned per joint using worst-case inertia, settling within 1.3s with minimal overshoot
- Centralized inverse dynamics controller with full nonlinear compensation via computed torque control law, settling within 0.7s
- Task-space trajectory tracking: end-effector moved from p_A=[0.2, 0.1, 0.2]m to p_B=[0.6, 0.1, 0.2]m using cubic polynomial trajectories, comparing both controllers

## Key results

| Controller | Settling Time | Overshoot |
|---|---|---|
| Decentralized PID | ~1.3s | Minimal |
| Centralized Inverse Dynamics | ~0.7s | Minimal |

Jacobian validation error: < 1×10⁻¹¹ across all matrix entries

## Tech stack

MATLAB · Simulink · Symbolic Math Toolbox · Robotics Toolbox
