# Integrated Model Predictive Control and State Estimation for Shipboard Power and Propulsion Systems

A generic, reduced-order research framework for studying coordinated power
generation, thermal-state regulation, propulsion demand tracking, state
estimation, and supervisory load management under dynamic maritime loads.

The public implementation deliberately abstracts the energy source as a
first-order power-generation block with thermal dynamics. It is intended for
control-systems research and does not model reactor kinetics, protection
systems, fuel behavior, nuclear safety logic, or plant-specific operations.

## Implemented

- Reduced-order shipboard generation / thermal / shaft-speed dynamics
- Dynamic propulsion + hotel + auxiliary demand scenarios
- PID baseline
- LQR baseline
- Constrained receding-horizon MPC
- Discrete Kalman state estimator
- Supervisory low-priority load shedding
- Parameter-uncertainty Monte Carlo evaluation
- Performance metrics for power tracking, thermal excursion, control effort,
  settling behavior, and constraint violations

## Architecture

```text
Maritime Load Scenario
        ↓
Ship Power Demand
        ↓
Power Management Supervisor
        ↓
Reference / Load-Shedding Logic
        ↓
PID / LQR / MPC Controller
        ↓
Reduced-Order Power Plant
        ↓
Synthetic Sensors
        ↓
Kalman Filter
        ↓
Estimated State
        └───────────────→ Controller
```

## Scope

This repository is intentionally generic and non-operational. It does not
contain detailed reactor physics, reactor protection logic, fuel-cycle models,
plant procedures, real vessel data, or safety-critical operating parameters.
