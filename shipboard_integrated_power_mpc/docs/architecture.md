# Architecture

```text
Dynamic Maritime Demand
        ↓
Power Management Supervisor
        ↓
Controller Target
        ↓
PID / LQR / MPC
        ↓
Generic Reduced-Order Plant
        ↓
Synthetic Sensors
        ↓
Kalman State Estimation
        ↓
Feedback
```

The model is deliberately abstracted to support control research without
encoding plant-specific nuclear-system behavior.
