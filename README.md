# Integrated Model Predictive Control and State Estimation for Shipboard Power and Propulsion Systems

A research-oriented simulation framework for **integrated modelling, state estimation, power management, and advanced control of shipboard power systems under dynamic maritime loads**.

The project investigates how a shipboard power-generation system can respond to rapidly changing propulsion and electrical power demands while maintaining stable system behavior, limiting thermal deviations, and respecting control constraints.

Three control approaches are considered:

- PID control
- Linear Quadratic Regulation (LQR)
- Model Predictive Control (MPC)

A discrete Kalman filter is additionally used for state estimation, while a supervisory power-management layer handles prioritized electrical loads.

The overall framework is:

```text
Dynamic Maritime Loads
        ↓
Ship Power Demand
        ↓
Power Management
        ↓
PID / LQR / MPC
        ↓
Reduced-Order Power Plant
        ↓
Synthetic Sensors
        ↓
Kalman Filter
        ↓
State Estimate
        └──────────────→ Controller
```

> **Note:** The public implementation uses a generic reduced-order power-generation model. It does not reproduce a specific nuclear reactor, vessel, propulsion plant, or safety-critical energy system.

---

# 1. Motivation

Modern ships may contain tightly coupled propulsion and electrical power systems.

The total power demand can be represented as:

```text
P_demand =
P_propulsion
+
P_critical
+
P_navigation
+
P_auxiliary
+
P_hotel
```

Unlike a stationary power system, shipboard demand can change significantly due to:

- Propulsion changes
- Maneuvering
- Environmental disturbances
- Auxiliary-system operation
- Hotel loads
- Load-shedding events

The generation system must therefore continuously adapt to changing demand.

This creates a dynamic control problem involving:

```text
Power Tracking
      +
Thermal Regulation
      +
Control Constraints
      +
State Estimation
      +
Load Management
```

---

# 2. System Architecture

The simulated architecture is:

```text
        Maritime Operating Scenario
                   |
                   v
           Ship Power Demand
                   |
                   v
        Power Management Layer
                   |
                   v
          Controller Reference
                   |
        +----------+----------+
        |          |          |
        v          v          v
       PID        LQR        MPC
        |          |          |
        +----------+----------+
                   |
                   v
        Reduced-Order Plant
                   |
                   v
          Synthetic Sensors
                   |
                   v
           Kalman Filter
                   |
                   v
          Estimated States
                   |
                   +----------→ Controller
```

This architecture separates:

```text
Load Generation
       ↓
Power Management
       ↓
Control
       ↓
Plant Dynamics
       ↓
Sensing
       ↓
State Estimation
       ↓
Feedback
```

---

# 3. Reduced-Order Dynamic Model

The current implementation uses a generic low-order model containing three states:

```text
x =
[
P
ΔT
Δω
]
```

where:

```text
P   = normalized generated power
ΔT  = normalized thermal-state deviation
Δω  = normalized shaft-speed deviation
```

The model is intentionally simplified to focus on dynamic-system analysis and controller design.

---

# 4. Power Generation Dynamics

The generated-power state is represented by a first-order dynamic model:

```text
dP/dt =
(P_cmd - P) / τ_P
```

where:

```text
P_cmd = commanded power
P     = generated power
τ_P   = power-response time constant
```

This represents the finite response rate of the generation system.

---

# 5. Thermal Dynamics

A generic thermal state is included to capture the relationship between generated power and demanded power.

The model is:

```text
d(ΔT)/dt =
[P - P_demand - h ΔT] / C_T
```

where:

```text
C_T = equivalent thermal capacity
h   = equivalent thermal-loss coefficient
```

If generated power and demand are unbalanced, the thermal state changes accordingly.

---

# 6. Shaft / Propulsion Dynamics

A reduced-order shaft-speed deviation state is represented as:

```text
d(Δω)/dt =
[P - P_demand - Δω] / τ_ω
```

where:

```text
τ_ω = equivalent shaft-response time constant
```

This provides a simple representation of the dynamic interaction between available power and propulsion demand.

---

# 7. State-Space Representation

The complete system can be expressed as:

```text
x_dot =
A x
+
B u
+
E d
```

where:

```text
x = system-state vector
u = power command
d = ship power demand
A = system matrix
B = control-input matrix
E = disturbance-input matrix
```

Measurements are represented as:

```text
y = Cx + v
```

where:

```text
v = measurement noise
```

This formulation provides a common basis for LQR, MPC, and Kalman-filter design.

---

# 8. Dynamic Maritime Load Scenarios

Several generic load profiles are implemented.

## Cruise

A constant power demand represents approximately steady operation:

```text
Power
  |
  | ─────────────────
  |
  +--------------------> Time
```

---

## Acceleration

A step increase represents increasing propulsion demand:

```text
Power
  |
  |          ┌──────────
  |          |
  |──────────┘
  |
  +--------------------> Time
```

---

## Heavy-Sea-Like Disturbance

A combination of sinusoidal components produces continuously varying demand:

```text
P_demand(t) =
P_0
+
A_1 sin(ω_1 t)
+
A_2 sin(ω_2 t)
```

This provides a generic fluctuating-load scenario.

---

## Berthing-Like Maneuver

Multiple power transitions are applied:

```text
Low
 ↓
High
 ↓
Low
```

This tests controller behavior during repeated demand changes.

---

## Load-Shedding Event

A sudden decrease in electrical demand is introduced to evaluate transient response.

---

# 9. PID Controller

PID provides the first control baseline.

The power-tracking error is:

```text
e(t) =
P_demand(t) - P(t)
```

The controller is:

```text
u(t) =
P_demand
+
Kp e(t)
+
Ki ∫e(t)dt
+
Kd de(t)/dt
```

The resulting command is bounded to the normalized operating interval:

```text
0 <= u <= 1
```

PID provides a conventional reference for evaluating the model-based controllers.

---

# 10. Linear Quadratic Regulator

The second baseline is LQR.

The controller minimizes:

```text
J =
∫ [
xᵀQx
+
uᵀRu
] dt
```

The optimal feedback law is:

```text
u =
u_ref
-
K(x - x_ref)
```

where the reference state corresponds approximately to:

```text
Generated Power = Demand
Thermal Deviation = 0
Shaft-Speed Deviation = 0
```

LQR therefore considers multiple dynamic states simultaneously.

---

# 11. Model Predictive Control

The main advanced controller is a finite-horizon Model Predictive Controller.

At each simulation step, MPC predicts future plant behavior over a finite horizon.

The optimization objective includes:

```text
J =
Σ [
w_P (P - P_demand)²
+
w_T ΔT²
+
w_ω Δω²
+
w_u Δu²
]
```

This allows MPC to balance:

- Power tracking
- Thermal-state regulation
- Shaft-speed regulation
- Smooth control action

---

# 12. MPC Constraints

The current implementation explicitly constrains the normalized control input:

```text
0 <= u <= 1
```

Therefore, the optimization problem becomes:

```text
Minimize:
    Tracking Error
    +
    Thermal Deviation
    +
    Shaft Deviation
    +
    Control Variation

Subject to:
    Control Bounds
```

The first optimized input is applied to the plant.

The optimization is repeated at every time step.

```text
Current State Estimate
        ↓
Predict Future States
        ↓
Optimize Control Sequence
        ↓
Apply First Command
        ↓
New Measurements
        ↓
Repeat
```

---

# 13. Why MPC?

Shipboard power systems may operate under rapidly varying loads and physical constraints.

MPC is attractive because it explicitly combines:

```text
Dynamic Model
      +
Prediction
      +
Optimization
      +
Constraints
```

Unlike a purely reactive controller, MPC evaluates the predicted consequences of candidate control actions.

---

# 14. Synthetic Sensor Model

The controller does not need to rely directly on perfect simulated ground-truth information.

Synthetic measurements are generated as:

```text
y =
x
+
v
```

where:

```text
v = Gaussian measurement noise
```

Noise is applied to the simulated:

- Power measurement
- Thermal-state measurement
- Shaft-state measurement

---

# 15. Kalman State Estimation

A discrete Kalman filter estimates the system states.

The prediction step is:

```text
x_hat(k|k-1) =
A_d x_hat(k-1)
+
B_d u(k-1)
```

The covariance prediction is:

```text
P(k|k-1) =
A_d P(k-1) A_dᵀ
+
Q
```

After receiving the measurement:

```text
y(k)
```

the Kalman gain is calculated:

```text
K =
P Cᵀ
(C P Cᵀ + R)^-1
```

The estimate is updated using:

```text
x_hat(k) =
x_hat(k|k-1)
+
K [
y(k)
-
C x_hat(k|k-1)
]
```

The estimated state is then supplied to the controller.

---

# 16. Estimation-Control Architecture

The resulting feedback loop becomes:

```text
Power Plant
     ↓
Synthetic Sensors
     ↓
Noisy Measurements
     ↓
Kalman Filter
     ↓
Estimated State
     ↓
PID / LQR / MPC
     ↓
Control Command
     ↓
Power Plant
```

This creates an integrated **estimation and control architecture**.

---

# 17. Ship Power Management

The project includes a generic supervisory power-management layer.

Electrical demand is divided into several categories:

```text
Critical Loads
      ↓
Propulsion
      ↓
Navigation
      ↓
Auxiliary
      ↓
Hotel Loads
```

The system assigns available power according to this priority order.

---

# 18. Load Prioritization

The generic priority hierarchy is:

```text
1. Critical Loads
2. Propulsion
3. Navigation
4. Auxiliary Systems
5. Hotel Loads
```

When:

```text
Available Power
<
Requested Power
```

lower-priority loads receive reduced allocation first.

This provides a simple supervisory load-management example.

---

# 19. Integrated Power Management

The complete supervisory-control structure is:

```text
Total Ship Demand
        ↓
Load Decomposition
        ↓
Priority Manager
        ↓
Available Power Check
        ↓
Served Demand
        ↓
Controller Reference
        ↓
PID / LQR / MPC
```

This allows power-generation control and shipboard demand management to be studied within the same simulation framework.

---

# 20. Performance Metrics

The controllers can be evaluated using several quantitative metrics.

## Power Tracking RMSE

```text
RMSE_P =
sqrt(
mean(
(P - P_demand)²
)
)
```

---

## Peak Power Error

```text
e_peak =
max |P - P_demand|
```

---

## Peak Thermal Deviation

```text
ΔT_peak =
max |ΔT|
```

---

## Control Effort

```text
E_u =
Σ u²
```

---

## Control-Rate Effort

```text
E_Δu =
Σ (Δu)²
```

This evaluates how aggressively the controller changes the power command.

---

## Constraint Violations

The simulation checks whether:

```text
u < 0
```

or:

```text
u > 1
```

occurs.

With correct input saturation or constrained MPC, the expected value should remain zero.

---

# 21. Controller Comparison

The framework supports comparison of:

| Method | Feedback | State-Space Model | Prediction | Explicit Input Constraints |
|---|---:|---:|---:|---:|
| PID | ✓ | — | — | Saturation |
| LQR | ✓ | ✓ | — | Saturation |
| MPC | ✓ | ✓ | ✓ | ✓ |

The comparison can be performed across multiple maritime load scenarios.

---

# 22. Example Evaluation Matrix

The intended experimental structure is:

```text
                Cruise
                   |
                   v
           PID / LQR / MPC

             Acceleration
                   |
                   v
           PID / LQR / MPC

              Heavy Sea
                   |
                   v
           PID / LQR / MPC

              Berthing
                   |
                   v
           PID / LQR / MPC

          Load-Shedding Event
                   |
                   v
           PID / LQR / MPC
```

This allows each controller to be tested under the same demand conditions.

---

# 23. Repository Structure

```text
shipboard_integrated_power_mpc/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── docs/
│   └── architecture.md
│
├── src/
│   ├── __init__.py
│   ├── model.py
│   ├── scenarios.py
│   ├── power_management.py
│   ├── estimators.py
│   ├── controllers.py
│   ├── simulation.py
│   └── evaluation.py
│
└── examples/
    ├── run_controller_comparison.py
    ├── plot_mpc_demo.py
    └── monte_carlo.py
```

---

# 24. Module Description

| Module | Purpose |
|---|---|
| `model.py` | Reduced-order shipboard power-system dynamics |
| `scenarios.py` | Dynamic maritime power-demand profiles |
| `power_management.py` | Priority-based load allocation |
| `estimators.py` | Discrete Kalman state estimator |
| `controllers.py` | PID, LQR and MPC controllers |
| `simulation.py` | Integrated closed-loop simulation |
| `evaluation.py` | Quantitative performance metrics |
| `run_controller_comparison.py` | Controller/scenario comparison |
| `plot_mpc_demo.py` | Representative MPC visualization |
| `monte_carlo.py` | Repeated stochastic evaluation |

---

# 25. Installation

Clone the repository:

```bash
git clone <repository-url>
cd shipboard-integrated-power-control
```

Install the dependencies:

```bash
pip install -r requirements.txt
```

Main dependencies:

```text
NumPy
SciPy
Matplotlib
```

---

# 26. Running the Controller Comparison

Run:

```bash
python examples/run_controller_comparison.py
```

The script evaluates:

```text
PID
LQR
MPC
```

under:

```text
Cruise
Acceleration
Heavy-Sea-Like Loading
Berthing-Like Loading
Load-Shedding Event
```

and reports the corresponding performance metrics.

---

# 27. Running the MPC Demonstration

Run:

```bash
python examples/plot_mpc_demo.py
```

The demonstration compares:

```text
Demanded Power
Generated Power
Estimated Power
```

under a dynamically varying maritime load.

---

# 28. Monte Carlo Evaluation

The repository also contains a stochastic evaluation example:

```bash
python examples/monte_carlo.py
```

Repeated simulations are performed using different sensor-noise realizations.

The resulting metrics are summarized using:

```text
Mean
+
Standard Deviation
```

This provides an initial assessment of estimator/controller sensitivity to measurement uncertainty.

---

# 29. Recommended Result Figures

Once the simulations are executed, representative figures can be stored as:

```text
results/
├── power_tracking_pid_lqr_mpc.png
├── heavy_sea_tracking.png
├── thermal_response.png
├── kalman_estimation.png
├── control_input_comparison.png
└── load_management.png
```

Only results produced by the actual simulation should be included.

---

# 30. Recommended Result Table

A final controller comparison can use:

| Controller | Power RMSE | Peak Error | Peak Thermal Deviation | Control Effort | Violations |
|---|---:|---:|---:|---:|---:|
| PID | measured | measured | measured | measured | measured |
| LQR | measured | measured | measured | measured | measured |
| MPC | measured | measured | measured | measured | measured |

No assumed or fabricated numerical values are required.

---

# 31. Technologies

- Python
- NumPy
- SciPy
- Matplotlib
- Dynamic System Modelling
- State-Space Methods
- PID Control
- Linear Quadratic Regulation
- Model Predictive Control
- Kalman Filtering
- Numerical Optimization
- Monte Carlo Simulation

---

# 32. Research Areas

The project is related to:

- Shipboard Power Systems
- Marine Energy Systems
- Integrated Power and Propulsion
- Dynamic System Modelling
- Advanced Control Systems
- Model Predictive Control
- State Estimation
- Power Management
- Maritime Systems
- Simulation-Based Engineering

---

# 33. Current Implementation

The current public implementation includes:

- Generic reduced-order power-generation dynamics
- Thermal-state dynamics
- Shaft-speed dynamics
- State-space representation
- Dynamic maritime load scenarios
- PID controller
- LQR controller
- Constrained MPC
- Synthetic measurement noise
- Discrete Kalman filter
- Priority-based power management
- Load-shedding logic
- Power-tracking metrics
- Thermal-performance metrics
- Control-effort metrics
- Repeated stochastic evaluation

---

# 34. Current Limitations

The current model intentionally does **not** include:

- Detailed nuclear reactor kinetics
- Neutron-transport models
- Fuel thermal models
- Reactor protection systems
- Safety-system logic
- Plant-specific operating procedures
- Detailed steam-cycle thermodynamics
- High-fidelity turbine models
- Electrical network transient models
- Real shipboard reactor parameters
- Real vessel operational data
- Hardware-in-the-loop validation

Therefore, this repository should be interpreted as a **control-oriented reduced-order shipboard power-system simulation**, not as a validated nuclear power-plant simulator.

---

# 35. Future Extensions

Future research can extend the framework toward higher-fidelity energy-system modelling.

Possible directions include:

### Higher-Fidelity Plant Models

```text
Reduced-Order Model
        ↓
Higher-Fidelity Thermal Model
        ↓
Turbine / Generator Dynamics
        ↓
Electrical Network
        ↓
Integrated Propulsion Model
```

### Advanced State Estimation

Possible extensions include:

- Extended Kalman Filter
- Unscented Kalman Filter
- Moving Horizon Estimation
- Sensor-fault detection

### Advanced MPC

The current MPC could be extended with:

- State constraints
- Power ramp-rate constraints
- Thermal constraints
- Explicit load-shedding optimization
- Economic MPC
- Robust MPC
- Stochastic MPC

### Parameter Uncertainty

Physical parameters could be randomized:

```text
τ = τ_0 (1 + δ)
```

```text
C_T = C_T0 (1 + δ)
```

```text
h = h_0 (1 + δ)
```

to evaluate robustness under modelling uncertainty.

### High-Fidelity Model Validation

A future research version could compare the reduced-order controller model against an independently validated high-fidelity simulation or experimental dataset.

---

# 36. Public Implementation Notice

This repository contains a **generic and sanitized research implementation** for dynamic-system modelling, state estimation, power management, and control studies.

The public implementation intentionally excludes:

- Platform-specific reactor information
- Detailed reactor physics
- Nuclear safety-system configurations
- Operational procedures
- Restricted vessel parameters
- Real plant-control parameters
- Real operational coordinates
- Proprietary propulsion-system data
- Confidential datasets

All parameters and scenarios are generic simulation examples.

---

# 37. Status

**Research-oriented simulation framework / active development**

The current project demonstrates the integrated pipeline:

```text
Dynamic Maritime Demand
        ↓
Power Management
        ↓
State Estimation
        ↓
PID / LQR / MPC
        ↓
Reduced-Order Dynamic Plant
        ↓
Performance Evaluation
```

The primary focus is on **control-system architecture, dynamic modelling, state estimation, constrained optimization, and integrated shipboard power management**.

---

# Author

**Mehmet Ateş**

Research interests:

- Autonomous and Control Systems
- Guidance, Navigation and Control
- Marine Systems
- Shipboard Power Systems
- Model Predictive Control
- State Estimation
- Dynamic System Modelling
- Power Management
- Reinforcement Learning
- Simulation-Based Engineering
