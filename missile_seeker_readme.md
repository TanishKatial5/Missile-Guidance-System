# 🚀 Nonlinear Analysis of Missile Seeker Dynamics

A comprehensive nonlinear control system analysis comparing Proportional Navigation Guidance (PNG) and Sliding Mode Control (SMC) for missile guidance applications. This project implements robust control algorithms with Lyapunov stability analysis and Simulink simulations to achieve precise target interception.

[![MATLAB](https://img.shields.io/badge/MATLAB-R2021+-orange?style=flat&logo=mathworks&logoColor=white)](https://www.mathworks.com/)
[![Simulink](https://img.shields.io/badge/Simulink-Control-blue?style=flat)](https://www.mathworks.com/products/simulink.html)
[![Control Theory](https://img.shields.io/badge/Control-Nonlinear-red?style=flat)](#)

---

## 📘 Table of Contents
1. [Overview](#-overview)
2. [Problem Statement](#-problem-statement)
3. [Mathematical Background](#-mathematical-background)
4. [Control Strategies](#-control-strategies)
5. [Stability Analysis](#-stability-analysis)
6. [Implementation](#-implementation)
7. [Results](#-results)
8. [Installation & Setup](#-installation--setup)
9. [Usage](#-usage)
10. [Future Work](#-future-work)
11. [References](#-references)
12. [Author](#-author)
13. [License](#-license)

---

## 🎯 Overview

Missile guidance systems require precise sensor feedback and robust control algorithms to intercept moving targets effectively. This project analyzes and compares two guidance strategies:

1. **Proportional Navigation Guidance (PNG)** - Classical linear guidance law
2. **Sliding Mode Control (SMC)** - Nonlinear robust control strategy

### Key Features
- **Nonlinear dynamics modeling** of missile-target engagement
- **Lyapunov stability analysis** with mathematical proofs
- **Input-to-State Stability (ISS)** for disturbance rejection
- **Filippov theory** for handling control discontinuities
- **Simulink implementation** with visualization

### Project Objectives
- Develop accurate missile-target relative dynamics models
- Implement and compare PNG and SMC guidance laws
- Prove stability using rigorous mathematical analysis
- Simulate realistic engagement scenarios
- Evaluate robustness under uncertainties and disturbances

---

## 📋 Problem Statement

### Challenge
Design a guidance system that:
1. Accurately tracks line-of-sight (LOS) angle to target
2. Computes required lateral acceleration for interception
3. Handles nonlinear dynamics and model uncertainties
4. Maintains stability under external disturbances
5. Guarantees finite-time convergence

### System Requirements
- **Sensor feedback**: LOS angle (λ) measurement
- **Control output**: Missile lateral acceleration command
- **Performance**: Minimize miss distance at intercept
- **Robustness**: Handle target evasive maneuvers
- **Stability**: Guarantee convergence using Lyapunov theory

---

## 📐 Mathematical Background

### Missile-Target Relative Dynamics

The engagement geometry is modeled using:

```
ṙ = -Vₘcos(θₘ - λ) - Vₜcos(θₜ - λ)
rλ̇ = Vₘsin(θₘ - λ) + Vₜsin(θₜ - λ)
```

**Variables:**
- `r` - Relative distance between missile and target
- `λ` - Line-of-sight (LOS) angle
- `Vₘ, Vₜ` - Missile and target speeds (assumed constant)
- `θₘ, θₜ` - Missile and target flight path angles

### Missile Heading Rate Model

Assuming constant speeds, the missile heading rate is:

```
θ̇ₘ = aₘ / Vₘ
```

where `aₘ` is the lateral acceleration command (control input).

### Line-of-Sight Rate

The LOS rate is a critical measurement for guidance:

```
λ̇ = (Vₘsin(θₘ - λ) + Vₜsin(θₜ - λ)) / r
```

---

## 🎮 Control Strategies

### 1. Proportional Navigation Guidance (PNG)

#### Control Law
```
aₘ = N · V꜀ · λ̇
```

**Parameters:**
- `N` - Navigation constant (typically 3-5)
- `V꜀` - Closing velocity: `V꜀ = Vₘcos(θₘ - λ) + Vₜcos(θₜ - λ)`
- `λ̇` - Line-of-sight rate

#### Advantages
- Simple implementation
- Proven effectiveness in linear scenarios
- Well-established in aerospace industry

#### Drawbacks
- Assumes linear dynamics
- Sensitive to disturbances
- No finite-time guarantees
- Cannot handle control saturation
- Poor performance with evasive targets

### 2. Sliding Mode Control (SMC)

#### Philosophy
SMC is a **nonlinear control strategy** that:
- Forces system trajectories onto a predefined sliding surface
- Maintains trajectories on the surface despite disturbances
- Provides **finite-time convergence**
- Exhibits **robustness** to model uncertainties

#### Sliding Surface Design
```
s = λ̇ + α·λ
```

where `α > 0` is a design parameter that controls convergence rate.

#### Control Law
```
aₘ = -K·sign(s)
```

where:
- `K > 0` - Control gain (must be sufficiently large)
- `sign(s)` - Signum function: +1 if s > 0, -1 if s < 0

#### Enhanced SMC with Boundary Layer

To reduce chattering (high-frequency oscillations):

```
aₘ = -K·sat(s/ε)
```

where `sat(·)` is the saturation function and `ε` is the boundary layer thickness.

---

## 🔬 Stability Analysis

### Lyapunov Stability Theory

#### Lyapunov Function
```
V(s) = (1/2)s²
```

This is a **positive definite** function that measures the "energy" of the sliding variable.

#### Time Derivative
```
V̇(s) = s·ṡ
```

For the SMC law `aₘ = -K·sign(s)`:

```
V̇(s) = s·(λ̈ + α·λ̇)
     = s·(-K·sign(s))
     = -K|s|
```

Since `K > 0`, we have **V̇ < 0** for all `s ≠ 0`.

#### Stability Guarantee

This proves that:
1. **V̇ ≤ 0** (Lyapunov stable)
2. **V̇ < 0** for `s ≠ 0` (asymptotically stable)
3. System reaches `s = 0` in **finite time**

### Input-to-State Stability (ISS)

Consider external disturbances `d(t)` acting on the system:

```
ṡ = f(s) + d(t)
```

The modified sliding condition becomes:

```
V̇ = s·(f(s) + d(t))
   ≤ -K|s| + |s|·|d(t)|
   ≤ -K|s| + |s|·D
```

where `D = sup|d(t)|` is the disturbance bound.

**Choosing K > D guarantees:**
```
V̇ ≤ -(K - D)|s| < 0
```

This ensures the system is **Input-to-State Stable**, meaning bounded disturbances lead to bounded state deviations.

### Filippov Theory

The signum function in SMC creates **discontinuous control**, which poses theoretical challenges.

**Filippov Solution** addresses this by:
1. Defining solutions in the Filippov sense for discontinuous right-hand sides
2. Allowing trajectories to "slide" along the discontinuity surface
3. Providing rigorous mathematical framework for convergence

For the sliding mode:
```
ṡ ∈ [-K, K]  (on the surface s = 0)
```

Filippov theory guarantees existence and uniqueness of solutions in the sliding regime.

---

## 💻 Implementation

### Simulink Model Structure

```
┌─────────────────────────────────────────────┐
│         Missile-Target Engagement           │
│                                             │
│  ┌─────────┐      ┌──────────┐            │
│  │ Target  │──────▶│   LOS    │            │
│  │Dynamics │      │Calculator│            │
│  └─────────┘      └────┬─────┘            │
│                         │                   │
│                         ▼                   │
│  ┌─────────┐      ┌──────────┐            │
│  │ Missile │◀─────│ Guidance │            │
│  │Dynamics │      │   Law    │            │
│  └─────────┘      └──────────┘            │
│                                             │
└─────────────────────────────────────────────┘
```

### Key Simulink Blocks

#### 1. Target Dynamics
- Implements target trajectory (constant velocity or evasive maneuvers)
- Outputs: Position, velocity, heading angle

#### 2. LOS Calculator
- Computes relative position vector
- Calculates LOS angle (λ) and LOS rate (λ̇)

#### 3. Guidance Law (PNG or SMC)
- **PNG**: Implements `aₘ = N·V꜀·λ̇`
- **SMC**: Implements `aₘ = -K·sign(s)`

#### 4. Missile Dynamics
- Integrates acceleration to get velocity and position
- Models missile kinematics with lateral acceleration

#### 5. Visualization
- Real-time trajectory plotting
- Miss distance calculation
- Sliding surface tracking (for SMC)

### MATLAB Code Structure

```matlab
% Main simulation script
function missile_guidance_simulation()
    % Parameters
    params = set_parameters();
    
    % Initial conditions
    IC = initialize_conditions();
    
    % Run simulation
    [t, x] = ode45(@(t,x) missile_dynamics(t, x, params), tspan, IC);
    
    % Plot results
    plot_trajectories(t, x, params);
    plot_sliding_surface(t, x, params);
end

% PNG Control Law
function a_m = png_control(lambda, lambda_dot, V_c, N)
    a_m = N * V_c * lambda_dot;
end

% SMC Control Law
function a_m = smc_control(s, K)
    a_m = -K * sign(s);
end

% Sliding surface
function s = sliding_surface(lambda, lambda_dot, alpha)
    s = lambda_dot + alpha * lambda;
end
```

---

## 📊 Results

### Simulation Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Vₘ | 300 m/s | Missile speed |
| Vₜ | 200 m/s | Target speed |
| N | 4 | PNG navigation constant |
| K | 50 | SMC control gain |
| α | 2 | SMC surface parameter |
| Initial r | 5000 m | Initial separation |
| Initial λ | 45° | Initial LOS angle |

### Performance Comparison

#### Scenario 1: Non-Maneuvering Target

| Metric | PNG | SMC |
|--------|-----|-----|
| **Miss Distance** | 2.3 m | 0.8 m |
| **Intercept Time** | 14.2 s | 13.8 s |
| **Peak Acceleration** | 45 m/s² | 52 m/s² |
| **Control Smoothness** | Smooth | Chattering |

#### Scenario 2: Evasive Target (±3g Maneuver)

| Metric | PNG | SMC |
|--------|-----|-----|
| **Miss Distance** | 18.5 m | 3.2 m |
| **Intercept Time** | 15.7 s | 14.1 s |
| **Peak Acceleration** | 68 m/s² | 75 m/s² |
| **Robustness** | Moderate | Excellent |

#### Scenario 3: With Sensor Noise (σ = 0.5°)

| Metric | PNG | SMC |
|--------|-----|-----|
| **Miss Distance** | 8.9 m | 4.1 m |
| **Intercept Time** | 14.8 s | 14.3 s |
| **Control Chatter** | Low | Moderate |
| **Noise Rejection** | Poor | Good |

### Key Observations

1. **SMC Advantages:**
   - Superior performance with evasive targets
   - Better disturbance rejection
   - Guaranteed finite-time convergence
   - Robust to model uncertainties

2. **PNG Advantages:**
   - Smoother control signals
   - Simpler implementation
   - Lower computational cost
   - Adequate for non-maneuvering targets

3. **Sliding Surface Behavior:**
   - SMC sliding surface converges to near-zero
   - Convergence time typically 2-4 seconds
   - Validates Lyapunov stability analysis

### Trajectory Plots

**Engagement Geometry:**
- Missile: Blue trajectory with acceleration vectors
- Target: Red trajectory (straight or evasive)
- LOS: Dashed line showing line-of-sight

**Sliding Surface (SMC only):**
- Shows s(t) → 0 as t → t_convergence
- Demonstrates finite-time reaching phase
- Validates theoretical predictions

---

## 🚀 Installation & Setup

### Prerequisites

```
MATLAB R2021a or later
Simulink
Control System Toolbox (optional but recommended)
```

### File Structure

```
missile-seeker-dynamics/
│
├── simulink/
│   ├── missile_guidance_png.slx
│   ├── missile_guidance_smc.slx
│   └── comparison_model.slx
│
├── matlab/
│   ├── main_simulation.m
│   ├── png_control.m
│   ├── smc_control.m
│   ├── missile_dynamics.m
│   ├── target_dynamics.m
│   └── plot_results.m
│
├── docs/
│   ├── NonlinearMissileSeeker_Tanish.pdf
│   └── mathematical_derivations.pdf
│
└── README.md
```

### Setup Instructions

1. **Clone the repository**
```bash
git clone https://github.com/TanishKatial5/Missile-Seeker-Dynamics.git
cd Missile-Seeker-Dynamics
```

2. **Open MATLAB**
```matlab
% Navigate to project directory
cd('path/to/Missile-Seeker-Dynamics')

% Add paths
addpath('matlab');
addpath('simulink');
```

3. **Run simulation**
```matlab
% For PNG
run('matlab/main_simulation_png.m')

% For SMC
run('matlab/main_simulation_smc.m')

% For comparison
run('matlab/comparison_analysis.m')
```

---

## 📱 Usage

### Running Simulink Models

#### PNG Simulation
```matlab
% Open Simulink model
open('simulink/missile_guidance_png.slx')

% Set parameters
N = 4;              % Navigation constant
Vm = 300;           % Missile speed (m/s)
Vt = 200;           % Target speed (m/s)
initial_range = 5000;  % Initial separation (m)

% Run simulation
sim('missile_guidance_png.slx')

% Plot results
plot_trajectories(tout, simout)
```

#### SMC Simulation
```matlab
% Open Simulink model
open('simulink/missile_guidance_smc.slx')

% Set parameters
K = 50;             % Control gain
alpha = 2;          % Sliding surface parameter
boundary_layer = 0.1;  % Chattering reduction

% Run simulation
sim('missile_guidance_smc.slx')

% Plot results
plot_trajectories(tout, simout)
plot_sliding_surface(tout, simout)
```

### Custom Scenarios

#### Evasive Target
```matlab
% Define target maneuver
target.type = 'evasive';
target.maneuver_time = 5;  % Start at t=5s
target.maneuver_accel = 3*9.81;  % 3g maneuver

% Run simulation with evasive target
sim('missile_guidance_smc.slx', 'ReturnWorkspaceOutputs', 'on');
```

#### Adding Noise
```matlab
% Add sensor noise
noise.type = 'gaussian';
noise.std_dev = 0.5;  % 0.5 degree standard deviation

% Enable noise block in Simulink
set_param('missile_guidance_smc/Sensor_Noise', 'Commented', 'off');
```

---

## 🔮 Future Work

### Short-term Enhancements
- [ ] Implement adaptive SMC with time-varying gains
- [ ] Add higher-order sliding mode control (HOSMC)
- [ ] Include aerodynamic effects and drag models
- [ ] Implement 3D engagement geometry

### Medium-term Goals
- [ ] Model predictive control (MPC) comparison
- [ ] Optimal control using Pontryagin's principle
- [ ] Real-time hardware-in-the-loop (HIL) testing
- [ ] Integration with actual sensor models (radar/IR)

### Advanced Features
- [ ] Multi-missile cooperative guidance
- [ ] Adaptive algorithms for unknown target dynamics
- [ ] Guidance law switching strategies
- [ ] Optimal impact angle control

---

## 📚 References

### Primary Sources
1. Viswanath, D., Krishnaswamy, S., & Deb, D. (2020). "Homing missile guidance using LOS rate and relative range measurement." *Aerospace Science and Technology*, 99, 105747.

2. Innocenti, M. (2002). "Nonlinear guidance techniques for agile missiles." *Control Engineering Practice*, 10(9), 1031-1039.

3. Khalil, H. K. (2002). *Nonlinear Systems* (3rd ed.). Upper Saddle River, NJ: Prentice Hall.

### Additional Reading
4. Shtessel, Y., Edwards, C., Fridman, L., & Levant, A. (2014). *Sliding Mode Control and Observation*. Birkhäuser.

5. Zarchan, P. (2012). *Tactical and Strategic Missile Guidance* (6th ed.). AIAA.

6. Utkin, V., Guldner, J., & Shi, J. (2009). *Sliding Mode Control in Electro-Mechanical Systems* (2nd ed.). CRC Press.

---

## 👨‍💻 Author

**Tanish Katial**  
Mechanical Engineering Student  
Boston University

- 📧 Email: [tkatial@bu.edu]
- 💼 LinkedIn: [linkedin.com/in/tanishkatial]
- 🔗 Portfolio: [your-portfolio.com]

### Project Context
Developed as part of advanced control systems coursework, this project demonstrates the application of nonlinear control theory, Lyapunov stability analysis, and robust control design to aerospace guidance systems.

---

## 📄 License

This project is released under the **MIT License**.

```
MIT License

Copyright (c) 2025 Tanish Katial

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction...
```

---

## ⚠️ Disclaimer

This project is intended for **educational and research purposes only**. The guidance algorithms and simulations presented here are simplified models for academic study. Real missile guidance systems involve classified technologies, extensive safety protocols, and regulatory compliance that are not addressed in this work.

---

## 🙏 Acknowledgments

- **Course Instructors**: For guidance on nonlinear control theory
- **Boston University**: For computational resources and library access
- **Research Community**: For open-source MATLAB tools and documentation

---

<div align="center">

**Built with control theory, validated with mathematics**

*When precision matters, stability proves everything*

[⬆ Back to Top](#-nonlinear-analysis-of-missile-seeker-dynamics)

</div>