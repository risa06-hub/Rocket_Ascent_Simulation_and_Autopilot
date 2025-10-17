# 🚀 Rocket Ascent Simulation and Autopilot

## Overview
**Rocket_Ascent_Simulation_and_Autopilot** is a personal project focused on the **numerical modeling** and **control simulation** of a launch vehicle during its ascent phase.  

The system models rocket dynamics, performs **sensor fusion** using a **Kalman Filter**, and implements an **autopilot system** with **Thrust Vector Control (TVC)** for stable flight guidance and control.  
It also provides data analysis and visualization tools for post-simulation evaluation.

---

## ✨ Key Features
- **🚀 Rocket Dynamics Simulation:**  
  Models complete aerodynamics, physics, and environmental interactions during ascent.

- **📡 Sensor Simulation:**  
  Includes realistic GPS, IMU, and gyroscope models for onboard measurement emulation.

- **🧠 State Estimation:**  
  Utilizes a **Kalman Filter** for accurate sensor fusion and real-time state estimation.

- **🕹️ Autopilot & Guidance Control:**  
  Implements closed-loop control algorithms for attitude stabilization and trajectory guidance.

- **🎯 Thrust Vector Control (TVC):**  
  Simulates actuator control for thrust direction adjustments and pitch/yaw control.

- **📊 Data Analysis & Visualization:**  
  Provides post-simulation analysis and visual representation of key flight parameters.

---

## 🧠 Project Structure
rocket_simulation_system/
│
├── main_simulation.py # Entry point for running the full simulation
├── rocket_dynamics.py # Aerodynamics and physics modeling
├── sensor_simulator.py # GPS, IMU, and gyroscope sensor models
├── state_estimator.py # Kalman Filter implementation for sensor fusion
├── autopilot.py # Guidance and control algorithms
├── tvc_controller.py # Thrust Vector Control system
├── config.py # Global constants and configurable parameters
└── analysis.py # Data analysis and visualization scripts

### config.py

- conatains the constants and parameters which are used throughout the project

### rocket_dynamics.py
-Atmosphere model
-- used ISA model, where:
1. for altitude,h<11000m, $$
p = p_0 \exp\left(-\frac{g (z - z_0)}{R T}\right)
$$



