# Rocket Ascent Simulation and Autopilot

## Overview
**Rocket_Ascent_Simulation_and_Autopilot** is a personal project focused on the **numerical modeling** and **control simulation** of a launch vehicle during its ascent phase.  

The system models rocket dynamics, performs **sensor fusion** using a **Kalman Filter**, and implements an **autopilot system** with **Thrust Vector Control (TVC)** for stable flight guidance and control.  
It also provides data analysis and visualization tools for post-simulation evaluation.

---

## Key Features
- **Rocket Dynamics Simulation:**  
  Models complete aerodynamics, physics, and environmental interactions during ascent.

- **Sensor Simulation:**  
  Includes realistic GPS, IMU, and gyroscope models for onboard measurement emulation.

- **Controllers**  
  Use PD, LQG and hybrid controllers for the control system of rocket 

- **Thrust Vector Control (TVC):**  
  Simulates actuator control for thrust direction adjustments and pitch/yaw control.

- **Data Analysis & Visualization:**  
  Provides post-simulation analysis and visual representation of key flight parameters.

---

## Project Structure
rocket_simulation_system
-main_simulation.py # Entry point for running the full simulation and Data analysis and visualization scripts
-rocket_dynamics.py # Aerodynamics and physics modeling
-sensor_simulator.py # GPS, IMU, and gyroscope sensor models
-Controller: lqg_controller.py,pd_controller.py,hybrid_controller.py,base_controller.py #control system
-config.py # Global constants and configurable parameters

<h3 style="text-decoration: underline;">Config.py</h3>

- Contains the constants and parameters which are used throughout the project

<h3 style="text-decoration: underline;">rocket_dynamics.py</h3>

- **Atmosphere Model**

### Troposphere (h < 11,000 m)

The temperature decreases linearly with altitude:

$$
T(h) = T_0 - L\,h
$$

The pressure is calculated as:

$$
p(h) = p_0 \left( \frac{T(h)}{T_0} \right)^{\frac{g}{R L}}
$$





