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
rocket_simulation_system
-main_simulation.py # Entry point for running the full simulation
-rocket_dynamics.py # Aerodynamics and physics modeling
-sensor_simulator.py # GPS, IMU, and gyroscope sensor models
-state_estimator.py # Kalman Filter implementation for sensor fusion
-autopilot.py # Guidance and control algorithms
-tvc_controller.py # Thrust Vector Control system
-config.py # Global constants and configurable parameters
-analysis.py # Data analysis and visualization scripts

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

where:

| Symbol | Description | Units |
|:--|:--|:--:|
| $T_0$ | Sea-level temperature | K |
| $p_0$ | Sea-level pressure | Pa |
| $L$ | Temperature lapse rate | K/m |
| $g$ | Gravitational acceleration | m/s² |
| $R$ | Specific gas constant | J/(kg·K) |

---

### Derivation

Starting from the **ideal gas law** and **hydrostatic equilibrium**:

$$
1. **Ideal gas law:**  
\[
P = \rho R T
\]

2. **Hydrostatic equation:**  
\[
\frac{dP}{dh} = - \rho(h) g
\]

3. Substitute \(\rho = P / (R T(h))\):  
\[
\frac{dP}{dh} = - \frac{P}{R T(h)} g \quad \Rightarrow \quad \frac{dP}{P} = - \frac{g}{R T(h)} dh
\]

4. Substitute the linear temperature profile \(T(h) = T_0 - L h\):  
\[
\frac{dP}{P} = - \frac{g}{R (T_0 - L h)} dh
\]

5. Integrate both sides:  
\[
\int_{P_0}^{P(h)} \frac{dP'}{P'} = - \frac{g}{R} \int_0^h \frac{dh'}{T_0 - L h'}
\]

6. Solve the integral:  
\[
\ln \frac{P(h)}{P_0} = \frac{g}{R L} \ln \frac{T(h)}{T_0} 
\quad \Rightarrow \quad
P(h) = P_0 \left( \frac{T(h)}{T_0} \right)^{\frac{g}{R L}}
\]
$$
---




