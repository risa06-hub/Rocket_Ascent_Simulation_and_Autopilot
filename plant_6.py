
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ------------------ Constants ------------------
g0 = 9.81
Re = 6371000.0
mu = g0 * Re**2

# ------------------ Stage definitions ------------------
stages = [
    {"burn_time": 120, "thrust": 4.8e6, "mdot": 2200, "inert": 18000.0},
    {"burn_time": 180, "thrust": 1.2e6, "mdot": 350,  "inert": 7000.0},
    {"burn_time": 300, "thrust": 0.4e6, "mdot": 70,   "inert": 1500.0},
    {"burn_time": 400, "thrust": 0.12e6, "mdot": 20,  "inert": 800.0},
]

m0 = 320000.0
m_min = 40000.0
Cd = 0.3
A_ref = 6.158

# ------------------ GUIDANCE STATE (replaces class variables) ------------------
guidance_phase = "ASCENT"
target_altitude = 400000  # 400 km
coast_start_time = 0
circularization_burn_complete = False

# ------------------ SIMPLE GUIDANCE FUNCTIONS ------------------
def update_guidance(t, r, v, stage_idx):
    global guidance_phase, coast_start_time, circularization_burn_complete

    r_mag = np.linalg.norm(r)
    v_mag = np.linalg.norm(v)
    alt = r_mag - Re

    # Calculate orbital parameters
    orbit = orbit_from_state(r, v)
    r_hat = r / r_mag
    v_horizontal = np.linalg.norm(v - np.dot(v, r_hat) * r_hat)

    # Phase transitions
    if guidance_phase == "ASCENT":
        # Stay in ascent until we have significant horizontal velocity and altitude
        if alt > 100000 and v_horizontal > 5000:  # 100 km alt, 5 km/s horizontal
            guidance_phase = "COAST_TO_APOAPSIS"
            coast_start_time = t
            print(f"Switching to COAST phase at t={t:.1f}s")

    elif guidance_phase == "COAST_TO_APOAPSIS":
        # Coast until we're near apoapsis (vertical velocity near zero)
        v_vertical = np.dot(v, r_hat)

        # If vertical velocity is small and positive (approaching apoapsis)
        # or if we've been coasting for a long time
        if (abs(v_vertical) < 10) or (t - coast_start_time > 600):
            guidance_phase = "CIRCULARIZE"
            print(f"Switching to CIRCULARIZE phase at t={t:.1f}s")
            print(f"  Altitude: {alt/1000:.1f} km")
            print(f"  Horizontal velocity: {v_horizontal:.1f} m/s")
            print(f"  Vertical velocity: {v_vertical:.1f} m/s")

    elif guidance_phase == "CIRCULARIZE":
        # Once circularization is done, go to coast
        if orbit["bound"] and orbit["rp"] > Re + 200000 and orbit["e"] < 0.2:
            guidance_phase = "ORBIT"
            circularization_burn_complete = True
            print(f"ORBIT ACHIEVED at t={t:.1f}s!")

    elif guidance_phase == "ORBIT":
        # We're done
        pass

    return guidance_phase

def get_thrust_control(t, r, v, stage_idx, phase):
    r_mag = np.linalg.norm(r)
    v_mag = np.linalg.norm(v)

    if phase == "ASCENT":
        # During ascent, use gravity turn profile
        if t < 10:
            pitch = np.deg2rad(89.0)
        elif t < 200:
            frac = (t - 10) / 190
            pitch = np.deg2rad(89.0 * (1 - frac) + 10.0 * frac)
        else:
            pitch = np.deg2rad(10.0)

        # Convert pitch to thrust direction
        r_hat = r / r_mag
        east = np.cross([0, 1, 0], r_hat)
        east /= np.linalg.norm(east)
        thrust_dir = np.sin(pitch) * r_hat + np.cos(pitch) * east
        thrust_mag = stages[stage_idx]["thrust"] if stage_idx < len(stages) else 0.0

    elif phase == "COAST_TO_APOAPSIS":
        # No thrust during coast
        thrust_dir = v / v_mag if v_mag > 0 else np.array([1, 0, 0])
        thrust_mag = 0.0

    elif phase == "CIRCULARIZE":
        # Thrust prograde (in velocity direction) to circularize
        thrust_dir = v / v_mag
        thrust_mag = stages[stage_idx]["thrust"] if stage_idx < len(stages) else 0.0

        # But only if we have fuel and the stage is active
        if stage_idx >= len(stages):
            thrust_mag = 0.0

    elif phase == "ORBIT":
        # No thrust in orbit
        thrust_dir = v / v_mag
        thrust_mag = 0.0

    else:
        thrust_dir = v / v_mag
        thrust_mag = 0.0

    return thrust_mag, thrust_dir / np.linalg.norm(thrust_dir)

# ------------------ Orbital mechanics functions ------------------
def atmosphere(h):
    if h < 11000:
        T = 288.15 - 0.0065 * h
        p = 101325 * (T / 288.15)**(9.81 / (287.05 * 0.0065))
    else:
        T = 216.65
        p = 22632 * np.exp(-9.81 / (287.05 * 216.65) * (h - 11000))
    rho = p / (287.05 * T)
    return rho, T

def gravity_accel(r):
    r_mag = np.linalg.norm(r)
    return -mu * r / (r_mag**3)

def specific_energy(r, v):
    r_mag = np.linalg.norm(r)
    return 0.5 * np.dot(v, v) - mu / r_mag

def orbit_from_state(r, v):
    r_mag = np.linalg.norm(r)
    eps = specific_energy(r, v)
    h_vec = np.cross(r, v)
    h_mag = np.linalg.norm(h_vec)

    if eps >= 0:
        return {"eps": eps, "a": np.nan, "e": np.nan, "rp": np.nan, "ra": np.nan, "bound": False}

    a = -mu / (2.0 * eps)
    e_vec = (np.cross(v, h_vec) / mu) - (r / r_mag)
    e = np.linalg.norm(e_vec)
    rp = a * (1 - e)
    ra = a * (1 + e)

    return {"eps": eps, "a": a, "e": e, "rp": rp, "ra": ra, "bound": True}

def flight_path_angle(r, v):
    r_hat = r / np.linalg.norm(r)
    v_mag = np.linalg.norm(v)
    if v_mag < 1e-8:
        return 0.0
    return np.arcsin(np.clip(np.dot(v, r_hat) / v_mag, -1, 1))

# ------------------ Simulation ------------------
dt = 0.5
t = 0.0
stage_idx = 0
stage_timer = 0.0
m = m0
r = np.array([0.0, 0.0, Re])
v = np.array([0.0, 0.0, 0.0])

# Logging arrays
t_log = [t]
r_log = [r.copy()]
v_log = [v.copy()]
m_log = [m]
phase_log = ["ASCENT"]
v_horizontal_log = [0.0]
v_vertical_log = [0.0]

print("Starting SIMPLE GUIDED ascent to orbit...")
print("Target: 400 km circular orbit")

step = 0
max_steps = 10000
orbit_achieved = False

while step < max_steps and not orbit_achieved:
    # Current state
    r_mag = np.linalg.norm(r)
    alt = r_mag - Re

    # Get guidance commands
    current_phase = update_guidance(t, r, v, stage_idx)
    thrust_mag, thrust_dir = get_thrust_control(t, r, v, stage_idx, current_phase)

    # Environment
    rho, T = atmosphere(max(alt, 0))
    v_mag = np.linalg.norm(v)

    # Forces
    drag_vec = -0.5 * rho * v_mag * Cd * A_ref * v if v_mag > 0 else np.zeros(3)
    thrust_vec = thrust_mag * thrust_dir
    gravity_vec = gravity_accel(r) * m

    # Acceleration
    a_vec = (thrust_vec + drag_vec + gravity_vec) / m

    # Integration
    v = v + a_vec * dt
    r = r + v * dt
    t += dt

    # Mass depletion (only if thrusting and stage active)
    if thrust_mag > 0 and stage_idx < len(stages):
        m = max(m - stages[stage_idx]["mdot"] * dt, m_min)
        stage_timer += dt

        # Stage separation
        if stage_timer >= stages[stage_idx]["burn_time"]:
            print(f"Stage {stage_idx + 1} separation at t={t:.1f}s, alt={alt/1000:.1f}km, v={v_mag:.1f}m/s")
            m = max(m - stages[stage_idx]["inert"], m_min)
            stage_idx += 1
            stage_timer = 0.0

    # Calculate diagnostics
    r_hat = r / r_mag
    v_vertical = np.dot(v, r_hat)
    v_horizontal = np.linalg.norm(v - v_vertical * r_hat)
    orbit_info = orbit_from_state(r, v)

    # Logging
    t_log.append(t)
    r_log.append(r.copy())
    v_log.append(v.copy())
    m_log.append(m)
    phase_log.append(current_phase)
    v_horizontal_log.append(v_horizontal)
    v_vertical_log.append(v_vertical)

    step += 1

    # Check for orbit achievement
    if current_phase == "ORBIT":
        orbit_achieved = True
        print(f"\n✅ MISSION SUCCESS! Stable orbit achieved at t={t:.1f}s")
        break

    # Safety checks
    if r_mag < Re:
        print(f"❌ Impact with Earth at t={t:.1f}s")
        break

    if t > 3600:  # 1 hour timeout
        print("⏰ Simulation timeout")
        break

# Convert to arrays
t_log = np.array(t_log)
r_log = np.array(r_log)
v_log = np.array(v_log)
m_log = np.array(m_log)
v_horizontal_log = np.array(v_horizontal_log)
v_vertical_log = np.array(v_vertical_log)

# Final analysis
final_orbit = orbit_from_state(r_log[-1], v_log[-1])
final_alt = (np.linalg.norm(r_log[-1]) - Re) / 1000
final_v = np.linalg.norm(v_log[-1])

print("\n" + "="*50)
print("FINAL RESULTS")
print("="*50)
print(f"Final phase: {phase_log[-1]}")
print(f"Final altitude: {final_alt:.1f} km")
print(f"Final velocity: {final_v:.1f} m/s")
print(f"Final horizontal velocity: {v_horizontal_log[-1]:.1f} m/s")
print(f"Simulation time: {t:.1f} s")
print(f"Final mass: {m_log[-1]:.1f} kg")

if final_orbit["bound"]:
    rp_alt = (final_orbit['rp'] - Re) / 1000
    ra_alt = (final_orbit['ra'] - Re) / 1000
    print(f"\nOrbital Elements:")
    print(f"  Semi-major axis: {final_orbit['a']/1000:.1f} km")
    print(f"  Eccentricity: {final_orbit['e']:.4f}")
    print(f"  Periapsis: {rp_alt:.1f} km")
    print(f"  Apoapsis: {ra_alt:.1f} km")

    if rp_alt > 200 and final_orbit['e'] < 0.1:
        print("✅ STABLE CIRCULAR ORBIT ACHIEVED!")
    elif rp_alt > 0:
        print("⚠️  Elliptical orbit achieved")
    else:
        print("❌ Suborbital trajectory")
else:
    print("❌ Escape trajectory")

# ------------------ Plots ------------------
plt.figure(figsize=(15, 10))

# Altitude and phase
plt.subplot(2, 3, 1)
plt.plot(t_log, (np.linalg.norm(r_log, axis=1) - Re) / 1000, 'b-', linewidth=2)
plt.ylabel('Altitude [km]')
plt.xlabel('Time [s]')
plt.grid(True)
plt.title('Altitude vs Time')

# Velocity components
plt.subplot(2, 3, 2)
plt.plot(t_log, np.linalg.norm(v_log, axis=1), 'k-', label='Total', linewidth=2)
plt.plot(t_log, v_horizontal_log, 'g-', label='Horizontal')
plt.plot(t_log, np.abs(v_vertical_log), 'r-', label='Vertical')
plt.ylabel('Velocity [m/s]')
plt.xlabel('Time [s]')
plt.legend()
plt.grid(True)
plt.title('Velocity Components')

# Trajectory
plt.subplot(2, 3, 3)
downrange = np.sqrt(r_log[:,0]**2 + r_log[:,1]**2) / 1000
altitude = (np.linalg.norm(r_log, axis=1) - Re) / 1000
plt.plot(downrange, altitude, 'b-', linewidth=2)
plt.xlabel('Downrange Distance [km]')
plt.ylabel('Altitude [km]')
plt.grid(True)
plt.title('Trajectory')

# Mass
plt.subplot(2, 3, 4)
plt.plot(t_log, m_log, 'purple', linewidth=2)
plt.ylabel('Mass [kg]')
plt.xlabel('Time [s]')
plt.grid(True)
plt.title('Vehicle Mass')

# Orbital elements
if final_orbit["bound"]:
    a_vals = []
    e_vals = []
    for i in range(len(r_log)):
        orbit = orbit_from_state(r_log[i], v_log[i])
        if orbit["bound"]:
            a_vals.append(orbit['a']/1000)
            e_vals.append(orbit['e'])
        else:
            a_vals.append(np.nan)
            e_vals.append(np.nan)

    plt.subplot(2, 3, 5)
    plt.plot(t_log, a_vals, 'blue', linewidth=2)
    plt.ylabel('Semi-major Axis [km]')
    plt.xlabel('Time [s]')
    plt.grid(True)
    plt.title('Orbital Elements')

    plt.subplot(2, 3, 6)
    plt.plot(t_log, e_vals, 'red', linewidth=2)
    plt.axhline(y=0, color='green', linestyle='--', label='Circular')
    plt.ylabel('Eccentricity')
    plt.xlabel('Time [s]')
    plt.legend()
    plt.grid(True)

plt.tight_layout()
plt.show()

# Phase timeline
print(f"\nMission Phase Timeline:")
unique_phases = []
phase_times = []
current_phase = phase_log[0]
start_time = t_log[0]

for i, phase in enumerate(phase_log):
    if phase != current_phase:
        unique_phases.append(current_phase)
        phase_times.append((start_time, t_log[i-1]))
        current_phase = phase
        start_time = t_log[i]

unique_phases.append(current_phase)
phase_times.append((start_time, t_log[-1]))

for phase, (start, end) in zip(unique_phases, phase_times):
    print(f"  {phase:15s}: {start:6.1f}s to {end:6.1f}s (duration: {end-start:5.1f}s)")
