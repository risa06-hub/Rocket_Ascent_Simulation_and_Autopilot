import numpy as np

Re = 6371000
mu = 3.986e14  
g0 = 9.81      
R_air = 287.05
rho0 = 1.225

# Rocket Parameters
rocket_mass = 320000.0
payload_mass = 40000.0
length = 70
diameter = 3.7
area_ref = np.pi * (diameter/2)**2
I_xx = 1e6
I_yy = 1e7
I_zz = 1e7


stages = [
    {"burn_time": 120,  "thrust_vac": 7.6e6, "mdot": 2500, "inert_mass": 18000, "tvc_max_angle": np.deg2rad(7.0)},   # Stage 1: 2 minutes
    {"burn_time": 180,  "thrust_vac": 2.4e6, "mdot": 500,  "inert_mass": 7000.0, "tvc_max_angle": np.deg2rad(10.0)}, # Stage 2: 3 minutes
    {"burn_time": 240,  "thrust_vac": 0.8e6, "mdot": 100,  "inert_mass": 1500,   "tvc_max_angle": np.deg2rad(12.0)}  # Stage 3: 4 minutes
]

# Total burn time: 120 + 180 + 240 = 540 seconds (9 minutes)
# Simulation Parameters
dt = 0.1
target_altitude = 400000
target_velocity = 7660





