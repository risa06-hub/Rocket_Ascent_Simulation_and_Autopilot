import numpy as np

Re=6371000
mu=3.98             #Earth gravitational parameter [m3/s2]
go=9.81             #Gravity sea level [m/s2]
R_air=287.05        #specific gas constant [J/kg/K] given by Pressure = rho * R_air * Temp
rho0=1.225          #sea-level density[kg/m3]

#Rocket Parameter
rocket_mass=320000.0                    #inital mass [kg]
payload_mass=40000.0                    #final mass [kg]
length=70                               #rocket length [m]
diameter=3.7                            #rocket diameter [m]
area_ref=np.pi*(diameter/2)**2          #reference area [m2]
I_xx=1e6                                #moment of inertia x [kg.m2]
I_yy=1e7
I_zz=1e7

#staging configuration
stages=[{"burn_time":120,
         "thrust_vac":4.8e6,
         "mdot":2200,
         "inert_mass":18000,
         "tvc_max_angle":np.deg2rad(7.0)},
        {"burn_time":180,
         "thrust_vac":1.2e6,
         "mdot":350,
         "inert_mass":7000.0,
         "tvc_max_angle":np.deg2rad(10.0},
        {"burn_time":300,
         "thrust_vac":0.4e6,
         "mdot":70,
         "inert_mass":1500,
         "tvc_max_angle":np.deg2rad(12.0}]

#Simulation Parameter
dt=0.1                          #time step [s]
target_altitude=400000          #target orbit altitude [m]
target_velocity=7660            #circular orbital velocity [m/s]





