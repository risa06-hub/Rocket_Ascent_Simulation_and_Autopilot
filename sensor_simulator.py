import numpy as np
from config import *

class SensorSimulator:
    def __init__(self):
        #sensor noise
        self.gps_pos_noise=10
        self.gps_vel_noise=0.1
        self.imu_accel_noise=0.01
        self.imu_gyro_noise=0.001
        self.baro_alt_noise=5
        self.previous_alt=0
        self.filter_alpha=0.1  #heavy filtering
        self.accel_bias = np.array([0.01, -0.005, 0.02])  # m/s2
        self.gyro_bias = np.array([0.0001, 0.0002, -0.0003])  # rad/s

    def simulate_gps(self,true_r,true_v,t):
        #gps measurements with noise,updated rate and correlated errors

        if abs(t-round(t)) > dt/2:
            return None

        #correlated errors
        common_error_scale=np.random.normal(0,1)        #random numbers from a normal distribution mena 0 and sd=1

        pos_noise=np.random.normal(0,self.gps_pos_noise,3) + common_error_scale*2
        vel_noise=np.random.normal(0,self.gps_vel_noise,3) + common_error_scale*0.2 #more accurate than psotion

        #gps has better horizontal thaant vertical accuratcy so we are adding more noise to the vetical componet
        pos_noise[2] *= 1.5
        vel_noise[2] *= 1.5

        gps_r=true_r + pos_noise
        gps_v=true_v + vel_noise

        return gps_r,gps_v

    def simulate_imu(self,true_accel_body,true_omega_body,true_euler):

        #add noise
        accel_noise= np.random.normal(0,self.imu_accel_noise,3)
        gyro_noise= np.random.normal(0,self.imu_gyro_noise,3)

        #IMU measures specific force(acc gravity) in body frame
        phi,theta,psi=true_euler
        g_body=np.array([-g0*np.sin(theta),
                         g0*np.cos(theta)*np.sin(phi),
                         g0*np.cos(theta)*np.cos((phi))])

        #speific force and gyro with bias and noise
        imu_accel=true_accel_body - g_body + self.accel_bias + accel_noise
        imu_gyro= true_omega_body + self.gyro_bias + gyro_noise

        return imu_accel, imu_gyro

    def simulate_barometer(self,true_altitude):
        #using low pass filter
        #raw noise
        alt_noise=np.random.normal(0,self.baro_alt_noise)

        #low pass filter
        raw_measurement= true_altitude + alt_noise
        filtered_alt=(self.filter_alpha*raw_measurement + (1- self.filter_alpha)*self.previous_alt)

        self.previous_alt=filtered_alt
        return filtered_alt

    def simulate_magnetometer(self,true_euler):
        # a very simplified magnetometer
        att_noise=np.random.normal(0,0.01,3)
        return true_euler + att_noise





