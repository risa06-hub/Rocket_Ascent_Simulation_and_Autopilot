import numpy as np
from config import *

class RocketDynamics:
    def __init__(self):
        # Aerodynamic coefficients
        self.Cd_min = 0.3
        self.Cd_max = 0.8
        self.Cl_alpha = 2.0
        self.Cm_alpha = -1.5

    def atmosphere_model(self, altitude):
        altitude = max(0.0, altitude)
        if altitude < 11000:
            T = 288.14 - 0.0065 * altitude
            P = 101325 * (T / 288.15) ** (g0 / (0.0065 * R_air))
        elif altitude < 25000:
            T = 216.65
            P = 22632 * np.exp(-g0 * (altitude - 11000) / (R_air * T))
        else:
            T = 216.65 + 0.001 * (altitude - 25000)
            P = 2488 * (216.65 / T) ** (g0 / (0.001 * R_air))
        rho = P / (R_air * T)
        return rho, T, P


    def compute_aerodynamic_forces(self, state, wind=np.zeros(3)):
        # state = [x,y,z,u,v,w,phi,theta,psi,p,q,r,mass]
        r = state[0:3]
        v = state[3:6]
        euler = state[6:9]
        omega = state[9:12]
        mass = state[12]

        altitude = max(0.0, np.linalg.norm(r) - Re)
        rho, T, P = self.atmosphere_model(altitude)

        phi, theta, psi = euler
        R_bi = self.euler_to_matrix(phi, theta, psi)

        # Air-relative velocity (in body frame)
        v_air_inertial = v - wind
        v_air_body = R_bi.T @ v_air_inertial
        v_mag = np.linalg.norm(v_air_body)
        if v_mag < 1e-6:
            return np.zeros(3), np.zeros(3)

        # Angles
        alpha = np.arctan2(v_air_body[2], v_air_body[0])
        beta = np.arcsin(np.clip(v_air_body[1] / v_mag, -1, 1))

        # Dynamic pressure
        q_bar = 0.5 * rho * v_mag**2

        # Aerodynamic coefficients
        mach = v_mag / max(1.0, np.sqrt(1.4 * R_air * T))
        Cd = self.compute_drag_coefficient(mach, alpha)
        Cl = np.clip(self.Cl_alpha * alpha, -1.5, 1.5)
        Cy = -0.5 * beta

        # Aerodynamic forces (body frame)
        F_aero_body = np.array([
            -q_bar * area_ref * Cd,
            q_bar * area_ref * Cy,
            -q_bar * area_ref * Cl
        ])

        # Aerodynamic moments
        M_aero_body = self.compute_aerodynamic_moments(q_bar, alpha, beta, omega, v_mag)

        # Convert to inertial frame
        F_aero_inertial = R_bi @ F_aero_body
        return F_aero_inertial, M_aero_body

    def compute_drag_coefficient(self, mach, alpha):
        Cd = np.clip(
            self.Cd_min + (self.Cd_max - self.Cd_min) * np.clip((mach - 0.8) / 0.4, 0, 1),
            self.Cd_min, self.Cd_max
        )
        Cd += 0.1 * alpha**2  # AoA effect
        return Cd

    def compute_aerodynamic_moments(self, q_bar, alpha, beta, omega, v_mag):
        # Base coefficients
        Cm = np.clip(self.Cm_alpha * alpha, -2, 2)
        Cn = 0.1 * beta
        Cl_roll = -0.01 * beta * alpha

        # Damping terms
        p, q, r = omega
        l_ref = length
        denom = max(v_mag, 1.0)
        Cm_damp = -0.5 * (q * l_ref / (2 * denom))
        Cn_damp = -0.3 * (r * l_ref / (2 * denom))
        Cl_damp = -0.2 * (p * l_ref / (2 * denom))

        M_aero_body = q_bar * area_ref * length * np.array([
            Cl_roll + Cl_damp,
            Cm + Cm_damp,
            Cn + Cn_damp
        ])
        return M_aero_body

    def gravity_forces(self, r, mass):
        r_mag = np.linalg.norm(r)
        if r_mag < 1e-3:
            return np.zeros(3), np.zeros(3)
        F_gravity = -(mu * mass / r_mag**3) * r

        # Optional gravity gradient torque
        if r_mag > Re:
            I = np.diag([I_xx, I_yy, I_zz])
            M_gravity = (3 * mu / r_mag**5) * np.cross(r, I @ r)
        else:
            M_gravity = np.zeros(3)
        return F_gravity, M_gravity

    def thrust_forces(self, thrust_mag, thrust_dir_body, tvc_angles):
        tvx, tvy = tvc_angles
        R_tvc = self.tvc_rotation_matrix(tvx, tvy)
        thrust_deflected = R_tvc @ thrust_dir_body
        F_thrust = thrust_mag * thrust_deflected
        thrust_pos = np.array([-length / 2, 0.0, 0.0])
        M_thrust = np.cross(thrust_pos, F_thrust)
        return F_thrust, M_thrust

    def tvc_rotation_matrix(self, tvx, tvy):
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(tvx), -np.sin(tvx)],
            [0, np.sin(tvx), np.cos(tvx)]
        ])
        Ry = np.array([
            [np.cos(tvy), 0, np.sin(tvy)],
            [0, 1, 0],
            [-np.sin(tvy), 0, np.cos(tvy)]
        ])
        return Ry @ Rx

    def euler_to_matrix(self, phi, theta, psi):
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi), np.cos(phi)]
        ])
        R_y = np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
        R_z = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])
        return R_z @ R_y @ R_x

    def euler_kinematics(self, phi, theta, omega):
        p, q, r = omega
        # Avoid singularities at 90° pitch
        cos_theta = np.clip(np.cos(theta), 1e-6, 1)
        euler_rates = np.array([
            p + q * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta),
            q * np.cos(phi) - r * np.sin(phi),
            (q * np.sin(phi) + r * np.cos(phi)) / cos_theta
        ])
        return euler_rates

    def state_derivative(self, state, F_total, M_total, mdot):
        r = state[0:3]
        v = state[3:6]
        euler = state[6:9]
        omega = state[9:12]
        mass = max(1.0, state[12])  # prevent division by zero

        phi, theta, psi = euler
        drdt = v
        dvdt = F_total / mass
        euler_rates = self.euler_kinematics(phi, theta, omega)

        I = np.diag([I_xx, I_yy, I_zz])
        domegadt = np.linalg.inv(I) @ (M_total - np.cross(omega, I @ omega))
        dmdt = -mdot

        return np.concatenate([drdt, dvdt, euler_rates, domegadt, [dmdt]])





