from math import sin, cos
from scipy.integrate import solve_ivp
import scipy.constants as sc
import matplotlib.pyplot as plt
# source https://cookierobotics.com/052/


class Drone():
    def __init__(
            self,
            mass,
            body_height,
            body_width,
            body_length,
            arm_length
            ) -> None:
        
        # Constants
        self.g = sc.g   # Gravitational acceleration (m/s^2)
        self.m = mass    # Mass (kg)
        self.h = body_height
        self.l = body_length
        self.w = body_width
        # Mass moments of inertia (kg*m^2)
        self.Ixx = 1/12 * self.m*(self.h**2 + self.l**2) 
        self.Iyy = 1/12 * self.m*(self.l**2 + self.w**2)
        self.Izz = 1/12 * self.m*(self.w**2 + self.h**2)

        self.L = arm_length
        return

    def trajectory(self, t):
        """
        Returns the desired position, velocity, and acceleration at a given time.
        Trajectory is a step (y changes from 0 to 0.5 at t=1)

        t     : Time (seconds), scalar
        return: Desired position & velocity & acceleration, y, z, vy, vz, ay, az
        """
        if t < 1:
            y = 0
        else:
            y = 3
        z = 3
        vy = 0
        vz = 0
        ay = 0
        az = 0

        return y, z, vy, vz, ay, az



    def controller(self, x, y_des, z_des, vy_des, vz_des, ay_des, az_des):
        """
        Returns force and moment to achieve desired state given current state.
        Calculates using PD controller.

        x     : Current state, [y, z, phi, vy, vz, phidot]
        y_des : desired y
        z_des : desired z
        vy_des: desired y velocity
        vz_des: desired z velocity
        ay_des: desired y acceleration
        az_des: desired z acceleration
        return: Force and moment to achieve desired state
        """
        Kp_y   = 0.4
        Kv_y   = 1.0
        Kp_z   = 0.4
        Kv_z   = 1.0
        Kp_phi = 18
        Kv_phi = 15
        
        phi_c = -1/self.g * (ay_des + Kv_y * (vy_des - x[3]) + Kp_y * (y_des - x[0]))
        F = self.m * (self.g + az_des + Kv_z * (vz_des - x[4]) + Kp_z * (z_des - x[1]))
        M = self.Ixx * (Kv_phi * (-x[5]) + Kp_phi * (phi_c - x[2]))

        return F, M



    def clamp(self, F, M):
        """
        Limit force and moment to prevent saturating the motor
        Clamp F and M such that u1 and u2 are between 0 and 1.7658

        u1      u2
        _____    _____
        |________|

        F = u1 + u2
        M = (u2 - u1)*L
        """
        u1 = 0.5*(F - M/self.L)
        u2 = 0.5*(F + M/self.L)
        
        if u1 < 0 or u1 > 1.7658 or u2 < 0 or u2 > 1.7658:
            print(f'motor saturation {u1} {u2}')
        
        u1_clamped = min(max(0, u1), 1.7658)
        u2_clamped = min(max(0, u2), 1.7658)
        F_clamped = u1_clamped + u2_clamped
        M_clamped = (u2_clamped - u1_clamped) * self.L

        return F_clamped, M_clamped



    def xdot(self, t, x):
        """
        Equation of motion
        dx/dt = f(t, x)

        t     : Current time (seconds), scalar
        x     : Current state, [y, z, phi, vy, vz, phidot]
        return: First derivative of state, [vy, vz, phidot, ay, az, phidotdot]
        """
        y_des, z_des, vy_des, vz_des, ay_des, az_des = self.trajectory(t)
        F, M = self.controller(x, y_des, z_des, vy_des, vz_des, ay_des, az_des)
        F_clamped, M_clamped = self.clamp(F, M)

        # First derivative, xdot = [vy, vz, phidot, ay, az, phidotdot]
        return [x[3],
                x[4],
                x[5],
                -F_clamped * sin(x[2]) / self.m,
                F_clamped * cos(x[2]) / self.m - self.g,
                M_clamped / self.Ixx]



def main():
    x0 = [0, 0, 0, 0, 0, 0] # Initial state [y0, z0, phi0, vy0, vz0, phidot0]
    t_span = [0, 20]            # Simulation time (seconds) [from, to]

    drone = Drone(
        mass=0.18,
        body_height=2,
        body_width=2,
        body_length=2,
        arm_length=0.086
    )

    # Solve for the states, x(t) = [y(t), z(t), phi(t), vy(t), vz(t), phidot(t)]
    sol = solve_ivp(drone.xdot, t_span, x0)


    # Plot
    fig, axs = plt.subplots()
    axs.plot(sol.y[0], sol.y[1]) # plot of drone's trajectory
    axs.set_xlabel("lateral position (m)")
    axs.set_ylabel("vertical position (m)")
    axs.set_xbound(0,5)
    axs.set_ybound(0,5)
    axs.set_title("Drone path from start point to end point")
    #axs[1].plot(sol.t, sol.y[1]) # z   vs t
    #axs[2].plot(sol.t, sol.y[2]) # phi vs t
    plt.show()

    return



if __name__ == "__main__":
    main()