from scipy.integrate import solve_ivp
import scipy.constants as sc
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import control as ct

from typing import List

# source https://cookierobotics.com/052/


class Drone():
    def __init__(
            self,
            mass: float,
            body_height: float,
            body_width: float,
            body_length: float,
            arm_length: float,
            initial_state: List
        ) -> None:

        
        # Constants
        self.g = sc.g   # Gravitational acceleration (m/s^2)
        self.m = mass    # Mass (kg)
        self.h = body_height
        self.l = body_length
        self.w = body_width
        self.L = arm_length
        self.max_motor_thrust = 1.7658

        # Mass moments of inertia (kg*m^2)
        self.Ixx = 1/12 * self.m*(self.h**2 + self.l**2) 
        self.Iyy = 1/12 * self.m*(self.l**2 + self.w**2)
        self.Izz = 1/12 * self.m*(self.w**2 + self.h**2)

        self.state = initial_state
        self.x = initial_state[0]
        self.y = initial_state[1]
        self.phi = initial_state[2]
        self.vx = initial_state[3]
        self.vy = initial_state[4]
        self.vphi = initial_state[5]

        self.Q = np.diag([10,1,1,1,1,1])
        self.R = np.diag([1,1,1])
        self.target_state = np.array([3.0,2.0,0.0,0.0,0.0,0.0]) # placeholder; replace with location of ball at some point
        self.drag = 0.1
        return

    def trajectory(self, t):
        """
        Returns the desired position, velocity, and acceleration at a given time.
        Trajectory is a step (y changes from 0 to yBall at t=1)
        t     : Time (seconds), scalar
        return: Desired position & velocity & acceleration, y, z, vy, vz, ay, az
        """
        if t < 1:
            x = 0
            # this loop basically starts the drone from a hover
        else:
            x = self.ball_x
            # set the goal point of the drone to be the same as the ball
            y = self.ball_y # arbitrary initial height of the drone

        vx = 0
        vy = 0
        ax = 0
        ay = 0
        return x,y,vx,vy,ax,ay
    
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
        # this section needs to be redone. feedforward is a no-no
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

        if u1 < 0 or u1 > self.max_motor_thrust or u2 < 0 or u2 > self.max_motor_thrust:
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
        x_des, y_des, vx_des, vy_des, ax_des, ay_des = self.trajectory(t)
        F, M = self.controller(x, x_des, y_des, vx_des, vy_des, ax_des, ay_des)

        F_clamped, M_clamped = self.clamp(F, M)

        # First derivative, xdot = [vy, vz, phidot, ay, az, phidotdot]
        return [x[3],
                x[4],
                x[5],
                -F_clamped * np.sin(x[2]) / self.m,
                F_clamped * np.cos(x[2]) / self.m - self.g,
                M_clamped / self.Ixx]
    
    def linearize_dynamics(self):
        # Linearized dynamics matrices
        A = np.array([[0,0,0,1.0,0,0],
                      [0,0,0,0,1.0,0],
                      [0,0,0,0,0,1.0],
                      [0,0,-self.g,0,0,0],
                      [0,0,0,0,0,0],
                      [0,0,0,0,0,0],])
        B = np.array([[0,0,0],
                      [0,0,0],
                      [0,0,0],
                      [0,0,0],
                      [1.0/self.m,0,-1.0],
                      [0,1.0/self.Ixx,0]])

        return A, B

    def compute_control(self, state, target_state, Q, R):
        A, B = self.linearize_dynamics()
        K, _, _ = ct.lqr(A, B, Q, R)
        control = -K.dot(state - target_state)
        return control

    def simulate(self):
        num_steps = int(5 / 0.1) + 1
        time = np.linspace(0.0, 5, num_steps)

        states = np.zeros((num_steps, 6))
        states[0] = np.array([self.state[0],
                     self.state[1],
                     self.state[2],
                     self.state[3],
                     self.state[4],
                     self.state[5]])

        for i in range(1, num_steps):
            state = states[i-1]
            control = self.compute_control(state, self.target_state, self.Q, self.R)
            A, B = self.linearize_dynamics()
            state_dot = A.dot(state) + B.dot(control)
            states[i] = state + state_dot * 0.1
        return time, states
    
    def interpolate_target_state(self, t, target_states):
        """This relates to the hardcoded positions at the bottom.
        I plan to replace this with a function that generates positions based
        off a ball falling. Currently, the program interpolates between the given
        points to plot where the target is."""
        t_max = max(target_states.keys())
        if t <= 0:
            return target_states[0]
        elif t >= t_max:
            return target_states[t_max]
        else:
            t_prev = max([t_prev for t_prev in target_states.keys() if t_prev < t])
            t_next = min([t_next for t_next in target_states.keys() if t_next > t])
            alpha = (t - t_prev) / (t_next - t_prev)
            target_prev = target_states[t_prev]
            target_next = target_states[t_next]
            target_interpolated = target_prev + alpha * (target_next - target_prev)
            return target_interpolated

    def animate_trajectory(self, time, states, target_states):
        fig, ax = plt.subplots()
        ax.set_xlim(0, 5)
        ax.set_ylim(0, 5)
        ax.set_aspect('equal')
        ax.set_xlabel("Horizontal position")
        ax.set_ylabel("Vertical position")
        ax.grid()
        def update(frame):
            x = states[frame, 0]
            y = self.y
            theta = states[frame, 2]
            quadrotor_body, = ax.plot([], [], 'k',marker=(2, 0, 90+theta*(180/np.pi)), markersize=20)
            quadrotor_propeller_post_right, = ax.plot([], [], 'k', marker=(2, 0, theta*(180/np.pi)), markersize=8)
            quadrotor_propeller_post_left, = ax.plot([], [], 'k', marker=(2, 0, theta*(180/np.pi)),markersize=8)
            quadrotor_propeller_blade_right, = ax.plot([], [], 'k', marker=(2, 0, 90+theta*(180/np.pi)), markersize=8)
            quadrotor_propeller_blade_left, = ax.plot([], [], 'k', marker=(2, 0, 90+theta*(180/np.pi)), markersize=8)
            target_position, = ax.plot([], [], 'ro', markersize=5)
            quadrotor_body.set_data(x, y)
            quadrotor_propeller_post_right.set_data(x+0.18, y+0.05)
            quadrotor_propeller_post_left.set_data(x-0.18, y+0.05)
            quadrotor_propeller_blade_right.set_data(x+0.18, y+0.1)
            quadrotor_propeller_blade_left.set_data(x-0.18, y+0.1)

            current_time = time[frame]
            current_target_state = self.interpolate_target_state(current_time, target_states)
            target_x = current_target_state[0]
            target_y = current_target_state[1]
            target_position.set_data(target_x, target_y)
            
            return quadrotor_body, target_position, quadrotor_propeller_post_right, quadrotor_propeller_post_left, quadrotor_propeller_blade_left,quadrotor_propeller_blade_right
        
        ani = FuncAnimation(fig, update, frames=len(time), blit=True)
        #ani.save("ball_catch.gif", dpi=300, writer=PillowWriter(fps=25))
        plt.show()

    def get_ball_data(self, ball_x: float, ball_y: float, ball_vx: float, ball_vy) -> None:
        self.ball_x = ball_x
        self.ball_y = ball_y
        self.ball_vx = ball_vx
        self.ball_vy = ball_vy
        return
    
def main():
    x0 = [0, 0, 0, 0, 0, 0] # Initial state [y0, z0, phi0, vy0, vz0, phidot0]
    t_span = [0, 20]            # Simulation time (seconds) [from, to]

    drone = Drone(
        mass=0.18,
        body_height=2,
        body_width=2,
        body_length=2,
        arm_length=0.086,
        initial_state=[0,2,0,0,0,0], # x, y, phi, vx, vy, vphi
    )

    # Solve for the states, x(t) = [y(t), z(t), phi(t), vy(t), vz(t), phidot(t)]
    #sol = solve_ivp(drone.xdot, t_span, x0)
    time, states = drone.simulate()
    target_states = {
        0.0: np.array([3.0, 5.0, 0.0, 0.0,0.0,0.0]),
        2.0: np.array([3.0, 2.0, 0.0, 0.0,0.0,0.0])
    }
    drone.animate_trajectory(time, states, target_states, )

    return




if __name__ == "__main__":
    main()