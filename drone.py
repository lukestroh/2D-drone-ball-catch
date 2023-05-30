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
            initial_state: List,
            dt: float
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

        # Dynamics
        self.A, self.B = self.linearize_dynamics()
        self.Q = np.diag([10,1,1,1,1,1])
        self.R = np.diag([1,1,1])


        self.target_state = np.array([3.0,2.0,0.0,0.0,0.0,0.0]) # placeholder; replace with location of ball at some point
        self.drag = 0.1
        self.dt = dt
        return

    
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

    def compute_control(self):
        K, _, _ = ct.lqr(self.A, self.B, self.Q, self.R)
        control = -K @ (self.state - self.target_state)
        return control

    
    def step(self):
        control = self.compute_control()
        state_dot = self.A @ self.state + self.B @ control
        self.state = self.state + state_dot * self.dt
        return
        
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

    dt = 0.001

    drone = Drone(
        mass=0.18,
        body_height=2,
        body_width=2,
        body_length=2,
        arm_length=0.086,
        initial_state=[0,2,0,0,0,0], # x, y, phi, vx, vy, vphi
        dt = dt
    )

    # Solve for the states, x(t) = [y(t), z(t), phi(t), vy(t), vz(t), phidot(t)]
    #sol = solve_ivp(drone.xdot, t_span, x0)
    time, states, controls = drone.simulate()
    # target_states = {
    #     0.0: np.array([3.0, 5.0, 0.0, 0.0,0.0,0.0]),
    #     2.0: np.array([3.0, 2.0, 0.0, 0.0,0.0,0.0])
    # }
    # drone.animate_trajectory(time, states, target_states, )

    # print(forces)
    plt.plot(time, controls[:,2])
    plt.show()

    return




if __name__ == "__main__":
    main()