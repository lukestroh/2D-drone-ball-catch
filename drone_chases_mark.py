import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import control as ct

class PlanarQuadrotor:
    """This is just something I'm working on. I wrote it in a separate
    file just to keep it clean while I work on it. I can integrate this
    into drone.py soon."""
    def __init__(self, mass, inertia, drag):
        self.mass = mass
        self.inertia = inertia
        self.drag = drag

    def linearize_dynamics(self):
        """Defines the matrices. I even included some arbitrary drag :)"""
        A = np.zeros((4, 4))
        B = np.zeros((4, 2))

        A[0, 2] = 1.0
        A[1, 3] = 1.0
        A[2, 2] = -self.drag / self.mass
        A[3, 3] = -self.drag / self.inertia

        B[2, 0] = 1.0 / self.mass
        B[3, 1] = 1.0 / self.inertia

        return A, B

    def compute_control(self, state, target_state, Q, R):
        """Function calls A and B matrices and performs LQR"""
        A, B = self.linearize_dynamics()
        K, _, _ = ct.lqr(A, B, Q, R)
        control = -K.dot(state - target_state)
        return control

    def simulate(self, initial_state, target_states, Q, R, dt, duration):
        num_steps = int(duration / dt) + 1
        time = np.linspace(0.0, duration, num_steps)

        states = np.zeros((num_steps, 4))
        states[0] = initial_state

        for i in range(1, num_steps):
            state = states[i-1]
            target_state = self.interpolate_target_state(time[i], target_states)
            control = self.compute_control(state, target_state, Q, R)
            A, B = self.linearize_dynamics()
            state_dot = A.dot(state) + B.dot(control)
            states[i] = state + state_dot * dt

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

    def animate_trajectory(self, time, states, target_states, initial_state):
        fig, ax = plt.subplots()
        ax.set_xlim(0, 5)
        ax.set_ylim(0, 5)
        ax.set_aspect('equal')
        ax.set_xlabel("Horizontal position")
        ax.set_ylabel("Vertical position")
        ax.grid()

        quadrotor_body, = ax.plot([], [], 'k_', markersize=20)
        quadrotor_propeller_post_right, = ax.plot([], [], 'k|', markersize=8)
        quadrotor_propeller_post_left, = ax.plot([], [], 'k|', markersize=8)
        quadrotor_propeller_blade_right, = ax.plot([], [], 'k_', markersize=8)
        quadrotor_propeller_blade_left, = ax.plot([], [], 'k_', markersize=8)
        target_position, = ax.plot([], [], 'ro', markersize=5)

        def update(frame):
            x = states[frame, 0]
            y = initial_state[1]
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
        ani.save("ball_catch.gif", dpi=300, writer=PillowWriter(fps=25))
        plt.show()
        
def main():
    quadrotor = PlanarQuadrotor(mass=1.0, inertia=1.0, drag=0.1)

    initial_state = np.array([0.5, 2.0, 0.0, 0.0])  # [x, y, vx, vy]

    # Define the target states at different time instances
    target_states = {
        0.0: np.array([3.0, 5.0, 0.0, 0.0]),
        2.0: np.array([3.0, 2.0, 0.0, 0.0])
    }
    # this is generally just to show that I got the drone to be able to pursue a target
    # next I want to adjust this so the position of the target is based off the location of a ball falling
    Q = [[30,0,0,0], [0,10,0,0], [0,0,10,0],[0,0,0,10]]  # state cost
    R = np.eye(2)  # control cost

    dt = 0.1  # time step
    duration = 5.0  # duration

    time, states = quadrotor.simulate(initial_state, target_states, Q, R, dt, duration)
    quadrotor.animate_trajectory(time, states, target_states, initial_state)


if __name__ == "__main__":
    main()
    # i am very proud of this fyi
    # id recommend watching the gif rather than watching the plt window that pops up