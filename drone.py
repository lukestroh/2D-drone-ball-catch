from drone_body import DroneBody
import ball

from scipy.integrate import solve_ivp
import scipy.constants as sc
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import control as ct

from typing import List, Tuple

# source https://cookierobotics.com/052/


class Drone(DroneBody):
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

        # Init drone body
        super().__init__(mass, body_height, body_length, body_width, arm_length)

        # Drone state
        self.state = initial_state
        self.x = initial_state[0]
        self.y = initial_state[1]
        self.phi = initial_state[2]
        self.vx = initial_state[3]
        self.vy = initial_state[4]
        self.vphi = initial_state[5]

        # Center of mass
        self.com = [self.x, self.y]

        # LQR Dynamics
        self.A, self.B = self.linearize_dynamics()
        self.Q = np.diag([100,1,1,10,1,1])
        self.R = np.diag([1,1,1])

        # Max motor thrust
        self.max_motor_thrust = 1.7658


        self.target_state = np.zeros(6)
        self.drag = 0.1
        self.dt = dt
        return

    
    def linearize_dynamics(self) -> Tuple[np.ndarray]:
        """ Create the linearized dynamics matrices """
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

    def compute_control(self) -> np.ndarray:
        K, _, _ = ct.lqr(self.A, self.B, self.Q, self.R)
        control = -K @ (self.state - self.target_state)
        return control
    
    def predict_ball_position(self, ball: ball.Ball)-> float:
        """
        Calculate the x location where the ball will be when it reaches the current drone height
        """
        # Find the time for the ball to fall from current position to the drone height
        ball_coefs = np.array([-self.g/2, ball.vy, (ball.y - self.y)])
        time_to_fall = max(np.roots(ball_coefs))

        # Find the predicted x location of the ball using that time
        x_prediction = ball.x + ball.vx * time_to_fall
        return x_prediction
    
    def update_target_state(
            self,
            tx: float,
            ty: float,
            tphi: float,
            tvx: float,
            tvy: float,
            tphidot: float
        ) -> None:
        self.target_state[0] = tx
        self.target_state[1] = ty
        self.target_state[2] = tphi
        self.target_state[3] = tvx
        self.target_state[4] = tvy
        self.target_state[5] = tphidot
        return

    def detect_impact(self, ball: ball.Ball) -> bool:
        """ Detect if the ball collides with the body of the drone """
        
        # Get absolute corner locations
        k = np.sqrt((self.w/2)**2 + (self.h/2)**2)
        theta = np.arctan2(self.h, self.w)
        body_right_corner_loc = (
            self.x + k * np.cos(self.phi - theta),
            self.y + k * np.sin(self.phi - theta)
        )
        body_left_corner_loc = (
            self.x - k * np.cos(self.phi - theta),
            self.y - k * np.sin(self.phi - theta)
        )

        # Create ax + by + c = 0 from corners
        # slope = (body_right_corner_loc[1] - body_left_corner_loc[1]) / (body_right_corner_loc[0] - body_left_corner_loc[0])
        # intercept = body_left_corner_loc[1] - slope * body_left_corner_loc[0]
        # a = 1
        # b = -1/slope
        # c = intercept/slope

        # https://math.stackexchange.com/questions/422602/convert-two-points-to-line-eq-ax-by-c-0
        a = body_right_corner_loc[1] - body_left_corner_loc[1]
        b = body_left_corner_loc[0] - body_right_corner_loc[0]
        c = (
            body_right_corner_loc[0] * body_left_corner_loc[1] -
            body_left_corner_loc[0] * body_right_corner_loc[1]
        )

        d = abs(a * ball.x + b * ball.y + c) / np.sqrt(a**2 + b**2)

        # distance between a point and a line:
            # d = abs(a*px + b*py + c)/np.sqrt(a**2 + b**2)
            # given that the line has the equation ax + by + c = 0
        # If the distance - ball radius <=0, then we have hit the drone
        if d - ball.radius <= 0:
            return True
        else:
            return False
    
    """ TODO:
        # Code
        Finish detect_impact
        update moment of inertia for original body
        create function to update MOI after impact
        create function that fixes ball to drone body (corresponding function in Ball?)
        write impulse_response function
            append this to current datatype? syncing time and state important

        # Data
        Figure out if data makes sense (Does drone flip upside down?)
        Generate control plots
        Generate Force/torque plots
        Generate ball plot?

    """
    
    
    def step(self) -> None:
        """ Perform a single movement iteration by updating the state """
        control = self.compute_control()
        state_dot = self.A @ self.state + self.B @ control
        self.state = self.state + state_dot * self.dt
        self.x = self.state[0]
        self.y = self.state[1]
        self.phi = self.state[2]
        self.vx = self.state[3]
        self.vy = self.state[4]
        self.vphi = self.state[5]
        return
    
    def update_moment_of_inertia(self) -> None:
        return
    
    def get_impulse_resp(
            self,
            t0: float,
            sim_time: float,
            i: int
        ) -> ct.TimeResponseData:
        
        time = np.linspace(t0, sim_time, int(sim_time/self.dt) - i)

        self.update_moment_of_inertia()
        self.A, self.B = self.linearize_dynamics() # not sure this is the right dynamics... needs input force on x,y?
        C = np.zeros((self.B.shape[1], self.B.shape[0]))
        D = 0
        sys = ct.StateSpace(self.A, self.B, C, D)
        data = ct.impulse_response(sys=sys, T=time)

        return data
    

        
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
        
    def simulate(self):
        num_steps = int(5 / self.dt) + 1
        time = np.linspace(0.0, 5, num_steps)

        states = np.zeros((num_steps, 4))
        states[0] = np.array([self.state[0],
                     self.state[1],
                     self.state[2],
                     self.state[3]])

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