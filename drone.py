from drone_body import DroneBody
import ball

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import control as ct

from typing import List, Tuple

# source https://cookierobotics.com/052/

class SimError(Exception):
    pass

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
        super().__init__(
            mass=mass,
            body_height=body_height,
            body_length=body_length,
            body_width=body_width,
            arm_length=arm_length
        )

        # Drone state
        self.state = initial_state
        self.x = initial_state[0]
        self.y = initial_state[1]
        self.phi = initial_state[2]
        self.vx = initial_state[3]
        self.vy = initial_state[4]
        self.vphi = initial_state[5]

        # Center of mass
        # self.com = [self.x, self.y]

        # LQR Dynamics
        self.A, self.B = self.linearize_dynamics()
        self.Q = np.diag([10,10,5,1,1,1])
        self.R = np.diag([1,1])
        self.K = self.compute_LQR_gain()

        # Max motor thrust
        self.max_motor_thrust = 1.7658


        self.target_state = self.state
        self.dt = dt
        return

    
    def linearize_dynamics(self, ball: ball.Ball = None) -> Tuple[np.ndarray]:
        """ 
        Create the linearized dynamics matrices
        u = [u1, u2, g]
            u1 = propeller direction force
            u2 = torque on drone
            g = force due to gravity
        """
        A = np.array([[0,0,0,1,0,0],
                        [0,0,0,0,1,0],
                        [0,0,0,0,0,1],
                        [0,0,-self.g,0,0,0],
                        [0,0,0,0,0,0],
                        [0,0,0,0,0,0]])
        

        if not ball:
            B = np.array([
                        [0, 0],
                        [0, 0],
                        [0, 0],
                        [0, 0],
                        [1/self.m, 0],
                        [0, 1/self.Izz]])
        else:
            B = np.array([
                        [0,0],
                        [0,0],
                        [0,0],
                        [0,0],
                        [1/(self.m+ball.mass),0],
                        [0,1/self.Izz]])
                
        return A, B
    
    def compute_LQR_gain(self) -> None:
        K, _, _ = ct.lqr(self.A, self.B, self.Q, self.R)
        return K

    def compute_control(self) -> np.ndarray:
        control = -self.K @ (np.subtract(self.state, self.target_state))
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
            self.x + k * np.cos(self.phi + theta),
            self.y + k * np.sin(self.phi + theta)
        )
        body_left_corner_loc = (
            self.x - k * np.cos(self.phi + theta),
            self.y + k * np.sin(self.phi + np.pi-theta)
        )

        # https://math.stackexchange.com/questions/422602/convert-two-points-to-line-eq-ax-by-c-0
        a = body_right_corner_loc[1] - body_left_corner_loc[1]
        b = body_left_corner_loc[0] - body_right_corner_loc[0]
        c = (
            body_right_corner_loc[0] * body_left_corner_loc[1] -
            body_left_corner_loc[0] * body_right_corner_loc[1]
        )

        d = abs(a * ball.x + b * ball.y + c) / np.sqrt(a**2 + b**2)

        # distance between a point p and a line:
            # d = abs(a*px + b*py + c)/np.sqrt(a**2 + b**2)
            # given that the line has the equation ax + by + c = 0
        # If the distance - ball radius <=0, then we have hit the drone

        # TODO: add to conditions that the ball.x must also be within x limits of drone
        if d - ball.radius <= 0:
            if (ball.x - ball.radius > body_right_corner_loc[0]) or (ball.x + ball.radius < body_left_corner_loc[0]):
                return False
            else:
                return True
        else:
            return False
  
    
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
    
    def _update_motor_locs(self) -> None:
        self.right_motor_loc = [
            self.x + (self.w + self.L) * np.cos(self.phi),
            self.y + (self.w + self.L) * np.sin(self.phi)
        ]
        self.left_motor_loc = [
            self.x - (self.w + self.L) * np.cos(self.phi),
            self.y + (self.w + self.L) * np.sin(self.phi)
        ]
        return

    def get_center_of_mass(self, ball: ball.Ball) -> Tuple[float]:
        self._update_motor_locs()
        center_of_mass = (
            ((self.x * self.m) + (self.right_motor_loc[0] * self.motor_mass) + (ball.x * ball.mass)) / (self.m + self.motor_mass + ball.mass),
            ((self.y * self.m) + (self.right_motor_loc[1] * self.motor_mass) + (ball.y * ball.mass)) / (self.m + self.motor_mass + ball.mass)
        )
        return center_of_mass
    
    def update_moment_of_inertia(self, ball: ball.Ball) -> None:
        """ Change the moment of inertia due to the addition of the ball and location, """
        # TODO: update MOI based on new COM
        # Double check if this is correct
        # http://astro1.panet.utoledo.edu/~mheben/PHYS_2130/Chapter11-1_mh.pdf
        center_of_mass = self.get_center_of_mass(ball=ball)
        d_self = np.sqrt((self.x - center_of_mass[0])**2 + (self.y - center_of_mass[1])**2)
        d_ball = np.sqrt((ball.x - center_of_mass[0])**2 + (ball.y - center_of_mass[1])**2)
        d_motor_r = np.sqrt((self.right_motor_loc[0] - center_of_mass[0])**2 + (self.right_motor_loc[1] - center_of_mass[1])**2)
        d_motor_l = np.sqrt((self.left_motor_loc[0] - center_of_mass[0])**2 + (self.left_motor_loc[1] - center_of_mass[1])**2)

        self.Izz = (
            (self.drone_body_Izz + self.m * d_self**2) + 
            (self.motor_Izz + self.motor_mass * d_motor_r**2) + 
            (self.motor_Izz + self.motor_mass * d_motor_l**2) +  
            (ball.Izz + ball.mass * d_ball**2)
        )
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
        C = np.identity(6)
        D = 0
        sys = ct.StateSpace(self.A, self.B, C, D)
        data = ct.impulse_response(sys=sys, T=time, X0=self.state)

        return data

        
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
            # TODO replace animation code from interpolate_target_state with new code
            #current_target_state = self.interpolate_target_state(current_time, target_states)
            #target_x = current_target_state[0]
            #target_y = current_target_state[1]
            #target_position.set_data(target_x, target_y)
            
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
        body_height=2.0,
        body_width=2.0,
        body_length=2.0,
        arm_length=0.086,
        initial_state=[0,2.0,0,0,0,0], # x, y, phi, vx, vy, vphi
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