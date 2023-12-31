import numpy as np
import scipy.constants as sc

import warnings
warnings.filterwarnings("error")

class Ball():
    def __init__(
            self,
            start_x: float,
            start_y: float,
            start_vx: float,
            start_vy: float,
            ball_mass: float,
            ball_radius: float,
            dt: float
        ):

        self.x = start_x
        self.y = start_y
        self.vx = start_vx
        self.vy = start_vy
        self.state = [self.x, self.y, self.vx, self.vy]

        self.mass = ball_mass
        self.radius = ball_radius
        self.Izz = 2 / 5 * self.mass * self.radius**2

        self.g = sc.g
        self.dt = dt

        return

    def step(self, collided: bool = False, drone = None):
        if collided:
            d = np.sqrt((self.x - drone.x)**2 + (self.y - drone.y)**2)
            if d < drone.h/2 + self.radius:
                d = drone.h/2 + self.radius
            try:
                theta = np.arcsin((drone.h/2 + self.radius) / d)
            except RuntimeWarning:
                print("\n",self.x, self.y, drone.x, drone.y, d, drone.h/2+self.radius)
            # print(d, theta, self.radius, drone.h/2+self.radius)
            # input()

            # Place the ball on the correct side of the drone
            if self.x >= drone.x:
                self.x = drone.x + d * np.cos(drone.phi + theta)
                self.y = drone.y + d * np.sin(drone.phi + theta)
            else:
                self.x = drone.x - d * np.cos(drone.phi + theta)
                self.y = drone.y + d * np.sin(drone.phi + np.pi - theta)
            
            # update vx, vy?? does this work if ball is on left side of drone COM?
            # would vx, vy = angular momentum of drone?

            # TODO: check if these are right:
            # https://math.stackexchange.com/questions/2444965/relationship-between-cartesian-velocity-and-polar-velocity
            self.vx = -d * drone.vphi * np.sin(theta)
            self.vy = d * drone.vphi * np.cos(theta)          


        else:
            self.vy += -self.g * self.dt
            self.x += self.vx * self.dt
            self.y += self.vy * self.dt
            
            if (self.y - self.radius <= 0):
                self.vx = 0
                self.y = 0
                self.vy = 0
        
        self.state = [self.x, self.y, self.vx, self.vy]
        return
    

