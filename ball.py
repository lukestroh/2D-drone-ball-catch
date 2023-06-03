import numpy as np
import scipy.constants as sc

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

        self.g = sc.g
        self.dt = dt
        return

    def step(self, drone_state = None):
        if drone_state:
            ... # TODO

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
    
    def update_loc_from_drone(self):
        
        return
        