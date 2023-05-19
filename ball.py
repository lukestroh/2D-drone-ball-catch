import numpy as np
import scipy.constants as sc


class Ball():
    def __init__(
            self,
            start_x,
            start_y,
            start_vx,
            start_vy,
            ball_mass,
            ball_radius,

            dt
        ):

        self.x = start_x
        self.y = start_y
        self.vx = start_vx
        self.vy = start_vy
        self.mass = ball_mass
        self.radius = ball_radius

        self.g = sc.g
        self.dt = dt
        return
    
    def get_position(self, t, drone_coordinates):
        """
        Step iteration of the ball falling in space
        """
        # start at some initial point with initial velocity values of 0
        # have ball fall bc gravity
        # initial x is same for drone and ball so ball just falls straight downwards
        
        t += self.dt
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.vy -= self.g * self.dt

        # This should maybe include the ball radius now?
        if (
            (drone_coordinates[1]-0.1) < self.y < drone_coordinates[1]
        ) and (
            (self.drone_coordinates[0]-0.25) <= self.x <= (self.drone_coordinates[0]+0.25)
        ):
            self.y = self.drone_coordinates[1]
            self.vy = 0
        elif (self.y <= 0):
            self.y = 0
            self.vy = 0
            self.vx = 0

        yield self.x, self.y