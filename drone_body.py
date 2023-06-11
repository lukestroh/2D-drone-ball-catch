import scipy.constants as sc

from typing import List

class DroneBody():
    def __init__(
            self,
            mass: float,
            body_height: float,
            body_length: float,
            body_width: float,
            arm_length: float
        ) -> None:
        
        # Constants
        self.g = sc.g   # Gravitational acceleration (m/s^2)
        self.m = mass    # Mass (kg)
        self.h = body_height
        self.l = body_length
        self.w = body_width
        self.L = arm_length
        self.post_height: float = 0.1
        self.motor_mass: float = 0.02 # kg
        self.motor_r = 0.015 # m
        self.motor_h = 0.02
        
        # Propeller locations
        self.right_motor_loc: List[float] = [self.w/2 + self.L, 0]
        self.left_motor_loc: List[float] = [-self.w/2 - self.L, 0]

        # Mass moments of inertia (kg*m^2)
        self.drone_body_Izz = (1/12) * self.m * (self.w**2 + self.h**2)
        self.motor_Izz = (1/12) * self.motor_mass * (3 * self.motor_r**2 + self.motor_h**2)
        # Parallel axis theorem
        self.Izz = self.drone_body_Izz + 2 * (self.motor_Izz + self.motor_mass * (self.w + self.L)**2)
    
        


        return