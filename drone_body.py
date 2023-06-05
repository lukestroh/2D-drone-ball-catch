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
        
        # Propeller locations
        self.right_prop_loc: List[float] = [
            self.l/2 + self.L, 
            self.h + self.post_height
        ]
        self.left_prop_loc: List[float] = [
            -self.l/2 - self.L, 
            self.h + self.post_height
        ]

        # Mass moments of inertia (kg*m^2)
        self.Ixx = (1/12) * (2 * self.m * (self.w**2 + self.h**2) + 2 * self.motor_mass * self.L**2) + 2 * self.motor_mass* self.L**2
        self.Iyy = (1/12) * (2 * self.m * (self.L**2 + self.h**2) + 2 * self.motor_mass * self.w**2) + 2 * self.motor_mass* self.w**2
        self.Izz = (1/12) * (2 * self.m * (self.L**2 + self.w**2) + 4 * self.motor_mass* self.h**2)

        return