from drone import Drone
from ball import Ball



def main():
    ball = Ball(
        start_x = 0, # m
        start_y = 10, # m
        start_vx = 0, # m/s
        start_vy = 0, # m/s
        ball_mass = 0.05, # kg
        ball_radius = 0.03, # m
        dt = 0.1 #s
    )

    drone = Drone(
        mass = 0.18,
        body_height = 2,
        body_width = 2,
        body_length = 2,
        arm_length = 0.086,
        initial_state = [0,0,0,0,0,0], # x, y, phi, vx, vy, vphi
    )

    return


if __name__ == "__main__":
    main()