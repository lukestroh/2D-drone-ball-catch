from drone import Drone
from ball import Ball

import matplotlib.pyplot as plt


def main():
    dt = 0.001
    time = 0
    timesteps = []
    drone_states = []
    ball_states = []

    ball = Ball(
        start_x = 0, # m
        start_y = 10, # m
        start_vx = 1, # m/s
        start_vy = 0, # m/s
        ball_mass = 0.05, # kg
        ball_radius = 0.03, # m
        dt = dt #s
    )

    drone = Drone(
        mass = 0.18,
        body_height = 2,
        body_width = 2,
        body_length = 2,
        arm_length = 0.086,
        initial_state = [0,0,0,0,0,0], # x, y, phi, vx, vy, vphi
        dt = dt
    )

    
    while True:

        timesteps.append(time)
        ball_states.append(ball.state)
        # drone_states.append(drone.state)
        ball.step()
        # drone.step()

        print(f"Ball (x,y): ({ball.x}, {ball.y})")
        ball_pred = drone.predict_ball_position(ball.x, ball.y, ball.vx, ball.vy)
        print(f"Ball pred: {ball_pred}")
        
        time += dt
        
        if time > 5:
            break

    #     if ball hits drone:
    #         parallel axis

    # plt.plot(timesteps, ball_states)
    # plt.show()

    return


if __name__ == "__main__":
    main()