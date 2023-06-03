from drone import Drone
from ball import Ball
import numpy as np

import matplotlib.pyplot as plt


def main():
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
        initial_state = [0,1,0,0,0,0], # x, y, phi, vx, vy, vphi
        dt = dt
    )

    dt = 0.001
    time = 0
    sim_time = 5 # s
    timesteps = np.zeros(sim_time/dt)
    drone_states = np.zeros((len(drone.state), sim_time/dt))
    ball_states = np.zeros((len(ball.state), sim_time/dt))

    
    while True:

        timesteps.append(time)
        ball_states.append(ball.state)
        drone_states.append(drone.state)
        ball.step()
        drone.step()

        ball_pred = drone.predict_ball_position(ball=ball)
        drone.update_target_state(ball_pred, drone.y, 0, 0, 0, 0)
        
        time += dt
        
        if ball.y < drone.y:
            break

        if drone.detect_impact(ball=ball):
            break
        

    # Plotting
    plt.plot(timesteps, drone_states)
    plt.legend(["x", "y", "phi", "vx", "vy", "vphi"])
    plt.show()

    return


if __name__ == "__main__":
    main()