from drone import Drone
from ball import Ball
import numpy as np

import matplotlib.pyplot as plt


def main():
    dt = 0.001
    time = 0
    sim_time = 5 # s

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
        mass = 0.18, # kg
        body_height = 0.1, # m
        body_width = 0.36, # m
        body_length = 0.36, # m
        arm_length = 0.086, # m
        initial_state = [0,1,0,0,0,0], # x, y, phi, vx, vy, vphi
        dt = dt # s
    )

    
    timesteps = np.linspace(0,5,int(sim_time/dt))
    drone_states = np.zeros((len(drone.state), int(sim_time/dt)), dtype=float)
    ball_states = np.zeros((len(ball.state), int(sim_time/dt)), dtype=float)

    i = 0
    while True:

        # timesteps[i] = time
        ball_states[:,i] = ball.state
        drone_states[:,i] = drone.state
        ball.step()
        drone.step()

        ball_pred = drone.predict_ball_position(ball=ball)
        drone.update_target_state(ball_pred, drone.y, 0, 0, 0, 0)
        
        time += dt
        
        if ball.y < drone.y:
            break

        if drone.detect_impact(ball=ball):
            print("True!")
            break
        
        i += 1

    imp_data = drone.get_impulse_resp(t0=time,  sim_time=sim_time, i=i)

    print(imp_data.outputs)
    # Plotting
    plt.plot(timesteps, drone_states.transpose())
    plt.legend(["x", "y", "phi", "vx", "vy", "vphi"])
    plt.show()

    return


if __name__ == "__main__":
    main()