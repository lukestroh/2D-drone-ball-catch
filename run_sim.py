from drone import Drone
from ball import Ball
import numpy as np

import matplotlib.pyplot as plt

from pprint import pprint


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
    collided = False
    while i < int(sim_time/dt):
        # timesteps[i] = time
        ball_states[:,i] = ball.state
        drone_states[:,i] = drone.state

        if not collided:
            ball.step()
            drone.step()

            ball_pred = drone.predict_ball_position(ball=ball)
            drone.update_target_state(ball_pred, drone.y, 0, 0, 0, 0)
            
            time += dt
            
            if ball.y < drone.y:
                break
            
            # Check for collision
            collided = drone.detect_impact(ball=ball)
            if collided:
                print(ball.state, drone.state)
                drone.update_moment_of_inertia()
                drone.update_target_state(drone.state[0], drone.state[1], drone.state[2], drone.state[3], drone.state[4], drone.state[5])
                drone.A, drone.B = drone.linearize_dynamics()
                
        else:
            ball.step(collided=True, drone=drone)
            drone.step()

        i += 1
        

    # while True:
    #     if time >= 5:
    #         break
    #     drone_states[:,i] = drone.state
    #     drone.step()
    #     time += dt
    #     i += 1


    # # Get the response from the impulse of the ball
    # imp_data = drone.get_impulse_resp(t0=time, sim_time=sim_time, i=i)
    # pprint(vars(imp_data))

    # idx = i
    # # Append impulse data to original data
    # while i < int(sim_time/dt):
    #     drone_states[:,i] = imp_data.x[:,i-idx]
    #     i += 1

    



    # Plotting
    plt.plot(timesteps, drone_states.transpose())
    plt.legend(["x", "y", "phi", "vx", "vy", "vphi"])
    plt.show()

    return


if __name__ == "__main__":
    main()