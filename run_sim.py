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
        start_y = 10.0, # m
        start_vx = 1.0, # m/s
        start_vy = 0, # m/s
        ball_mass = 0.5, # kg
        ball_radius = 0.03, # m
        dt = dt #s
    )

    drone = Drone(
        mass = 0.18, # kg
        body_height = 0.1, # m
        body_width = 0.36, # m
        body_length = 0.36, # m
        arm_length = 0.086, # m
        initial_state = [0,1.0,0,0,0,0], # x, y, phi, vx, vy, vphi
        dt = dt # s
    )

    
    timesteps = np.linspace(0,5,int(sim_time/dt))
    drone_states = np.zeros((len(drone.state), int(sim_time/dt)), dtype=float)
    ball_states = np.zeros((len(ball.state), int(sim_time/dt)), dtype=float)
    print("MOI1: ", drone.Ixx)
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

                # print(ball.state, drone.state)

                drone.update_moment_of_inertia(ball=ball)
                drone.update_target_state(drone.state[0], drone.state[1], drone.state[2], drone.state[3], drone.state[4], drone.state[5])
                drone.A, drone.B = drone.linearize_dynamics()
                drone.K = drone.compute_LQR_gain()
                
        else:
            ball.step(collided=True, drone=drone)
            drone.step()

        i += 1
    
    print("MOI2: ", drone.Ixx)

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
    fig=plt.figure()
    ax1 = fig.add_subplot(311)
    ax1.plot(timesteps, drone_states[0], 'k')
    ax1.plot(timesteps, drone_states[3], '#ff7f0e')
    ax1.plot(timesteps, np.zeros(len(timesteps)), 'k--', linewidth=0.75)
    ax1.legend(["x", "vx"], loc=1, fontsize=8)
    ax1.set_xlim([0,3])
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Horizontal Location (m)")
    #plt.xlim([0,5])
    ax2 = fig.add_subplot(312)
    ax2.plot(timesteps, drone_states[1], 'k')
    ax2.plot(timesteps, drone_states[4], '#ff7f0e')
    ax2.legend(["y", "vy"], loc=1, fontsize=8)
    ax2.set_xlim([0,3])
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Vertical Location (m)")
    #plt.xlim([0,5])
    # the y vy plot is redundant because vy=0
    ax3 = fig.add_subplot(313)
    ax3.plot(timesteps, drone_states[2], 'k')
    ax3.plot(timesteps, drone_states[5], '#ff7f0e')
    ax3.plot(timesteps, np.zeros(len(timesteps)), 'k--', linewidth=0.5)
    ax3.legend(["phi", "vphi"], loc=4, fontsize=8)
    ax3.set_xlim([0,3])
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Angle (degrees)")
    plt.show()

    return


if __name__ == "__main__":
    main()