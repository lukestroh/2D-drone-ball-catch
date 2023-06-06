from drone import Drone
from ball import Ball
import numpy as np

import matplotlib.pyplot as plt

from pprint import pprint

class SimError(Exception):
    pass

def main():
    dt = 0.001
    t0 = 0
    sim_time = 6 # s

    ball = Ball(
        start_x = 0, # m
        start_y = 10.0, # m
        start_vx = 0.0, # m/s
        start_vy = 0, # m/s
        ball_mass = 0.03, # kg
        ball_radius = 0.03, # m
        dt = dt #s
    )

    drone = Drone(
        mass = 0.25, # kg
        body_height = 0.1, # m
        body_width = 0.36, # m
        body_length = 0.36, # m
        arm_length = 0.086, # m
        initial_state = [0,1.0,0,0,0,0], # x, y, phi, vx, vy, vphi
        dt = dt # s
    )
    
    # Pre-allocate data arrays
    timesteps = np.linspace(t0, sim_time, int(sim_time/dt))
    drone_states = np.zeros((len(drone.state), int(sim_time/dt)), dtype=float)
    ball_states = np.zeros((len(ball.state), int(sim_time/dt)), dtype=float)

    # Set loop initial conditions
    i = 0
    collided = False
    impulse = False
    while i < int(sim_time/dt):
        ball_states[:,i] = ball.state
        drone_states[:,i] = drone.state
        
        # If collision with the ball has not occurred
        if not collided:
            ball.step()
            drone.step()

            # Get ball target position
            ball_pred = drone.predict_ball_position(ball=ball)
            drone.update_target_state(ball_pred, drone.y, 0, 0, 0, 0)

            # Quit if the vall falls below the drone and we haven't collided
            if ball.y < drone.y:
                raise SimError("The ball was not caught by the drone.")
            
            # Check for collision
            collided = drone.detect_impact(ball=ball)
            if collided:

                print(ball.state, drone.state)

                drone.update_moment_of_inertia(ball=ball)
                drone.update_target_state(drone.x, drone.y, 0, 0, 0, 0)
                drone.A, drone.B = drone.linearize_dynamics(ball=ball, impulse=True)
                drone.K = drone.compute_LQR_gain()
                impulse = True

        # If collision has occurred     
        else:
            # Add the force due to impulse to B for one timestep
            if impulse:
                ball.step(collided=True, drone=drone)
                drone.step()
                drone.A, drone.b = drone.linearize_dynamics(ball=ball, impulse=False)
                drone.K = drone.compute_LQR_gain()
                impulse = False
            else:
                ball.step(collided=True, drone=drone)
                drone.step()

        i += 1


    # Plotting
    # x
    fig=plt.figure()
    ax1 = fig.add_subplot(311)
    ax1.plot(timesteps, drone_states[0], 'k')
    ax1.plot(timesteps, drone_states[3], '#ff7f0e')
    ax1.plot(timesteps, np.zeros(len(timesteps)), 'k--', linewidth=0.75)
    ax1.legend(["x", "vx"], loc=1, fontsize=8)
    ax1.set_xlim([t0,sim_time])
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Horizontal Location (m)")
    #plt.xlim([0,5])

    # y
    ax2 = fig.add_subplot(312)
    ax2.plot(timesteps, drone_states[1], 'k')
    ax2.plot(timesteps, drone_states[4], '#ff7f0e')
    ax2.legend(["y", "vy"], loc=1, fontsize=8)
    ax2.set_xlim([t0,sim_time])
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Vertical Location (m)")
    #plt.xlim([0,5])

    # Phi
    ax3 = fig.add_subplot(313)
    ax3.plot(timesteps, drone_states[2], 'k')
    ax3.plot(timesteps, drone_states[5], '#ff7f0e')
    ax3.plot(timesteps, np.zeros(len(timesteps)), 'k--', linewidth=0.5)
    ax3.legend(["phi", "vphi"], loc=4, fontsize=8)
    ax3.set_xlim([t0,sim_time])
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Angle (degrees)")
    plt.show()

    return


""" TODO:
    # Code
    *Finish detect_impact
    *update moment of inertia for original body (done)
    *create function to update MOI after impact (done?)
        *needs updated COM
    *create function that fixes ball to drone body (corresponding function in Ball?)
    *write impulse_response function
        *append this to current datatype? syncing time and state important

    # Data
    Figure out if data makes sense 
        Does drone flip upside down?
        Does the drone lose y position after collision?
        Does it make sense if the ball has 0 vx?
            Currently, no
        Does it make sense if the ball has +/- vx?
            Currently, no

    Generate control plots
    Generate Force/torque plots
    Generate ball plot?
    Simplify the animation to make the drone just a rectangle
"""


if __name__ == "__main__":
    main()