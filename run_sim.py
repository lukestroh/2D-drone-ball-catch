from drone import Drone, SimError
from ball import Ball
import numpy as np

import matplotlib.pyplot as plt
import plotly.subplots as sp
import plotly.graph_objects as go



def main():
    dt = 0.001
    t0 = 0
    sim_time = 5 # s

    ball = Ball(
        start_x = 0, # m
        start_y = 10.0, # m
        start_vx = 0.0, # m/s
        start_vy = 0, # m/s
        ball_mass = 0.05, # kg
        ball_radius = 0.03, # m
        dt = dt #s
    )

    drone = Drone(
        mass = 0.25, # kg
        body_height = 0.1, # m
        body_width = 0.36, # m
        body_length = 0.36, # m
        arm_length = 0.086, # m
        initial_state = [0,1,0,0,0,0], # x, y, phi, vx, vy, vphi
        dt = dt # s
    )

    # Dict for gathering step and K matrix updates
    ball_pred = drone.predict_ball_position(ball=ball)
    drone.update_target_state(ball_pred, drone.y, 0, 0, 0, 0)
    control_gains: dict = {0: dict(K=drone.K, target=drone.target_state, mass=drone.m)}
    drone.state = [0,1,0,0,0,0]
    
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
                drone.update_moment_of_inertia(ball=ball)
                drone.update_target_state(drone.x, drone.y, 0, 0, 0, 0)
                drone.A, drone.B = drone.linearize_dynamics(ball=ball)
                drone.K = drone.compute_LQR_gain()
                control_gains[i] = dict(K=drone.K, target=drone.target_state, mass=drone.m+ball.mass)
                impulse = True

        # If collision has occurred     
        else:
            # If impulse step, step with force due to impulse to B for one timestep
            if impulse:
                # Perfectly elastic conservation of momentum
                drone.vx = (drone.m * drone.vx + ball.mass * ball.vx) / (drone.m + ball.mass)
                drone.vy = (drone.m * drone.vy + ball.mass * ball.vy) / (drone.m + ball.mass)
                drone.state = [
                    drone.state[0],
                    drone.state[1],
                    drone.state[2],
                    drone.vx,
                    drone.vy,
                    drone.state[5]
                ]
                
                ball.step(collided=True, drone=drone)
                drone.step()
                impulse = False

            else:
                ball.step(collided=True, drone=drone)
                drone.step()

        i += 1


    # Plotting
    # x
    fig=plt.figure()
    
    ax1 = fig.add_subplot(411)
    ax1.set_title(f"LQR response of a 2D quadcopter catching a ball")
    ax1.plot(timesteps, drone_states[0])#, '#015aaa')
    ax1.plot(timesteps, drone_states[3])#, '#e68a00')
    ax1.plot(timesteps, np.zeros(len(timesteps)), 'k--', linewidth=0.75)
    ax1.legend(["x", "vx"], loc=1, fontsize=8)
    ax1.set_xlim([t0,sim_time])
    ax1.set_ylabel("Horizontal Position \n (m)/(m/s)")

    # y
    ax2 = fig.add_subplot(412)
    ax2.plot(timesteps, drone_states[1])
    ax2.plot(timesteps, drone_states[4])
    ax2.plot(timesteps, np.zeros(len(timesteps)), 'k--', linewidth=0.75)
    ax2.legend(["y", "vy"], loc=1, fontsize=8)
    ax2.set_xlim([t0,sim_time])
    ax2.set_ylabel("Vertical Position \n (m)/(m/s)")

    # Phi
    ax3 = fig.add_subplot(413)
    ax3.plot(timesteps, drone_states[2])
    ax3.plot(timesteps, drone_states[5])
    ax3.plot(timesteps, np.zeros(len(timesteps)), 'k--', linewidth=0.5)
    ax3.legend(["phi", "vphi"], loc=4, fontsize=8)
    ax3.set_xlim([t0,sim_time])
    ax3.set_ylabel("Angle \n(rad)/(rad/s)")

    # Control
    ax4 = fig.add_subplot(414)
    u1 = np.zeros(len(timesteps))
    u2 = np.zeros(len(timesteps))
    for i, K_ref in control_gains.items():
        u1[i:] = np.sqrt(
            (-K_ref["K"][0][0] * (drone_states[0,i:] - K_ref["target"][0]))**2 + 
            (-K_ref["K"][0][1] * (drone_states[1,i:] - K_ref["target"][1]))**2
        ) + K_ref["mass"] * drone.g
        u2[i:] = K_ref['K'][1][2] * drone_states[2,i:]
    ax4.plot(timesteps, u1)
    ax4.plot(timesteps, u2)
    ax4.legend(["u1", "u2"])
    ax4.set_xlim([t0, sim_time])
    ax4.set_ylabel("Control inputs")
    
    # # Forces
    # ax5 = fig.add_subplot(515)
    # F1 = u1/2 + u2/(drone.w/2 + drone.L)
    # F2 = u1/2 - u2/(drone.w/2 + drone.L)
    # ax5.plot(timesteps, F1)
    # ax5.plot(timesteps, F2)
    # ax5.legend(["F1", "F2"])
    # ax5.set_xlim([t0, sim_time])
    # ax5.set_ylabel("Motor forces (N)")
    # ax5.set_xlabel("Time (s)")
    plt.show()

    return

if __name__ == "__main__":
    main()