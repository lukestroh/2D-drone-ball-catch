import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

g = 9.81 # m/s**2
# The coefficient of restitution for bounces (-v_up/v_down).
cor = 0.2 # unitless
# realistically I should model the ball as a spring mass damper system
# I have code that does that on my pc but forgot to upload it to the repo
ballMass = 1 #kg #just for now
# The time step for the animation.
dt = 0.005
xMin = 0 # m
xMax = 5 # m
yMin = 0 # m
yMax = 4 # m
plotlen = 10


# multiple iterations just to show that the drone does catch the ball
for i in range(plotlen):

    ballStart = [random.uniform(xMin+0.5, xMax-0.5),random.uniform(yMax/2, yMax)] # initial positi#on of ball
    ballInitVelocity = [random.uniform(-2,2),random.uniform(0,2)] # initial velocity of ball
    dronePos = [random.uniform(xMin+0.5, xMax-0.5),random.uniform(yMin+1, yMax/2)] # initial position of drone
    droneInitVelocity = [0,0] # initial velocity of drone

    def get_pos(t=0):
        """Math for position of the ball"""
        x, y, vx, vy = ballStart[0], ballStart[1], ballInitVelocity[0], ballInitVelocity[1]
        while xMin < x < xMax:
            t += dt
            x += vx * dt
            y += vy * dt
            vy -= g * dt
            if (dronePos[1]-0.1 < y < dronePos[1]) and (dronePos[0]-0.25 <= x <= dronePos[0]+0.25):
                # if the ball is intersecting the drone, 'catch' the ball
                y = dronePos[1]
                vy = -vy * cor
                vx -= vx * 0.35 # just to kind of slow the rolling down
            elif (y < yMin):
                y = yMin
                vy = -vy * cor
                vx -= vx * 0.35 # just to kind of slow the rolling down
            yield x, y

    def init():
        """Initialize the animation figure."""
        ax.set_xlim(xMin, xMax)
        ax.set_ylim(yMin, yMax)
        ax.set_xlabel('$x$ (m)')
        ax.set_ylabel('$y$ (m)')
        line.set_data(xdata, ydata)
        ball.set_center((ballStart[0], ballStart[1]))
        height_text.set_text(f'Current Height: {ballStart[1]:.1f} m')
        return line, ball, height_text

    def animate(pos):
        """For each frame, advance the animation to the new position, pos."""
        x, y = pos
        xdata.append(x)
        ydata.append(y)
        line.set_data(xdata, ydata)
        ball.set_center((x, y))
        height_text.set_text(f'Current Height: {y:.1f} m')
        return  line, ball, height_text

    # Set up a new Figure, with equal aspect ratio so the ball appears round.

    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    line, = ax.plot([], [], lw=.5, color='red') # this is code if we want to map the ball trajectory
    # drawing the drone with no motion yet :)
    drone = plt.plot([dronePos[0]-0.25, dronePos[0]+0.25], [dronePos[1],dronePos[1]], color='black')
    drone_prop_left = plt.plot([dronePos[0]-0.25, dronePos[0]-0.25], [dronePos[1]-0.05,dronePos[1]+0.1], color='black')
    drone_prop_right = plt.plot([dronePos[0]+0.25, dronePos[0]+0.25], [dronePos[1]-0.05,dronePos[1]+0.1], color='black')
    # draw a point at the ball's initial position
    ballOrigin = plt.plot(ballStart[0], ballStart[1], 'r.')
    originText = ax.text(ballStart[0]+0.15, ballStart[1]-0.05, f'Ball Initial Position')
    # drawing the ball being dropped
    ball = plt.Circle((ballStart[0], ballStart[1]), 0.08)
    height_text = ax.text(xMin+0.1, yMax-0.2, f'Current Height: {ballStart[1]:.1f} m')
    ax.add_patch(ball)
    xdata, ydata = [], []

    # animating the whole thing
    interval = 1000*dt
    ani = animation.FuncAnimation(fig, animate, get_pos, blit=True,
                        interval=interval, repeat=False, init_func=init)
    plt.show()
    plt.close(fig)