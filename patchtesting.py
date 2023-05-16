import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

x0 = 1
y0 = 1
propWidth = 0.125
propHeight = 0.5
frameThickness = 0.05
frameWidth = 1.0
drone_vertices = np.array([
[x0, y0],
[x0, y0+propHeight],
[x0-propWidth,  y0+propHeight],
[x0-propWidth,  y0+propHeight+frameThickness],
[x0+(2*propWidth)-2*frameThickness,  y0+propHeight+frameThickness],
[x0+(2*propWidth)-2*frameThickness,  y0+propHeight],
[x0+(1*propWidth)-frameThickness-0.03,  y0+propHeight],
[x0+(1*propWidth)-frameThickness-0.03,  y0+frameThickness],
[x0+(1*propWidth)-frameThickness+frameWidth,  y0+frameThickness],
[x0+(1*propWidth)-frameThickness+frameWidth,  y0+propHeight],
[x0-frameThickness+frameWidth,  y0+propHeight],
[x0-frameThickness+frameWidth,  y0+propHeight+frameThickness],
[x0-frameThickness+frameWidth+(2*propWidth)+frameThickness,  y0+propHeight+frameThickness],
[x0-frameThickness+frameWidth+(2*propWidth)+frameThickness,  y0+propHeight],
[x0-frameThickness+frameWidth+(1*propWidth)+frameThickness,  y0+propHeight],
[x0-frameThickness+frameWidth+(1*propWidth)+frameThickness,  y0]])
drone = patches.Polygon(drone_vertices)

def draw_drone(self):
    """Code for plotting a static image of the drone"""
    fig, ax = plt.subplots()
    ax.set_xbound(0, 5)
    ax.set_ybound(0,5)
    ax.add_patch(drone)
    plt.show()




xdata, ydata = [], []
def get_pos(t=0):
    x, y, vx, vy, dt = 1, 1, 10, 0, 0.1 #initial positions and velocities
    while 0< x < 5:
        t += dt
        x += vx * dt
        y += vy * dt
        if drone_vertices[0][0] != 2 and drone_vertices[0][1] != 1:
            vx -= 0.1
    yield x, y

def init():
    """Initialize the animation figure."""
    ax.set_xlim(0, 5)
    ax.set_ylim(0, 5)
    ax.set_xlabel('$x$ (m)')
    ax.set_ylabel('$y$ (m)')
    drone.set_xy(drone_vertices)
    #line.set_data(xdata, ydata)
    #ball.set_center((ballStart[0], ballStart[1]))
    #height_text.set_text(f'Current Height: {ballStart[1]:.1f} m')
    return drone,

def animate(pos):
    """For each frame, advance the animation to the new position, pos."""
    x, y = pos
    xdata.append(x)
    ydata.append(y)
    drone_vertices[:,0] += x
    #drone.set_data(xdata, ydata)
    drone.set_xy(drone_vertices)
    #line.set_data(xdata, ydata)
    #ball.set_center((x, y))
    #height_text.set_text(f'Current Height: {y:.1f} m')
    return  drone,

fig, ax = plt.subplots()
ax.set_xbound(0, 10)
ax.set_ybound(0,10)
ax.add_patch(drone)
ani = animation.FuncAnimation(fig, animate, get_pos, blit=True,
                    interval=1000, repeat=False, init_func=init)
plt.show()
