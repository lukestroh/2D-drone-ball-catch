import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class Drone():
    def __init__(self):
        self.x0 = 1
        self.y0 = 1
        self.propWidth = 0.125
        self.propHeight = 0.5
        self.frameThickness = 0.05
        self.frameWidth = 1.0
        self.drone_vertices = np.array([
        [self.x0, self.y0],
        [self.x0, self.y0+self.propHeight],
        [self.x0-self.propWidth,  self.y0+self.propHeight],
        [self.x0-self.propWidth,  self.y0+self.propHeight+self.frameThickness],
        [self.x0+(2*self.propWidth)-2*self.frameThickness,  self.y0+self.propHeight+self.frameThickness],
        [self.x0+(2*self.propWidth)-2*self.frameThickness,  self.y0+self.propHeight],
        [self.x0+(1*self.propWidth)-self.frameThickness-0.03,  self.y0+self.propHeight],
        [self.x0+(1*self.propWidth)-self.frameThickness-0.03,  self.y0+self.frameThickness],
        [self.x0+(1*self.propWidth)-self.frameThickness+self.frameWidth,  self.y0+self.frameThickness],
        [self.x0+(1*self.propWidth)-self.frameThickness+self.frameWidth,  self.y0+self.propHeight],
        [self.x0-self.frameThickness+self.frameWidth,  self.y0+self.propHeight],
        [self.x0-self.frameThickness+self.frameWidth,  self.y0+self.propHeight+self.frameThickness],
        [self.x0-self.frameThickness+self.frameWidth+(2*self.propWidth)+self.frameThickness,  self.y0+self.propHeight+self.frameThickness],
        [self.x0-self.frameThickness+self.frameWidth+(2*self.propWidth)+self.frameThickness,  self.y0+self.propHeight],
        [self.x0-self.frameThickness+self.frameWidth+(1*self.propWidth)+self.frameThickness,  self.y0+self.propHeight],
        [self.x0-self.frameThickness+self.frameWidth+(1*self.propWidth)+self.frameThickness,  self.y0]])
        self.drone = patches.Polygon(self.drone_vertices)

    def draw_drone(self):
        """Code for plotting a static image of the drone"""
        fig, ax = plt.subplots()
        ax.set_xbound(0, 5)
        ax.set_ybound(0,5)
        ax.add_patch(self.drone)
        plt.show()


D = Drone()

xdata, ydata = [], []
def get_pos(t=0):
    x, y, vx, vy, dt = 1, 1, 10, 0, 0.1 #initial positions and velocities
    while 0< x < 5:
        t += dt
        x += vx * dt
        y += vy * dt
        if D.drone_vertices[0:0] != [2, 1]:
            vx -= 0.1
    yield x, y

def init():
    """Initialize the animation figure."""
    ax.set_xlim(0, 5)
    ax.set_ylim(0, 5)
    ax.set_xlabel('$x$ (m)')
    ax.set_ylabel('$y$ (m)')
    D.drone.set_xy(D.drone_vertices)
    #line.set_data(xdata, ydata)
    #ball.set_center((ballStart[0], ballStart[1]))
    #height_text.set_text(f'Current Height: {ballStart[1]:.1f} m')
    return D.drone,

def animate(pos):
    """For each frame, advance the animation to the new position, pos."""
    x, y = pos
    xdata.append(x)
    ydata.append(y)
    D.drone_vertices[:,0] += x
    #self.drone.set_data(xdata, ydata)
    D.drone.set_xy(D.drone_vertices)
    #line.set_data(xdata, ydata)
    #ball.set_center((x, y))
    #height_text.set_text(f'Current Height: {y:.1f} m')
    return  D.drone,

fig, ax = plt.subplots()
ax.set_xbound(0, 10)
ax.set_ybound(0,10)
ax.add_patch(D.drone)
ani = animation.FuncAnimation(fig, animate, get_pos, blit=True,
                    interval=1000, repeat=False, init_func=init)
plt.show()
