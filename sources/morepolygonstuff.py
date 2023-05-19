import matplotlib.patches as patches
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

vertices = np.array([[1.0, 1.0], [1.0,2.0], [2.0,3.0]])
blob = patches.Polygon(vertices)
# why did i program this
# help me please. i have lost my mind
def init():
    return blob,
def animate(pos):
    vertices[:,0] =vertices[:,0]*np.cos(-pos)-vertices[:,1]*np.sin(-pos)
    vertices[:,1] =vertices[:,0]*np.sin(-pos)+vertices[:,1]*np.cos(-pos)
    blob.set_xy(vertices)
    return blob,
def pos():
    x = 0.1
    yield x
fig, ax = plt.subplots()
ax.set_xbound(0, 10)
ax.set_ybound(0,10)
ax.add_patch(blob)


ani = animation.FuncAnimation(fig, animate, pos, init_func=init, interval=100, blit=True)

plt.show()