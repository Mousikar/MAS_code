import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig, ax = plt.subplots()

x = np.random.normal(size=50)
y = np.random.normal(size=50)
u = np.random.normal(size=50)
v = np.random.normal(size=50)

q = ax.quiver(x, y, u, v, units='xy', scale=1.0)
qk = ax.quiverkey(q, 0.9, 0.9, 2, r'2 \frac{m}{s}', labelpos='E', coordinates='axes')

def update_quiver(num):
    u = 1
    v = 2
    print(u)
    q.set_UVC(u,v)
    return q,

ani = animation.FuncAnimation(fig, update_quiver, frames=400, interval=25, blit=False)
plt.show()
