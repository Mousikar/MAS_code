import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from PIL import Image
from PIL import Image, ImageFile
ImageFile.LOAD_TRUNCATED_IMAGES = True

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import imageio

# 定义智能体类
class Agent:
    def __init__(self, position):
        self.position = np.array(position)

    def move_towards(self, target):
        # 将智能体朝着目标点移动
        direction = target - self.position
        self.position += direction * 0.1

# 定义目标类
class Target:
    def __init__(self, position):
        self.position = np.array(position)




# 定义智能体类和目标类，以及surround_control函数...

def plot_scene(i, agents, target):
    plt.clf()
    for agent in agents:
        plt.scatter(agent.position[0], agent.position[1], c='blue', marker='o')
    plt.scatter(target.position[0], target.position[1], c='red', marker='x')
    plt.xlim(-5, 5)
    plt.ylim(-5, 5)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Multi-Agent Surround Control')
    plt.grid(True)

# # 创建一个列表来保存每一帧的图像
# frames = []

# def surround_control(agents, target, num_steps=100):
#     fig = plt.figure()
#     ani = FuncAnimation(fig, plot_scene, frames=num_steps, fargs=(agents, target), blit=False)

#     for i in range(num_steps):
#         # 计算合围的中心点
#         center = np.mean([agent.position for agent in agents], axis=0)
        
#         # 智能体向目标点移动
#         for agent in agents:
#             agent.move_towards(center)

#         # 保存当前帧图像
#         plt.clf()
#         plot_scene(i, agents, target)
#         fig.canvas.draw()
#         buf = fig.canvas.tostring_rgb()
#         ncols, nrows = fig.canvas.get_width_height()
#         frames.append(np.frombuffer(buf, dtype=np.uint8).reshape(nrows, ncols, 3))

#     # 保存GIF动画
#     imageio.mimsave('surround_control.gif', frames, fps=10)

# def main():
#     num_agents = 5
#     agents = [Agent([np.random.uniform(-5, 5), np.random.uniform(-5, 5)]) for _ in range(num_agents)]
#     target = Target([0, 0])

#     surround_control(agents, target)

# if __name__ == "__main__":
#     main()



# 创建一个列表来保存每一帧的图像
frames = []

# 合围控制函数
def surround_control(agents, target, num_steps=20):
    for step in range(num_steps):
        # 计算合围的中心点
        center = np.mean([agent.position for agent in agents], axis=0)
        
        # 智能体向目标点移动
        for agent in agents:
            agent.move_towards(target.position)

        # 绘制合围过程并将当前图像添加到帧列表中
        plot_scene(agents, target)
        plt.savefig('temp_frame.png')
        plt.pause(0.1)
        frames.append(Image.open('temp_frame.png'))

    # 保存GIF动画
    frames[0].save('surround_control.gif', save_all=True, append_images=frames[1:], optimize=False, duration=10)

def plot_scene(agents, target):
    plt.clf()
    for agent in agents:
        plt.scatter(agent.position[0], agent.position[1], c='blue', marker='o')
    plt.scatter(target.position[0], target.position[1], c='red', marker='x')
    plt.xlim(-5, 5)
    plt.ylim(-5, 5)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Multi-Agent Surround Control')
    plt.grid(True)

def main():
    num_agents = 5
    agents = [Agent([np.random.uniform(-5, 5), np.random.uniform(-5, 5)]) for _ in range(num_agents)]
    target = Target([0, 0])

    plt.ion()
    surround_control(agents, target)
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
