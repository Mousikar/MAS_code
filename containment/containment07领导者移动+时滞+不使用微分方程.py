import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import matplotlib.animation as animation

class Agent:
    def __init__(self, position, delaytime):
        self.position = np.array(position)
        self.velocity = np.zeros(2)  # 初始化速度为零
        
        self.delaytime = delaytime # 初始化时延
        self.delayposition = np.array(position) # 初始化延时信息为初始位置

    def move(self,deltat):
        # 根据当前速度更新位置
        self.position += self.velocity * deltat

    def update_delayposition(self,delayposition):
        # 更新延时信息
        self.delayposition = np.array(delayposition)

class FollowerAgent(Agent):
    def __init__(self, index, position, delaytime, neighbors, leader_indices):
        super().__init__(position, delaytime)
        self.index = index
        self.neighbors = neighbors
        self.leader_indices = leader_indices

    def update_velocity(self, a, b, agents, leader_agents):
        neighbor_positions = [agents[i].delayposition for i in self.neighbors]
        leader_positions = [leader_agents[i].delayposition for i in self.leader_indices]
        # self.velocity = a * (np.sum(neighbor_positions, axis=0) - len(self.neighbors) * self.position)
        # self.velocity -= b * (np.sum(leader_positions, axis=0) - len(self.leader_indices) * self.position)
        
        self.velocity = - a * (len(self.neighbors) * self.position - np.sum(neighbor_positions, axis=0)) 
        self.velocity = self.velocity - b * (len(self.leader_indices) * self.position - np.sum(leader_positions, axis=0))
        # return leader_positions

class LeaderAgent(Agent):
    def __init__(self, index, position, delaytime):
        super().__init__(position, delaytime)
        self.index = index
    # 领导者轨迹
    def track_leader(self,iter):
        # aa = 0
        # bb = 0  # 领导者不动
        aa = (10-np.max(leader_positions[:,0]))/t_sum   # 走直线 走斜线 走sin线
        bb = 0 #走直线
        # bb = (10-np.max(leader_positions[:,1]))/t_sum    # 走斜线
        # bb = 2*self.position[1] *np.cos(2*iter/t_sum)                # 走sin线
        self.velocity = np.array([aa, bb])

num_followers = 6
num_leaders = 3
num_leaders = 6
a = 1
b = 1

t_sum = 20
num_iterations = 100
deltat = t_sum/num_iterations
t_delay = 0.35
linjie = 8*int(np.floor(t_delay/deltat))+1
# 每个机器人时滞的步长
tau_follower = [linjie, linjie, linjie, linjie, linjie, linjie]
tau_leader =  [1,1,1,1,1,1]
# [1,2,3,4,1,2] [5,5,5,5,5,5] [4,4,4,4,4,4] [3,3,3,3,3,3] [2,2,2,2,2,2] [1,1,1,1,1,1] 
# [1,2,3,1,1,2]


leader_positions = 0.5 * np.ones([num_leaders, 2]) + 4 * np.random.rand(num_leaders, 2)
# follower_positions = 10 * np.random.rand(num_followers, 2)
# follower_positions=6*np.ones([num_followers, 2]) + 4 * np.random.rand(num_followers, 2)
follower_positions=np.array([[0, 6],
                            [0, 6],
                            [0, 6],
                            [0, 6],
                            [0, 6],
                            [0, 6]]) + 4 * np.random.rand(num_leaders, 2)

print(follower_positions)
print(leader_positions)
print('时间间隔', deltat)
print('跟随者时滞时长', deltat*np.array(tau_follower))
print('领导者时滞时长', deltat*np.array(tau_leader))

follower_topology = {0: [1, 3], 1: [0, 2], 2: [1], 3: [0, 4], 4: [3, 5], 5: [4]}
leader_topology = {0: [0], 1: [1], 2: [2], 3: [], 4: [], 5: []}
leader_topology = {0: [0], 1: [1], 2: [2], 3: [3], 4: [4], 5: [5]}

follower_agents = []
leader_agents = []
for i in range(num_followers):
    neighbors = follower_topology.get(i, [])
    # print(neighbors)
    leader_indices = [leader_idx for leader_idx, leaders in leader_topology.items() if i in leaders]
    follower_agent = FollowerAgent(i, follower_positions[i], tau_follower[i], neighbors, leader_indices)
    follower_agents.append(follower_agent)

for i in range(num_leaders):
    leader_agent = LeaderAgent(i, leader_positions[i], tau_leader[i])
    leader_agents.append(leader_agent)

# 初始化时滞
tau = np.max([tau_follower,tau_leader])
if tau==0:
    print('error')
follower_positions_history = []
leader_positions_history = []
for i in range(tau):
    follower_positions_history.append(follower_positions)
    leader_positions_history.append(leader_positions)

for iter in range(num_iterations):
    # for agent in follower_agents:
    # for leadagent in leader_agents:

    for agent in follower_agents:
        delayposition = follower_positions_history[-agent.delaytime][agent.index,:]
        agent.update_delayposition(delayposition)
        agent.update_velocity(a, b, follower_agents, leader_agents)
        agent.move(deltat)
    for leadagent in leader_agents:
        delayposition = leader_positions_history[-leadagent.delaytime][leadagent.index,:]
        leadagent.update_delayposition(delayposition)
        leadagent.track_leader(iter)
        leadagent.move(deltat)
    
    current_follower_positions = np.array([agent.position for agent in follower_agents])
    current_leader_positions = np.array([agent.position for agent in leader_agents])
    # print(current_follower_positions)
    follower_positions_history.append(current_follower_positions)
    leader_positions_history.append(current_leader_positions)

# 打印最终的智能体位置
# for i, agent in enumerate(follower_agents):
#     print(f"Agent {i+1}: {agent.position}")
# 打印历史位置
# print(follower_positions_history)
# print(leader_positions_history)


# 绘制智能体的运动轨迹并保存为图片
fig, ax  = plt.subplots(figsize=(8, 8))
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
leader_scatter = ax.scatter(leader_positions[:, 0], leader_positions[:, 1], c='r', marker='o', label='Leaders')
follower_scatter = ax.scatter(follower_positions[:, 0], follower_positions[:, 1], c='b', marker='o', label='Followers')
hull_line, = ax.plot([], [], c='r', linewidth=2)

leader_labels = []
follower_labels = []
for i in range(num_leaders):
    leader_labels.append(ax.text(leader_positions[i, 0], leader_positions[i, 1], f"Leader {i+1}", ha='center', va='center', color='r', fontsize=8, fontweight='bold', alpha=0.3))
for i in range(num_followers):
    follower_labels.append(ax.text(follower_positions[i, 0], follower_positions[i, 1], f"Follower {i+1}", ha='center', va='center', color='b', fontsize=8, fontweight='bold', alpha=0.3))

ax.legend(loc='upper right')

colors = ['red', 'green', 'blue', 'orange', 'purple', 'yellow']
linestyles = ['--', '-']
line_alpha = [0.2, 0.4, 0.6, 0.8, 1]
#--------------------------------------------------------------
for k in range(num_followers):
    x=[]
    y=[]
    for i,positions in enumerate(follower_positions_history):
        x.append(positions[k,0])
        y.append(positions[k,1])
    plt.plot(x, y, lw=2, color=colors[k], linestyle=linestyles[1])
for k in range(num_leaders):
    x=[]
    y=[]
    for i,positions in enumerate(leader_positions_history):
        x.append(positions[k,0])
        y.append(positions[k,1])
    plt.plot(x, y, lw=2, color=colors[k], linestyle=linestyles[0], alpha=line_alpha[1])#--------------------------------------------------------------

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Agent Trajectories')
plt.legend()
plt.grid()
plt.savefig('agent_trajectories.png')
# plt.show()

# 制作动画
def update(frame):
    follower_positions=follower_positions_history[frame].reshape([num_followers, 2])
    leader_positions=leader_positions_history[frame].reshape([num_leaders, 2])

    leader_scatter.set_offsets(leader_positions)
    follower_scatter.set_offsets(follower_positions)

    # Update labels
    for i in range(num_leaders):
        leader_labels[i].set_position((leader_positions[i, 0] + 0.8, leader_positions[i, 1] + 0.1))
    for i in range(num_followers):
        follower_labels[i].set_position((follower_positions[i, 0] + 0.8, follower_positions[i, 1] + 0.1))

    # Calculate the convex hull for leaders only
    hull = ConvexHull(leader_positions)
    hull_vertices = np.append(hull.vertices, hull.vertices[0])  # Closing the hull by connecting the first vertex again
    hull_line.set_xdata(leader_positions[hull_vertices, 0])
    hull_line.set_ydata(leader_positions[hull_vertices, 1])

    return leader_scatter, follower_scatter, hull_line, *leader_labels, *follower_labels

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=num_iterations, interval=0.5*t_sum/num_iterations, blit=True)

# Save the animation as a gif
ani.save('agent_trajectories.gif', writer='pillow')



