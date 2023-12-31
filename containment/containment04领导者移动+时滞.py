import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial import ConvexHull
from scipy.integrate import solve_ivp
from ddeint import ddeint

# 领导者轨迹
def track_leader(leader_positions,t):
    temp=np.ones([num_leaders, 2])
    a = (10-np.max(leader_positions[:,0]))/t_sum
    # b = 0 #走直线
    b = (10-np.max(leader_positions[:,1]))/t_sum    # 走斜线
    # b = np.min(leader_positions[:,1])                 # 走sin线
    temp[:,0]=a*t*np.ones([num_leaders])
    temp[:,1]=b*t*np.ones([num_leaders])    # 走直线 斜线
    # temp[:,1]=b*np.sin(2*t)*np.ones([num_leaders])    # 走sin线
    leader_positions_t = temp + leader_positions
    return leader_positions_t

# Define the number of leaders and followers
num_leaders = 6
# num_leaders = 3
num_followers = 6
num_agents = num_leaders + num_followers

# Define the communication topology for followers
follower_topology =  {0: [1, 3], 1: [0, 2], 2: [1], 3: [0, 4], 4: [3, 5], 5: [4]}

# Define the communication topology for leaders
leader_topology = {0: [0], 1: [1], 2: [2], 3: [3], 4: [4], 5: [5]}
# leader_topology = {0: [0], 1: [1], 2: [2], 3: [], 4: [], 5: []}

# Create the communication topology matrix
communication_topology = np.zeros((num_followers, num_followers))
for i in range(num_followers):
    neighbors = follower_topology[i]
    for neighbor in neighbors:
        communication_topology[i, neighbor] = 1

# Compute the Laplacian matrix
degree_matrix = np.diag(np.sum(communication_topology, axis=1))
laplacian_matrix = degree_matrix - communication_topology

print("Communication Topology Matrix:")
print(communication_topology)

print("\nLaplacian Matrix:")
print(laplacian_matrix)

# Create A0
A0 = np.zeros((num_followers, num_followers))
for i in range(num_followers):
    leaders = leader_topology[i]
    A0[i, i] = len(leaders)

print("\nA0:")
print(A0)

L1=laplacian_matrix+A0
print("\nL1:")
print(L1)

# Create L2
L2 = np.zeros((num_followers, num_leaders))
for i in range(num_followers):
    leaders = leader_topology[i]
    for leader in leaders:
        L2[i, leader] = -1

print("\nL2:")
print(L2)


# Define the maximum movement distance per iteration for followers
max_movement_followers = 0.1

# Define the number of iterations
num_iterations = 200

# Initialize leader positions randomly in the range [0, 10)
# leader_positions = 10 * np.random.rand(num_leaders, 2)
leader_positions=0.5*np.ones([num_leaders, 2]) + 4 * np.random.rand(num_leaders, 2)

# Initialize follower positions randomly in the range [0, 10)
follower_positions = 10 * np.random.rand(num_followers, 2)
# follower_positions=4*np.ones([num_leaders, 2]) + 6 * np.random.rand(num_leaders, 2)
follower_positions=np.array([[0, 6],
                            [0, 6],
                            [0, 6],
                            [0, 6],
                            [0, 6],
                            [0, 6]]) + 4 * np.random.rand(num_leaders, 2)

t_sum=20 # 秒
dmax=0.2   # 可以修改时延
t_span = (0, t_sum)  # 时间范围
t = np.linspace(0, t_sum, num_iterations)  # 用于绘制的时间点

# 微分方程的初值
X=follower_positions.reshape([1,num_followers*2])

initial_condition = X[0]  # 初始条件

# solution = solve_ivp(matrix_differential_equation, t_span, initial_condition, dense_output=True)

# Y = solution.sol(t)

# 控制方程
def model(Y, t, d):
    X = Y(t)
    Xd = Y(t - d)
    XX = Xd.reshape([num_followers, 2])
    cols=np.shape(XX)[1]
    # print(cols)
    dX_dt = np.zeros((num_followers, cols))
    leader_positions_t = track_leader(leader_positions,t)
    for i in range(cols):
        dX_dt[:,i] = - np.dot(L1,XX[:,i]) - np.dot(L2,leader_positions_t[:,i])
    # print(dX_dt)
    dX_dt = dX_dt.reshape([1,num_followers*2])
    return  dX_dt

g = lambda t: X[0]
print(g)


fig, ax = plt.subplots()
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

#--------------------------------------------------------------
# 绘制每一列数据对应的曲线，并设置不同的颜色
colors = ['red', 'green', 'blue', 'orange', 'purple', 'yellow']
linestyles = ['--', '-']
line_alpha = [0.2,0.7]
count = 0
for d in [0, dmax]: # 可以修改时滞时间
    print("Computing for d=%.02f" % d)
    yy = ddeint(model, g, t, fargs=(d,))
    print(yy.shape)
    # WE PLOT X AGAINST Y
    for k in range(num_followers):
        x=[]
        y=[]
        for i in range(num_iterations):
            x.append(yy[i,:].reshape([num_followers, 2])[k,0])
            y.append(yy[i,:].reshape([num_followers, 2])[k,1]) 
        ax.plot(x, y, lw=2, color=colors[k], linestyle=linestyles[count], alpha=line_alpha[count])
    count+=1

# 添加图例
ax.legend()
#--------------------------------------------------------------

# Function to update the plot in each animation frame
def update(frame):
    follower_positions=yy[frame,:].reshape([num_followers, 2])
    # 领导者的位置
    leader_positions_t = track_leader(leader_positions,frame*t_sum/num_iterations)

    leader_scatter.set_offsets(leader_positions_t)
    follower_scatter.set_offsets(follower_positions)

    # Update labels
    for i in range(num_leaders):
        leader_labels[i].set_position((leader_positions_t[i, 0] + 0.8, leader_positions_t[i, 1] + 0.1))
    for i in range(num_followers):
        follower_labels[i].set_position((follower_positions[i, 0] + 0.8, follower_positions[i, 1] + 0.1))

    # Calculate the convex hull for leaders only
    hull = ConvexHull(leader_positions_t)
    hull_vertices = np.append(hull.vertices, hull.vertices[0])  # Closing the hull by connecting the first vertex again
    hull_line.set_xdata(leader_positions_t[hull_vertices, 0])
    hull_line.set_ydata(leader_positions_t[hull_vertices, 1])

    return leader_scatter, follower_scatter, hull_line, *leader_labels, *follower_labels

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=num_iterations, interval=t_sum/num_iterations, blit=True)

# Save the animation as a gif
ani.save('containment.gif', writer='pillow')
