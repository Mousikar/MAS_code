import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial import ConvexHull
from scipy.integrate import solve_ivp

def matrix_differential_equation(tt, X):
    X = X.reshape([num_followers, 2])
    cols=np.shape(X)[1]
    # print(cols)
    dX_dt = np.zeros((num_followers, cols))
    for i in range(cols):
        dX_dt[:,i] = - np.dot(L1,X[:,i]) - np.dot(L2,leader_positions[:,i])
    # print(dX_dt)
    dX_dt = dX_dt.reshape([1,num_followers*2])
    return dX_dt


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
leader_positions = 10 * np.random.rand(num_leaders, 2)
leader_positions=0*np.ones([num_leaders, 2]) + 4 * np.random.rand(num_leaders, 2)

# Initialize follower positions randomly in the range [0, 10)
follower_positions = 10 * np.random.rand(num_followers, 2)
follower_positions=4*np.ones([num_leaders, 2]) + 6 * np.random.rand(num_leaders, 2)

t_sum=5 # 秒
t_span = (0, t_sum)  # 时间范围
t = np.linspace(0, t_sum, num_iterations)  # 用于绘制的时间点

# 微分方程的初值
X=follower_positions.reshape([1,num_followers*2])

initial_condition = X[0]  # 初始条件

solution = solve_ivp(matrix_differential_equation, t_span, initial_condition, dense_output=True)

Y = solution.sol(t)

fig, ax = plt.subplots()
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
leader_scatter = ax.scatter(leader_positions[:, 0], leader_positions[:, 1], c='r', marker='o', label='Leaders')
follower_scatter = ax.scatter(follower_positions[:, 0], follower_positions[:, 1], c='b', marker='o', label='Followers')
hull_line, = ax.plot([], [], c='r', linewidth=2)

leader_labels = []
follower_labels = []
for i in range(num_leaders):
    leader_labels.append(ax.text(leader_positions[i, 0], leader_positions[i, 1], f"Leader {i+1}", ha='center', va='center', color='r', fontsize=8, fontweight='bold'))
for i in range(num_followers):
    follower_labels.append(ax.text(follower_positions[i, 0], follower_positions[i, 1], f"Follower {i+1}", ha='center', va='center', color='b', fontsize=8, fontweight='bold'))

ax.legend(loc='upper right')

#--------------------------------------------------------------
for k in range(num_followers):
    x=[]
    y=[]
    for i in range(num_iterations):
        x.append(Y[:,i].reshape([num_followers, 2])[k,0])
        y.append(Y[:,i].reshape([num_followers, 2])[k,1]) 
    plt.plot(x, y)
#--------------------------------------------------------------

# Function to update the plot in each animation frame
def update(frame):
    follower_positions=Y[:,frame].reshape([num_followers, 2])

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
ani = animation.FuncAnimation(fig, update, frames=num_iterations, interval=t_sum/num_iterations, blit=True)

# Save the animation as a gif
ani.save('multi_agent_containment.gif', writer='pillow')

# ----------------------------------------------------





# # Function to calculate the convex hull using scipy's ConvexHull
# def convex_hull(points):
#     hull = ConvexHull(points)
#     return hull

# # Function to calculate the centroid of a set of points using numpy
# def centroid(points):
#     x, y = points[:, 0], points[:, 1]
#     cx, cy = np.mean(x), np.mean(y)
#     return np.array([cx, cy])

# # Function to calculate the distance between two agents
# def distance_between_agents(agent1, agent2):
#     return np.linalg.norm(agent1 - agent2)

# # Function for leaders to communicate their positions to the respective followers
# def leader_communication():
#     for leader_id in range(num_leaders):
#         leader_position = leader_positions[leader_id]
#         follower_ids = leader_topology[leader_id]
#         for follower_id in follower_ids:
#             follower_positions[follower_id] = leader_position

# # Function to update the follower positions based on the convex hull
# def update_follower_positions():
#     global follower_positions
#     for follower_id in range(num_followers):
#         neighbor_indices = [x - 1 for x in follower_topology[follower_id]]  # Convert indices to 0-based
#         neighbors = [follower_positions[i] for i in neighbor_indices]

#         # Combine leader positions and neighbor positions to find the convex hull
#         all_positions = np.vstack((leader_positions, np.array(neighbors)))
#         hull = convex_hull(all_positions)
#         hull_points = all_positions[hull.vertices]

#         # Calculate the centroid of the convex hull
#         hull_centroid = centroid(hull_points)

#         # Calculate the direction vector towards the centroid
#         direction = hull_centroid - follower_positions[follower_id]

#         # Normalize the direction vector and calculate the new position
#         distance = np.linalg.norm(direction)
#         if distance > 0:
#             direction /= distance
#         movement = min(max_movement_followers, distance)
#         follower_positions[follower_id] += movement * direction

# # Visualization setup
# fig, ax = plt.subplots()
# ax.set_xlim(0, 10)
# ax.set_ylim(0, 10)
# leader_scatter = ax.scatter(leader_positions[:, 0], leader_positions[:, 1], c='r', marker='o', label='Leaders')
# follower_scatter = ax.scatter(follower_positions[:, 0], follower_positions[:, 1], c='b', marker='o', label='Followers')
# hull_line, = ax.plot([], [], c='r', linewidth=2)

# ax.legend(loc='upper right')

# # Function to update the plot in each animation frame
# def update(frame):
#     leader_communication()  # Leaders communicate their positions to followers
#     update_follower_positions()

#     leader_scatter.set_offsets(leader_positions)
#     follower_scatter.set_offsets(follower_positions)

#     # Calculate and plot the convex hull
#     all_positions = np.vstack((leader_positions, follower_positions))
#     hull = convex_hull(all_positions)
#     hull_line.set_xdata(all_positions[hull.vertices, 0])
#     hull_line.set_ydata(all_positions[hull.vertices, 1])

#     return leader_scatter, follower_scatter, hull_line

# # Create the animation
# animation.FuncAnimation(fig, update, frames=num_iterations, interval=200,repeat=False)


# # Visualization setup
# fig, ax = plt.subplots()
# ax.set_xlim(0, 10)
# ax.set_ylim(0, 10)
# leader_scatter = ax.scatter(leader_positions[:, 0], leader_positions[:, 1], c='r', marker='o', label='Leaders')
# follower_scatter = ax.scatter(follower_positions[:, 0], follower_positions[:, 1], c='b', marker='o', label='Followers')
# hull_line, = ax.plot([], [], c='r', linewidth=2)

# leader_labels = []
# follower_labels = []
# for i in range(num_leaders):
#     leader_labels.append(ax.text(leader_positions[i, 0], leader_positions[i, 1], f"Leader {i}", ha='center', va='center', color='r', fontsize=8, fontweight='bold'))
# for i in range(num_followers):
#     follower_labels.append(ax.text(follower_positions[i, 0], follower_positions[i, 1], f"Follower {i}", ha='center', va='center', color='b', fontsize=8, fontweight='bold'))

# ax.legend(loc='upper right')

# # Function to update the plot in each animation frame
# def update(frame):
#     leader_communication()  # Leaders communicate their positions to followers
#     update_follower_positions()

#     leader_scatter.set_offsets(leader_positions)
#     follower_scatter.set_offsets(follower_positions)

#     # Calculate the convex hull for leaders only
#     hull = convex_hull(leader_positions)
#     hull_line.set_xdata(leader_positions[hull.vertices, 0])
#     hull_line.set_ydata(leader_positions[hull.vertices, 1])

#     # Update labels
#     for i in range(num_leaders):
#         leader_labels[i].set_position(leader_positions[i])
#     for i in range(num_followers):
#         follower_labels[i].set_position(follower_positions[i])

#     return leader_scatter, follower_scatter, hull_line, *leader_labels, *follower_labels

# # Create the animation
# animation.FuncAnimation(fig, update, frames=num_iterations, interval=200, blit=True)
