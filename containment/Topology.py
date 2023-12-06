import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

# Define the number of followers
num_followers = 12
num_followers = 10

# Define the communication topology for followers
# follower_topology = {0: [6], 
#                      1: [7], 
#                      2: [8], 
#                      3: [9], 
#                      4: [10], 
#                      5: [11],
#                      6: [7, 9],
#                      7: [6, 8],
#                      8: [7],
#                      9: [6, 10],
#                      10:[9,11],
#                      11:[10]}
follower_topology = {0: [1], 
                     1: [2,4], 
                     2: [5], 
                     3: [2], 
                     4: [5], 
                     5: [],
                     6: [0],
                     7: [1, 3],
                     8: [2,4],
                     9: [0, 3]}
# Create the communication topology matrix
communication_topology = np.zeros((num_followers, num_followers))
for i in range(num_followers):
    neighbors = follower_topology[i]
    for neighbor in neighbors:
        communication_topology[neighbor, i] = 1

# Compute the Laplacian matrix
degree_matrix = np.diag(np.sum(communication_topology, axis=1))
laplacian_matrix = degree_matrix - communication_topology

print("Communication Topology Matrix:")
print(communication_topology)

print("\nLaplacian Matrix:")
print(laplacian_matrix)

# 计算Laplacian矩阵的特征值
eigenvalues = np.linalg.eigvals(laplacian_matrix)

# 找出最大特征值
max_eigenvalue = np.max(eigenvalues)
print("Laplacian矩阵的最大特征值为:", max_eigenvalue)
print("时延最大值为:", np.pi / 2 /max_eigenvalue)

# Create a graph for visualization
G = nx.Graph()
G.add_nodes_from(range(num_followers))

for i in range(num_followers):
    neighbors = follower_topology[i]
    for neighbor in neighbors:
        G.add_edge(i, neighbor)

# Manually set the positions of nodes
pos = {
    0: (0, 3),
    1: (1, 3),
    2: (2, 3),
    3: (0, 0),
    4: (1, 0),
    5: (2, 0),
    6: (0, 2),
    7: (1, 2),
    8: (2, 2),
    9: (0, 1),
    10: (1, 1),
    11: (2, 1)
}

# # Visualize the network topology
# pos = nx.spring_layout(G, seed=42)
labels = {i: f"{i}" for i in range(num_followers)}
leader_nodes = []
follower_nodes = []
for node in G.nodes:
    if node < 6:  # Assuming first 6 nodes are leaders
        leader_nodes.append(node)
    else:
        follower_nodes.append(node)

nx.draw_networkx_nodes(G, pos, nodelist=leader_nodes, node_color='y', node_size=1000)
nx.draw_networkx_nodes(G, pos, nodelist=follower_nodes, node_color='g', node_size=1000)
nx.draw_networkx_edges(G, pos, width=1.0, alpha=0.5)
nx.draw_networkx_labels(G, pos, labels, font_size=10, font_weight='bold')
plt.title("Follower Network Topology")
plt.axis('off')
plt.show()
