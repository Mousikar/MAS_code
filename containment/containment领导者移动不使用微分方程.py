import numpy as np
import matplotlib.pyplot as plt

class Agent:
    def __init__(self, position):
        self.position = np.array(position)
        self.velocity = np.zeros(2)  # 初始化速度为零

    def move(self):
        # 根据当前速度更新位置
        self.position += self.velocity

class FollowerAgent(Agent):
    def __init__(self, position, neighbors, leader_indices):
        super().__init__(position)
        self.neighbors = neighbors
        self.leader_indices = leader_indices

    def update_velocity(self, a, b, agents):
        neighbor_positions = [agents[i].position for i in self.neighbors]
        leader_positions = [agents[i].position for i in self.leader_indices]
        self.velocity = a * (np.sum(neighbor_positions, axis=0) - len(self.neighbors) * self.position)
        self.velocity -= b * (np.sum(leader_positions, axis=0) - len(self.leader_indices) * self.position)
        return leader_positions

class LeaderAgent(Agent):
    def __init__(self, position):
        super().__init__(position)

def main():
    num_followers = 6
    num_leaders = 3
    a = 0.1
    b = 0.05

    follower_positions = 10 * np.random.rand(num_followers, 2)
    leader_positions = 0.5 * np.ones([num_leaders, 2]) + 4 * np.random.rand(num_leaders, 2)

    follower_topology = {0: [1, 3], 1: [0, 2], 2: [1], 3: [0, 4], 4: [3, 5], 5: [4]}
    leader_topology = {0: [0], 1: [1], 2: [2], 3: [], 4: [], 5: []}

    agents = []

    for i in range(num_followers):
        neighbors = follower_topology.get(i, [])
        print(neighbors)
        leader_indices = [leader_idx for leader_idx, leaders in leader_topology.items() if i in leaders]
        follower_agent = FollowerAgent(follower_positions[i], neighbors, leader_indices)
        agents.append(follower_agent)

    for i in range(num_leaders):
        leader_agent = LeaderAgent(leader_positions[i])
        agents.append(leader_agent)



    num_iterations = 1

    follower_positions_history = []

    for _ in range(num_iterations):
        for agent in agents:
            if isinstance(agent, FollowerAgent):
                neighbor_positions = agent.update_velocity(a, b, agents)
                print(neighbor_positions)
            agent.move()
        
        current_follower_positions = np.array([agent.position for agent in agents[:num_followers]])
        follower_positions_history.append(current_follower_positions)

#     # 打印最终的智能体位置
#     for i, agent in enumerate(agents):
#         print(f"Agent {i+1}: {agent.position}")
#     # print(follower_positions_history)

#     # 绘制智能体的运动轨迹并保存为图片
#     plt.figure(figsize=(8, 6))
#     for i,positions in enumerate(follower_positions_history):
#         plt.plot(positions[:, 0], positions[:, 1])
#         print(positions)
#     plt.scatter(leader_positions[:, 0], leader_positions[:, 1], c='r', marker='o', label='Leaders')
#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.title('Agent Trajectories')
#     plt.legend()
#     plt.grid()
#     plt.savefig('agent_trajectories.png')
#     plt.show()

if __name__ == "__main__":
    main()
