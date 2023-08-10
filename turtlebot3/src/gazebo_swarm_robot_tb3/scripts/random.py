#! /usr/bin/env python2.7
# encoding: utf-8
import rospy
import tf
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from swarm_robot_control_new import SwarmRobot
import math
from scipy.spatial import ConvexHull

# SE(3)
def T(pose):
    T = np.zeros([3,3])
    T[0,0] = np.cos(pose[2])
    T[1,0] = np.sin(pose[2])
    T[0,1] = - np.sin(pose[2])
    T[1,1] = np.cos(pose[2])
    T[0,2] = pose[0]
    T[1,2] = pose[1]
    T[2,2] = 1.0
    return T
def T_inv(T):
    T_inv = T
    T_inv[:2,:2] = T[:2,:2].T
    T_inv[:2,2] = np.dot(-T[:2,:2].T,T[:2,2])
    return T_inv

def main():
    rospy.init_node("ramdon")
    rate = rospy.Rate(50)

    # Set ids of swarm robot based on Aruco marker
    # 六个机器人
    swarm_robot_id = [1, 2, 3, 4, 5, 6]
    follower_topology = {0: [1, 3], 1: [0, 2], 2: [1], 3: [0, 4], 4: [3, 5], 5: [4]}
    leader_topology = {0: [0], 1: [1], 2: [2], 3: [], 4: [], 5: []}
    # 两个机器人
    # swarm_robot_id = [1, 2]
    # follower_topology = {0: [1], 1: [0]}
    # leader_topology = {0: [], 1: []}

    # Initialize swarm robot
    swarm_robot = SwarmRobot(swarm_robot_id)
    leader_robot_pose = [[1, 1, -0.1], 
                    [1, 2, 0.1], 
                    [2, 1, -0.1],
                    [2, 2, 0.1]]
    points = [x[0:2] for x in leader_robot_pose]
    hull = ConvexHull(points)

    # Convergence threshold
    num_iterations = 1000
    t_sum = 10
    deltat = t_sum/num_iterations

    # Velocity scale and threshold
    MAX_W = 1       # Maximum angle velocity (rad/s)
    MIN_W = 0.05    # Minimum angle velocity(rad/s)
    MAX_V = 0.2     # Maximum linear velocity(m/s)
    MIN_V = 0.01    # Minimum linear velocity(m/s)
    k_v = 0.1       # Scale of linear velocity
    k_w = 2       # Scale of angle velocity

    # Mobile robot poses and for next poses
    cur_pose = np.zeros([len(swarm_robot_id),3])
    del_theta = np.zeros(len(swarm_robot_id))

    # Get swarm robot poses firstly
    current_robot_pose = swarm_robot.get_robot_poses()
    rate.sleep()
    # print(current_robot_pose)

    # Convergence sign
    is_conv = False

    # While loop
    # while not is_conv:
    for i in range(num_iterations):
        for i in range(swarm_robot.robot_num):
            print('----------------------------------------------')
            current_robot_pose = swarm_robot.get_robot_poses()
            # test_point = [x[0:2] for x in current_robot_pose]
            # is_inside = hull([test_point])
            # if is_inside[0]:
            #     is_conv = False

            neighbor_positions = []
            leader_positions = []
            neighbors = follower_topology.get(i, [])
            print(neighbors)
            for item in neighbors:
                neighbor_positions.append(current_robot_pose[item])
            print(neighbor_positions)
            leaders = leader_topology.get(i, [])
            print(leaders)
            for item in leaders:
                leader_positions.append(leader_robot_pose[item])
            print(leader_positions)

            # # 计算相对位置
            # tf_i0 = T(current_robot_pose[i])
            # tf_j0 = [T(item) for item in neighbor_positions]
            # tf_0i = T_inv(tf_i0)
            # tf_ji = [np.dot(tf_0i,item) for item in tf_j0]
            # tf_neighbor = [item[:,2] for item in tf_ji]
            # # velocity = - ([x * len(neighbors) for x in current_robot_pose[i]] - np.sum(tf_neighbor, axis=0))
            # velocity = np.sum(tf_neighbor, axis=0)
            # print(np.sum(tf_neighbor, axis=0))
            # # 领导者
            # if leader_positions != []:
            #     tf_j0 = [T(item) for item in leader_positions]
            #     tf_ji = [np.dot(tf_0i,item) for item in tf_j0]
            #     tf_leader = [item[:,2] for item in tf_ji]
            #     velocity = + np.sum(tf_leader, axis=0)
            #     print(np.sum(tf_leader, axis=0))

            # 之前的速度
            # use position get velocity
            print([x * len(leaders) for x in current_robot_pose[i]])
            print(np.sum(neighbor_positions, axis=0))
            velocity = - ([x * len(neighbors) for x in current_robot_pose[i]] - np.sum(neighbor_positions, axis=0)) 
            velocity = velocity - ([x * len(leaders) for x in current_robot_pose[i]] - np.sum(leader_positions, axis=0))
            print(velocity)
            v=k_v*math.sqrt(math.pow(velocity[0],2)+math.pow(velocity[1],2))
            w=k_w*math.atan2(velocity[1], velocity[0])

            # # 角度相减去 
            # velocity = - ([x * len(neighbors) for x in current_robot_pose[i]] - np.sum(neighbor_positions, axis=0)) 
            # velocity = velocity - ([x * len(leaders) for x in current_robot_pose[i]] - np.sum(leader_positions, axis=0))
            # print(velocity)
            # k = 0.2
            # v=math.sqrt(math.pow(k*velocity[0],2)+math.pow(k*velocity[1],2))
            # w=math.atan2(velocity[1], velocity[0])-current_robot_pose[i][2]

            # 移动单个机器人
            # rate.sleep()
            swarm_robot.move_robot(i, v, w)



    # Stop all robots
    swarm_robot.stop_robots()

    rospy.loginfo("Succeed!")

if __name__ == "__main__":
    main()
