#! /usr/bin/env python2.7
# -*- coding: utf-8 -*-
import rospy
import tf
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from swarm_robot_control_new import SwarmRobot
import math
from scipy.spatial import ConvexHull

def main():
    rospy.init_node("containment_L")
    rate = rospy.Rate(50)

    # Set ids of swarm robot based on Aruco marker
    # 六个机器人
    num_followers = 6
    num_leaders = 6
    swarm_robot_id = [1, 2, 3, 4, 5, 6]
    follower_topology = {0: [1, 3], 1: [0, 2], 2: [1], 3: [0, 4], 4: [3, 5], 5: [4]}
    leader_topology = {0: [0], 1: [1], 2: [2], 3: [3], 4: [4], 5: [5]}
    # 强连通
    # follower_topology = {0: [1, 3], 1: [0, 2], 2: [1, 5], 3: [0, 4], 4: [3, 5], 5: [2, 4]}
    # Create the communication topology matrix
    communication_topology = np.zeros((num_followers, num_followers))
    for i in range(num_followers):
        neighbors = follower_topology[i]
        for neighbor in neighbors:
            communication_topology[i, neighbor] = 1

        # Compute the Laplacian matrix
        degree_matrix = np.diag(np.sum(communication_topology, axis=1))
        laplacian_matrix = degree_matrix - communication_topology

        # Create A0
        A0 = np.zeros((num_followers, num_followers))
        for i in range(num_followers):
            leaders = leader_topology[i]
            A0[i, i] = len(leaders)

        L1=laplacian_matrix

        # Create L2
        L2 = np.zeros((num_followers, num_leaders))
        for i in range(num_followers):
            leaders = leader_topology[i]
            for leader in leaders:
                L2[i, leader] = -1

    # Initialize swarm robot
    swarm_robot = SwarmRobot(swarm_robot_id)
    leader_robot_pose = np.array([[-1, 1, -0.1], 
                    [-1, -1, 0.1], 
                    [1, -1, -0.1],
                    [1, 1, 0.1],
                    [2, 1, 0.1],
                    [2, 2, 0.1]])
    leader_robot_pose = 0 * np.ones([6,3])+np.array([[0, 2, -0.1], 
                    [1, 2, 0.1], 
                    [2, 2, -0.1],
                    [0, 0, 0.1],
                    [1, 0, 0.1],
                    [2, 0, 0.1]])
    # 取出前两列作为凸包顶点
    points = leader_robot_pose[:, :2]

    # 创建 ConvexHull 对象
    hull = ConvexHull(points)

    # Convergence sign
    is_conv = False

    # 指定保存的文件名
    file_name = "leader_follower_points.txt"

    # 将测试点保存到文本文件中
    with open(file_name, "w") as file:
        # While loop
        v_lim=np.ones(swarm_robot.robot_num)
        k = 0.008 * np.ones(swarm_robot.robot_num)
        MAX_W = 1       # Maximum angle velocity (rad/s)
        MIN_W = 0.00000000000000000005    # Minimum angle velocity(rad/s)
        MAX_V = 0.2     # Maximum linear velocity(m/s)
        MIN_V = 0.00000000000001    # Minimum linear velocity(m/s)
        # for i in range(200):
        iter_pos = 0
        while not is_conv:
            print('----------------------------------------------')
            current_robot_pose = swarm_robot.get_robot_poses()
            cur_pos = np.array(current_robot_pose)
            dotx = np.dot(L1+A0,leader_robot_pose[:,0]-cur_pos[:,0])
            doty = np.dot(L1+A0,leader_robot_pose[:,1]-cur_pos[:,1])
            print("sum:",np.sum(np.abs(leader_robot_pose[:,:2]-cur_pos[:,:2])))
            # 移动所有机器人
            speed = []
            for i in range(swarm_robot.robot_num):
                print(k[i])
                v=math.sqrt(math.pow(k[i] * dotx[i],2)+math.pow(k[i] * doty[i],2))
                v = swarm_robot.check_vel(v, MAX_V, MIN_V)
                w = math.atan2(doty[i], dotx[i]) - current_robot_pose[i][2]
                # w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                for item in follower_topology.get(i, []):
                    # 计算两个向量的夹角（弧度）
                    angle_radians = np.arccos(np.dot(cur_pos[item,:2]-cur_pos[i,:2], cur_pos[i,:2]) / (np.linalg.norm(cur_pos[item,:2]-cur_pos[i,:2]) * np.linalg.norm(cur_pos[i,:2])))
                    # print('',np.linalg.norm(cur_pos[item,:2]-cur_pos[i,:2]))
                    # print(np.linalg.norm(cur_pos[item,:2]-cur_pos[i,:2]) < 0.21 and np.abs(angle_radians)<1.57)
                    if np.linalg.norm(cur_pos[item,:2]-cur_pos[i,:2]) < 0.21 and np.abs(angle_radians)<1.57:
                        v= MAX_V * np.random.rand(1)
                        w= MAX_W * np.random.rand(1)
                if v<=0.02:
                    k[i]=0.08
                    v_lim[i]=0
                    if np.max(v_lim)==0 and np.sum(np.abs(leader_robot_pose[:,:2]-cur_pos[:,:2]))<0.4:
                        is_conv = True
                    if v<=0.002:
                        k[i]=0.8
                else:
                    v_lim[i]=1
                    k[i] = 0.005 * np.sum(np.abs(leader_robot_pose[i,:2]-cur_pos[i,:2]))
                    # k[i]=0.008
                speed.append([v,w])
                file.write("%.12f, %.12f, %.12f, %.12f, %.12f, " % (current_robot_pose[i][0], current_robot_pose[i][1], current_robot_pose[i][2], v, w))
            file.write("\n")

            rate.sleep()
            swarm_robot.move_robots(speed)
            iter_pos += 1

        file.write("\n")

        # 角度一致
        iter_ang = 0
        # Convergence threshold
        conv_th = 0.1  # Threshold of angle, in rad

        # threshold
        k_w = 0.1       # Scale of angle velocity
        k_v = 0.1       # Scale of linear velocity

        # Mobile robot poses and for next poses
        cur_theta = np.zeros(len(swarm_robot_id))
        del_theta = np.zeros(len(swarm_robot_id))

        # Get swarm robot poses firstly
        current_robot_pose = swarm_robot.get_robot_poses()

        for i in range(len(swarm_robot_id)):
            cur_theta[i] = current_robot_pose[i][2]

        # Convergence sign
        is_conv = False

        # While loop
        while not is_conv:

            # Judge whether reached
            del_theta = -np.dot(L1, cur_theta)
            is_conv = all(np.abs(del_theta) <= conv_th)

            # Swarm robot move
            w_his = []
            for i in range(len(swarm_robot_id)):
                w = del_theta[i] * k_w
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                swarm_robot.move_robot(i, 0.0, w)
                w_his.append(w)


            # Time sleep for robot move
            rospy.sleep(0.01)

            # Get swarm robot poses
            current_robot_pose = swarm_robot.get_robot_poses()

            for i in range(len(swarm_robot_id)):
                cur_theta[i] = current_robot_pose[i][2]
                # file.write(f"{current_robot_pose[i][0]}, {current_robot_pose[i][1]}, {current_robot_pose[i][2]}, 0.0, {w_his[i]}, ")
                file.write("%.12f, %.12f, %.12f, 0.0, %.12f, " % (current_robot_pose[i][0], current_robot_pose[i][1], current_robot_pose[i][2], w_his[i]))
            file.write("\n")
            iter_ang += 1


    # Stop all robots
    swarm_robot.stop_robots()
    print("位置迭代：")
    print(iter_pos)
    print("角度迭代：")
    print(iter_ang)
    rospy.loginfo("Succeed!")

if __name__ == "__main__":
    main()