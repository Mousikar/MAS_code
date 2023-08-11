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

        L1=laplacian_matrix+A0

        # Create L2
        L2 = np.zeros((num_followers, num_leaders))
        for i in range(num_followers):
            leaders = leader_topology[i]
            for leader in leaders:
                L2[i, leader] = -1

    # Initialize swarm robot
    swarm_robot = SwarmRobot(swarm_robot_id)
    leader_robot_pose = np.array([[-1, 2, -0.1], 
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
    MAX_W = 1       # Maximum angle velocity (rad/s)
    MIN_W = 0.00000000000000000005    # Minimum angle velocity(rad/s)
    MAX_V = 0.2     # Maximum linear velocity(m/s)
    MIN_V = 0.00000000000001    # Minimum linear velocity(m/s)
    # 指定保存的文件名
    file_name = "containment_l.txt"

    # 将测试点保存到文本文件中
    with open(file_name, "w") as file:

        # While loop
        v_lim=np.ones(swarm_robot.robot_num)
        k = 0.005
        # for i in range(200):
        iter_pos = 0
        while not is_conv:
            print('----------------------------------------------')
            current_robot_pose = swarm_robot.get_robot_poses()
            cur_pos = np.array(current_robot_pose)
            dotx = - np.dot(L1,cur_pos[:,0]) - np.dot(L2,leader_robot_pose[:,0])
            doty = - np.dot(L1,cur_pos[:,1]) - np.dot(L2,leader_robot_pose[:,1])
            # print(dotx,doty)
            # 移动所有机器人
            speed = []
            for i in range(swarm_robot.robot_num):
                v=math.sqrt(math.pow(k*dotx[i],2)+math.pow(k*doty[i],2))
                v = swarm_robot.check_vel(v, MAX_V, MIN_V)
                w=math.atan2(doty[i], dotx[i]) - current_robot_pose[i][2]
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                speed.append([v,w])
                # w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                if v<=0.02:
                    k=0.05
                    v_lim[i]=0
                    # 要检查的多个点
                    test_points = cur_pos[:, :2]
                    # 判断每个点是否在凸包内
                    # all_inside = all(hull.points[hull.vertices].shape[0] == len(ConvexHull(points + [p]).vertices) for p in test_points)
                    all_inside = all(all(v in hull.vertices for v in ConvexHull(np.vstack((points, [p]))).vertices) for p in test_points)
                    if np.max(v_lim)==0 and all_inside:
                        is_conv = True
                        if all_inside:
                            print("All test points are inside the convex hull.")
                else:
                    v_lim[i]=1
                    k=0.005
                file.write("%.12f, %.12f, %.12f, %.12f, %.12f, " % (current_robot_pose[i][0], current_robot_pose[i][1], current_robot_pose[i][2], v, w))
            file.write("\n")

            rate.sleep()
            swarm_robot.move_robots(speed)
            iter_pos += 1

        # 在运动400次
        for i in range(400):
            print('----------------------------------------------')
            current_robot_pose = swarm_robot.get_robot_poses()
            cur_pos = np.array(current_robot_pose)
            dotx = - np.dot(L1,cur_pos[:,0]) - np.dot(L2,leader_robot_pose[:,0])
            doty = - np.dot(L1,cur_pos[:,1]) - np.dot(L2,leader_robot_pose[:,1])
            # print(dotx,doty)
            # 移动所有机器人
            speed = []
            for i in range(swarm_robot.robot_num):
                v=math.sqrt(math.pow(k*dotx[i],2)+math.pow(k*doty[i],2))
                v = swarm_robot.check_vel(v, MAX_V, MIN_V)
                w=math.atan2(doty[i], dotx[i]) - current_robot_pose[i][2]
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                speed.append([v,w])
                if v<=0.02:
                    k=0.05
                    v_lim[i]=0
                    # 要检查的多个点
                    test_points = cur_pos[:, :2]
                    # 判断每个点是否在凸包内
                    # all_inside = all(hull.points[hull.vertices].shape[0] == len(ConvexHull(points + [p]).vertices) for p in test_points)
                    all_inside = all(all(v in hull.vertices for v in ConvexHull(np.vstack((points, [p]))).vertices) for p in test_points)
                    if np.max(v_lim)==0 and all_inside:
                        is_conv = True
                        if all_inside:
                            print("All test points are inside the convex hull.")
                else:
                    v_lim[i]=1
                    k=0.005
                file.write("%.12f, %.12f, %.12f, %.12f, %.12f, " % (current_robot_pose[i][0], current_robot_pose[i][1], current_robot_pose[i][2], v, w))
            file.write("\n")
            
            rate.sleep()
            swarm_robot.move_robots(speed)

    # Stop all robots
    swarm_robot.stop_robots()
    print("位置迭代：")
    print(iter_pos)

    rospy.loginfo("Succeed!")

if __name__ == "__main__":
    main()