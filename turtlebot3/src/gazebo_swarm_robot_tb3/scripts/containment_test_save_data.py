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
    rospy.init_node("containment_robot_control_formation_save_data")
    rate = rospy.Rate(50)

    # Set ids of swarm robot based on Aruco marker
    # 六个机器人
    swarm_robot_id = [1, 2, 3, 4, 5, 6]
    follower_topology = {0: [1, 3], 1: [0, 2], 2: [1], 3: [0, 4], 4: [3, 5], 5: [4]}
    leader_topology = {0: [0], 1: [1], 2: [2], 3: [3], 4: [4], 5: [5]}
    # 两个机器人
    # swarm_robot_id = [1, 2]
    # follower_topology = {0: [1], 1: [0]}
    # leader_topology = {0: [], 1: []}
    # 三个机器人
    # swarm_robot_id = [1, 2, 3]
    # follower_topology = {0: [1, 2], 1: [0,2], 2: [0,1]}
    # leader_topology = {0: [], 1: [], 2: []} # 一致性
    # leader_topology = {0: [1], 1: [2], 2: [3]} # 合围
    # 4个机器人
    # swarm_robot_id = [1, 2, 3, 4]
    # follower_topology = {0: [1, 2], 1: [0, 3], 2: [0, 3], 3: [1, 2]}
    # # leader_topology = {0: [], 1: [], 2: [], 3: []} # 一致性
    # leader_topology = {0: [0], 1: [1], 2: [2], 3: [3]} # 合围

    # Initialize swarm robot
    swarm_robot = SwarmRobot(swarm_robot_id)
    leader_robot_pose = [[0, 2, -0.1], 
                    [1, 2, 0.1], 
                    [2, 2, -0.1],
                    [0, 0, 0.1],
                    [1, 0, 0.1],
                    [2, 0, 0.1]]
    points = [x[0:2] for x in leader_robot_pose]
    hull = ConvexHull(points)

    # Velocity scale and threshold
    MAX_W = 1       # Maximum angle velocity (rad/s)
    MIN_W = 0.00000000000000000005    # Minimum angle velocity(rad/s)
    MAX_V = 0.2     # Maximum linear velocity(m/s)
    MIN_V = 0.00000000000001    # Minimum linear velocity(m/s)
    k_v = 0.005       # Scale of linear velocity
    k_w = 2       # Scale of angle velocity

    # 指定保存的文件名
    file_name = "containment_test.txt"

    # # Get swarm robot poses firstly
    # current_robot_pose = swarm_robot.get_robot_poses()
    # rate.sleep()
    # # print(current_robot_pose)

    # Convergence sign
    is_conv = False

    # While loop
    v_lim=np.ones(swarm_robot.robot_num)

    # 将测试点保存到文本文件中
    with open(file_name, "w") as file:
        while not is_conv:
        # for i in range(num_iterations):
            speed = []
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

                # 之前的速度
                # use position get velocity
                # print([x * len(leaders) for x in current_robot_pose[i]])
                # print(np.sum(neighbor_positions, axis=0))
                # velocity = - ([x * len(neighbors) for x in current_robot_pose[i]] - np.sum(neighbor_positions, axis=0)) 
                # velocity = velocity - ([x * len(leaders) for x in current_robot_pose[i]] - np.sum(leader_positions, axis=0))
                # print(velocity)
                # v=k_v*math.sqrt(math.pow(velocity[0],2)+math.pow(velocity[1],2))
                # w=k_w*math.atan2(velocity[1], velocity[0])

                # 角度相减去 
                velocity = - ([x * len(neighbors) for x in current_robot_pose[i]] - np.sum(neighbor_positions, axis=0)) 
                velocity = velocity - ([x * len(leaders) for x in current_robot_pose[i]] - np.sum(leader_positions, axis=0))
                print(velocity)
                v=math.sqrt(math.pow(k_v*velocity[0],2)+math.pow(k_v*velocity[1],2))
                v = swarm_robot.check_vel(v, MAX_V, MIN_V)
                w=math.atan2(velocity[1], velocity[0])-current_robot_pose[i][2]
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                speed.append([v,w])
                if v<=0.02:
                    k_v=0.05
                    v_lim[i]=0
                    # 要检查的多个点
                    test_points = [x[0:2] for x in current_robot_pose]
                    # 判断每个点是否在凸包内
                    # all_inside = all(hull.points[hull.vertices].shape[0] == len(ConvexHull(points + [p]).vertices) for p in test_points)
                    all_inside = all(all(v in hull.vertices for v in ConvexHull(np.vstack((points, [p]))).vertices) for p in test_points)
                    if np.max(v_lim)==0 and all_inside:
                        is_conv = True
                        if all_inside:
                            print("All test points are inside the convex hull.")
                else:
                    v_lim[i]=1
                    k_v=0.005
                file.write("%.12f, %.12f, %.12f, %.12f, %.12f, " % (current_robot_pose[i][0], current_robot_pose[i][1], current_robot_pose[i][2], v, w))
            file.write("\n")

            # 移动机器人
            rate.sleep()
            swarm_robot.move_robots(speed)



    # Stop all robots
    swarm_robot.stop_robots()

    rospy.loginfo("Succeed!")

if __name__ == "__main__":
    main()
