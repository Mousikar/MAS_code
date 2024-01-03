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
    rospy.init_node("leadermove")
    rate = rospy.Rate(50)

    # Set ids of swarm robot based on Aruco marker
    # 10个机器人
    num_leader = 4
    num_follower = 6
    swarm_robot_id = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    # follower_topology = {0: [1, 3], 1: [0, 2], 2: [1], 3: [0, 4], 4: [3, 5], 5: [4]}
    # leader_topology = {0: [6], 1: [], 2: [7], 3: [8], 4: [], 5: [9]}

    # 这个拓扑有些拥挤
    # follower_topology = {0: [], 1: [0], 2: [1,3], 3: [], 4: [1], 5: [2,4]}
    # leader_topology = {0: [6,9], 1: [7], 2: [8], 3: [8,9], 4: [8], 5: []}

    follower_topology = {0: [], 1: [0], 2: [1,3], 3: [], 4: [1], 5: [0,2]}
    leader_topology = {0: [6,9], 1: [7], 2: [8], 3: [8,9], 4: [6], 5: []}

    # Initialize swarm robot
    swarm_robot = SwarmRobot(swarm_robot_id)
    r_star = np.array([[2, 2], 
                       [5, 2], 
                       [5, 5],
                       [2, 5]])

    # Convergence threshold
    t_sum = 6
    dT = 0.001
    iter = int(t_sum / dT)

    # 历史变量
    r_star_his = []
    dot_r_star_his = []
    theta_d = [0,0,0,0,0,0,0,0,0,0]
    theta_d1 = [0,0,0,0,0,0,0,0,0,0]
    kk = [0,0,0,0,0,0,0,0,0,0]
    ddot_hat_theta_d = [0,0,0,0,0,0,0,0,0,0]
    R =100
    dot_hat_theta_d = [0,0,0,0,0,0,0,0,0,0]
    hat_theta_d  = [0,0,0,0,0,0,0,0,0,0]

    # Velocity scale and threshold
    MAX_W = 1       # Maximum angle velocity (rad/s)
    MIN_W = 0.00000000000000000005    # Minimum angle velocity(rad/s)
    MAX_V = 0.3     # Maximum linear velocity(m/s)
    MIN_V = 0.00000000000001    # Minimum linear velocity(m/s)
    k_v = 0.01       # Scale of linear velocity
    k_w = 0.5       # Scale of angle velocity
    k_L = 0.1
    # 指定保存的文件名
    file_name = "followermove.txt"

    # Convergence sign
    is_conv = False

    # While loop
    v_lim=np.ones(swarm_robot.robot_num)

    # 将测试点保存到文本文件中
    with open(file_name, "w") as file:
        for k in range(iter):
            if k > iter/12:
                k_v = 0.05
            if k%100==0:
                print(k/float(iter))
            # print('----------------------------------------------')
            current_robot_pose = swarm_robot.get_robot_poses()
            points = [x[0:2] for x in current_robot_pose]
            speed = []
            for i in range(num_follower):
            #     # test_point = [x[0:2] for x in current_robot_pose]
            #     # is_inside = hull([test_point])
            #     # if is_inside[0]:
            #     #     is_conv = False

                neighbor_positions = []
                leader_positions = []
                neighbors = follower_topology.get(i, [])
                # print(neighbors)
                for item in neighbors:
                    neighbor_positions.append(current_robot_pose[item])
                # print(neighbor_positions)
                leaders = leader_topology.get(i, [])
                # print(leaders)
                for item in leaders:
                    leader_positions.append(current_robot_pose[item])
                # print(leader_positions)


                # 角度相减去 
                v_leader = np.array([0.01 * np.cos(0.6), 0.01 * np.sin(0.6), 0])
                velocity = v_leader - ([x * len(neighbors) for x in current_robot_pose[i]] - np.sum(neighbor_positions, axis=0)) 
                velocity = velocity - ([x * len(leaders) for x in current_robot_pose[i]] - np.sum(leader_positions, axis=0))
                # print(velocity)
                v=math.sqrt(math.pow(k_v*velocity[0],2)+math.pow(k_v*velocity[1],2))
                v = swarm_robot.check_vel(v, MAX_V, MIN_V)
                theta_d = math.atan2(velocity[1], velocity[0])
                w= k_w * (theta_d-current_robot_pose[i][2])
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                # v=0
                # w=0
                speed.append([v,w])
            #     if v<=0.02:
            #         k_v=0.05
            #         v_lim[i]=0
            #         # 要检查的多个点
            #         test_points = [x[0:2] for x in current_robot_pose]
            #         # 判断每个点是否在凸包内
            #         # all_inside = all(hull.points[hull.vertices].shape[0] == len(ConvexHull(points + [p]).vertices) for p in test_points)
            #         all_inside = all(all(v in hull.vertices for v in ConvexHull(np.vstack((points, [p]))).vertices) for p in test_points)
            #         if np.max(v_lim)==0 and all_inside:
            #             is_conv = True
            #             if all_inside:
            #                 print("All test points are inside the convex hull.")
            #     else:
            #         v_lim[i]=1
            #         k_v=0.005
                file.write("%.12f, %.12f, %.12f, %.12f, %.12f, " % (current_robot_pose[i][0], current_robot_pose[i][1], current_robot_pose[i][2], v, w))
            file.write("\n")

            for i in [6,7,8,9]:
                v=0.01
                w=0
                speed.append([v,w])
            # 移动机器人
            rate.sleep()
            swarm_robot.move_robots(speed)



    # Stop all robots
    swarm_robot.stop_robots()

    rospy.loginfo("Succeed!")

if __name__ == "__main__":
    main()
