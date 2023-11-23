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
    follower_topology = {0: [1, 3], 1: [0, 2], 2: [1], 3: [0, 4], 4: [3, 5], 5: [4]}
    leader_topology = {0: [6], 1: [], 2: [7], 3: [8], 4: [], 5: [9]}

    # Initialize swarm robot
    swarm_robot = SwarmRobot(swarm_robot_id)
    r_star = np.array([[2, 2], 
                       [3, 2], 
                       [3, 3],
                       [2, 3]])

    # Convergence threshold
    t_sum = 60
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
    # leader期望轨迹，在20——25秒有个变化，队形缩小，在40转弯
    for k in range(iter):
        if k <= iter/3:
            dot_r_star = np.array([ [0.1, 0.15], 
                                    [0.1, 0.15],
                                    [0.1, 0.15], 
                                    [0.1, 0.15]])
            r_star = r_star + dot_r_star * dT
        if k > iter/3  & k<= iter/3+int(5/dT):
            dot_r_star = np.array([ [0.1, 0.15], 
                                    [0.1, 0.15],
                                    [0.1, 0.15], 
                                    [0.1, 0.15]]) + np.array([
                [-0.1, -0.1],
                [0.1, -0.1],
                [0.1, 0.1],
                [-0.1,0.1]
            ])
            r_star = r_star + dot_r_star * dT
        if k > iter/3+int(5/dT) & k<= iter/3*2:
            dot_r_star = np.array([ [0.1, 0.15], 
                                    [0.1, 0.15],
                                    [0.1, 0.15], 
                                    [0.1, 0.15]])
            r_star = r_star + dot_r_star * dT
        if k > iter/3*2:
            dot_r_star = np.array([ [0.1, 0.15], 
                                    [0.1, 0.15],
                                    [0.1, 0.15], 
                                    [0.1, 0.15]])
            r_star = r_star + dot_r_star * dT
        r_star_his.append(r_star)
        dot_r_star_his.append(dot_r_star)

    # Velocity scale and threshold
    MAX_W = 1       # Maximum angle velocity (rad/s)
    MIN_W = 0.00000000000000000005    # Minimum angle velocity(rad/s)
    MAX_V = 0.3     # Maximum linear velocity(m/s)
    MIN_V = 0.00000000000001    # Minimum linear velocity(m/s)
    k_v = 0.005       # Scale of linear velocity
    k_w = 2       # Scale of angle velocity
    k_L = 0.5
    # 指定保存的文件名
    file_name = "leadermove.txt"

    # Convergence sign
    is_conv = False

    # While loop
    v_lim=np.ones(swarm_robot.robot_num)

    # 将测试点保存到文本文件中
    with open(file_name, "w") as file:
        for k in range(iter):
            # print('----------------------------------------------')
            current_robot_pose = swarm_robot.get_robot_poses()
            points = [x[0:2] for x in current_robot_pose]
            speed = []
            for i in range(num_follower):
            #     # test_point = [x[0:2] for x in current_robot_pose]
            #     # is_inside = hull([test_point])
            #     # if is_inside[0]:
            #     #     is_conv = False

            #     neighbor_positions = []
            #     leader_positions = []
            #     neighbors = follower_topology.get(i, [])
            #     print(neighbors)
            #     for item in neighbors:
            #         neighbor_positions.append(current_robot_pose[item])
            #     print(neighbor_positions)
            #     leaders = leader_topology.get(i, [])
            #     print(leaders)
            #     for item in leaders:
            #         leader_positions.append(leader_robot_refpose[item])
            #     print(leader_positions)


            #     # 角度相减去 
            #     velocity = - ([x * len(neighbors) for x in current_robot_pose[i]] - np.sum(neighbor_positions, axis=0)) 
            #     velocity = velocity - ([x * len(leaders) for x in current_robot_pose[i]] - np.sum(leader_positions, axis=0))
            #     print(velocity)
            #     v=math.sqrt(math.pow(k_v*velocity[0],2)+math.pow(k_v*velocity[1],2))
            #     v = swarm_robot.check_vel(v, MAX_V, MIN_V)
            #     w=math.atan2(velocity[1], velocity[0])-current_robot_pose[i][2]
            #     w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                v=0
                w=0
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
            #     file.write("%.12f, %.12f, %.12f, %.12f, %.12f, " % (current_robot_pose[i][0], current_robot_pose[i][1], current_robot_pose[i][2], v, w))
            # file.write("\n")

            for i in [6,7,8,9]:
                u_r = dot_r_star_his[k][i-6] - k_L * ( points[i] - r_star_his[k][i-6] )

                theta_d[i] = math.atan2(u_r[1], u_r[0])
                # -------------------------------保证theta_d在0和2 * pi之间---------------------------------------
                if k == 0:
                    theta_d1[i] = theta_d[i]
                    kk[i] = 0
                delta = - 0.7 * np.pi ** 2
                if theta_d[i] * theta_d1[i] < delta:
                    if theta_d[i] < 0:
                        kk[i] = kk[i] + 1
                    else:
                        kk[i] = kk[i] - 1
                theta_d1[i] = theta_d[i]
                theta_d[i] = theta_d[i] + 2 * np.pi * kk[i]
                # -------------------------------保证theta_d在0和2 * pi之间---------------------------------------

                v = math.sqrt(u_r[0]**2 + u_r[1]**2)

                ddot_hat_theta_d[i] = - R**2 * (hat_theta_d[i] - theta_d[i]) - 2 * R * dot_hat_theta_d[i]        # 线性二阶微分器
                dot_hat_theta_d[i] = dot_hat_theta_d[i] + ddot_hat_theta_d[i] * dT
                hat_theta_d[i] = hat_theta_d[i] + dot_hat_theta_d[i] * dT
                
                w = dot_hat_theta_d[i] + k_w * (theta_d[i] - current_robot_pose[i][2])      # 暂时不加上饱和函数
                      
                # v=math.sqrt(math.pow(k_L*u_r[0],2)+math.pow(k_L*u_r[1],2))
                v = swarm_robot.check_vel(v, MAX_V, MIN_V)
                # w=math.atan2(u_r[1], u_r[0])-current_robot_pose[i][2]
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                speed.append([v,w])
            # 移动机器人
            rate.sleep()
            swarm_robot.move_robots(speed)



    # Stop all robots
    swarm_robot.stop_robots()

    rospy.loginfo("Succeed!")

if __name__ == "__main__":
    main()
