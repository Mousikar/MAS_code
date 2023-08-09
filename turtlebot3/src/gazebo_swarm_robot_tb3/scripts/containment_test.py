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
    rospy.init_node("containment_robot_control_formation")
    rate = rospy.Rate(0.3)

    # Set ids of swarm robot based on Aruco marker
    swarm_robot_id = [1, 2, 3, 4, 5, 6]
    follower_topology = {0: [1, 3], 1: [0, 2], 2: [1], 3: [0, 4], 4: [3, 5], 5: [4]}
    leader_topology = {0: [0], 1: [1], 2: [2], 3: [3], 4: [], 5: []}
    # swarm_robot_id = [1, 2]
    # follower_topology = {0: [], 1: []}
    # leader_topology = {0: [0], 1: []}

    # Initialize swarm robot
    swarm_robot = SwarmRobot(swarm_robot_id)
    leader_robot_pose = [[1, 1, -0.1], 
                    [1, 2, 0.1], 
                    [2, 1, -0.1],
                    [2, 2, 0.1]]
    points = [x[0:2] for x in leader_robot_pose]
    hull = ConvexHull(points)

    # Convergence threshold
    num_iterations = 100
    t_sum = 10
    deltat = t_sum/num_iterations

    # Velocity scale and threshold
    MAX_W = 1       # Maximum angle velocity (rad/s)
    MIN_W = 0.05    # Minimum angle velocity(rad/s)
    MAX_V = 0.2     # Maximum linear velocity(m/s)
    MIN_V = 0.01    # Minimum linear velocity(m/s)
    k_v = 0.5       # Scale of linear velocity
    k_w = 0.1       # Scale of angle velocity

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
    while not is_conv:
    # for i in range(num_iterations):
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()
            # test_point = [x[0:2] for x in current_robot_pose]
            # is_inside = hull([test_point])
            # if is_inside[0]:
            #     is_conv = False

            neighbor_positions = []
            leader_positions = []
            neighbors = follower_topology.get(i, [])
            # print(neighbors)
            for item in neighbors:
                # print(current_robot_pose[item])
                neighbor_positions.append(current_robot_pose[item])
            leaders = leader_topology.get(i, [])
            # print(leaders)
            for item in leaders:
            #     print(leader_robot_pose[item])
                leader_positions.append(leader_robot_pose[item])

            # use position get velocity
            # print([x * len(leaders) for x in current_robot_pose[i]])
            # print(np.sum(neighbor_positions, axis=0))
            velocity = - ([x * len(neighbors) for x in current_robot_pose[i]] - np.sum(neighbor_positions, axis=0)) 
            # velocity = velocity - ([x * len(leaders) for x in current_robot_pose[i]] - np.sum(leader_positions, axis=0))
            # print(velocity)
            v=k_v*math.sqrt(math.pow(velocity[0],2)+math.pow(velocity[1],2))
            w=k_w*math.atan2(velocity[1], velocity[0])
            # 移动单个机器人
            swarm_robot.move_robot(i, v, w)



    # Stop all robots
    swarm_robot.stop_robots()

    rospy.loginfo("Succeed!")

if __name__ == "__main__":
    main()
