#! /usr/bin/env python2.7
# encoding: utf-8
import rospy
import tf
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from swarm_robot_control_new import SwarmRobot

def main():
    rospy.init_node("swarm_robot_control_formation")

    # Set ids of swarm robot based on Aruco marker
    swarm_robot_id = [1, 2, 3, 4, 5]

    # Initialize swarm robot
    swarm_robot = SwarmRobot(swarm_robot_id)

    # Set Laplacian Matrix
    lap = np.array([[4, -1, -1, -1, -1],
                    [-1, 4, -1, -1, -1],
                    [-1, -1, 4, -1, -1],
                    [-1, -1, -1, 4, -1],
                    [-1, -1, -1, -1, 4]])

    # Convergence threshold
    conv_th = 0.05  # Threshold of angle, in rad

    # Velocity scale and threshold
    MAX_W = 1       # Maximum angle velocity (rad/s)
    MIN_W = 0.05    # Minimum angle velocity(rad/s)
    MAX_V = 0.2     # Maximum linear velocity(m/s)
    MIN_V = 0.01    # Minimum linear velocity(m/s)
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
        del_theta = -np.dot(lap, cur_theta)
        is_conv = all(np.abs(del_theta) <= conv_th)

        # Swarm robot move
        for i in range(len(swarm_robot_id)):
            w = del_theta[i] * k_w
            w = swarm_robot.check_vel(w, MAX_W, MIN_W)
            swarm_robot.move_robot(i, 0.0, w)

        # Time sleep for robot move
        rospy.sleep(0.05)

        # Get swarm robot poses
        current_robot_pose = swarm_robot.get_robot_poses()

        for i in range(len(swarm_robot_id)):
            cur_theta[i] = current_robot_pose[i][2]

    # Stop all robots
    swarm_robot.stop_robots()

    rospy.loginfo("Succeed!")

if __name__ == "__main__":
    main()
