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
    rospy.init_node("init")
    rate = rospy.Rate(1)

    # Set ids of swarm robot based on Aruco marker
    # 六个机器人
    swarm_robot_id = [1, 2, 3, 4, 5, 6]
    swarm_robot = SwarmRobot(swarm_robot_id)

    # 移动所有机器人
    for i in range(10):
        speed = []
        for i in range(swarm_robot.robot_num):
            v=5*np.random.rand(1,2)
            speed.append([v[0][0],0.1*v[0][1]])
        swarm_robot.move_robots(speed)
        rate.sleep()
        # Stop all robots
        swarm_robot.stop_robots()

    rospy.loginfo("Succeed!")

if __name__ == "__main__":
    main()