#! /usr/bin/env python
# encoding: utf-8
import os
import sys
import rospy
path = os.path.abspath(".")
sys.path.insert(0,path + "/src/gazebo_swarm_robot_tb3/scripts")
import swarm_robot_control_new
import math
import numpy as np

# 初始化节点
rospy.init_node("test")
rate = rospy.Rate(0.3)
index = [1,2,3,4,5]
# 建立对象
SwarmRobot = swarm_robot_control_new.SwarmRobot(index)
# # 基本信息
# print("swarm_robot_id = ",SwarmRobot.swarm_robot_id)# 机器人节点V
# print("robot_num = ",SwarmRobot.robot_num)# 机器人总数
# # # 获取单个机器人的姿态
# # getRobotPose = SwarmRobot.get_robot_pose(0) # 0是第一个# 获取第1个机器人的姿态
# # getRobotPose = SwarmRobot.get_robot_pose(1) # 0是第一个# 获取第二个机器人的姿态
# # getRobotPose = SwarmRobot.get_robot_pose(2) # 0是第一个# 获取第3个机器人的姿态
# # getRobotPose = SwarmRobot.get_robot_pose(3) # 0是第一个# 获取第4个机器人的姿态
# # getRobotPose = SwarmRobot.get_robot_pose(4) # 0是第一个# 获取第5个机器人的姿态
# 获取所有机器人的姿态
getAllRobotPose = SwarmRobot.get_robot_poses() # 获取所有机器人的姿态
print(getAllRobotPose)# 获取到的姿态
v=0
w=1
# 移动单个机器人
rate.sleep()
flag = SwarmRobot.move_robot(3, v, w)
rate.sleep()
flag = SwarmRobot.move_robot(4, v, w)
print(flag)
# 移动所有机器人
speed = []
for i in range(SwarmRobot.robot_num):
    speed.append([v,w])
rate.sleep()
flag = SwarmRobot.move_robots(speed)
print(flag)
# 停止单个机器人
rate.sleep()
SwarmRobot.stop_robot(0)
# 停下所有机器人
rate.sleep()
SwarmRobot.stop_robots()


# spin
# rospy.spin()