#! /usr/bin/env python
# encoding: utf-8
import os
import sys
import rospy
path = os.path.abspath(".")
sys.path.insert(0,path + "/src/gazebo_swarm_robot_tb3/scripts")
import swarm_robot_control_h

# 初始化节点
rospy.init_node("test")
index = [1,2,3,4,5]
# 建立对象
SwarmRobot = swarm_robot_control_h.SwarmRobot(index)
# 获取第1个机器人的姿态
getRobotPose = SwarmRobot.getRobotPose(0) # 0是第一个
# 获取到的姿态
print(getRobotPose)
# 获取第二个机器人的姿态
getRobotPose = SwarmRobot.getRobotPose(1) # 0是第一个
# 获取到的姿态
print(getRobotPose)
# 获取第3个机器人的姿态
getRobotPose = SwarmRobot.getRobotPose(2) # 0是第一个
# 获取到的姿态
print(getRobotPose)
# 获取第4个机器人的姿态
getRobotPose = SwarmRobot.getRobotPose(3) # 0是第一个
# 获取到的姿态
print(getRobotPose)
# 获取第5个机器人的姿态
getRobotPose = SwarmRobot.getRobotPose(4) # 0是第一个
# 获取到的姿态
print(getRobotPose)
# 获取所有机器人的姿态
getAllRobotPose = SwarmRobot.getAllRobotPose() 
# 获取到的姿态
print(getAllRobotPose)
# 机器人节点V
print("swarm_robot_id = ",SwarmRobot.swarm_robot_id)
# 机器人总数
print("robot_num = ",SwarmRobot.robot_num)
# spin
rospy.spin()