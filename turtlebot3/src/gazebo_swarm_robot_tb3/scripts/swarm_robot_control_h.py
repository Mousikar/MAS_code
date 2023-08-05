#! /usr/bin/env python2.7
# encoding: utf-8
num = 1000
import tf2_ros
import tf
import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R

class SwarmRobot(object):
    def __init__(self,swarm_robot_id):
        self.swarm_robot_id=swarm_robot_id
        self.robot_num=len(swarm_robot_id)
        # for i in range(10):
        #     vel_topic = "/robot_" + str(i+1) + "/cmd_vel"
        #     self.cmd_vel_pub_p[i] = rospy.Publisher(vel_topic, Twist, queue_size=10)
    def getRobotPose(self,index):
        robot_frame = 'robot_' + str(self.swarm_robot_id[index]) + '/base_footprint';
        base_marker = 'robot_' + str(self.swarm_robot_id[index]) + '/odom';
        print(robot_frame)
        # 创建 TF 订阅对象
        self.tf_listener = tf.TransformListener()
        tmptimenow = rospy.Time(0)
        self.tf_listener.waitForTransform(base_marker, robot_frame, tmptimenow, rospy.Duration(0.5))
        # rate = rospy.Rate(10)
        # 获得坐标变换
        try:
            # rate.sleep()
            transform, rotation = self.tf_listener.lookupTransform(base_marker, robot_frame, tmptimenow)
            # rospy.loginfo(transform)
            # rospy.loginfo(rotation)
        except Exception as e:
            rospy.logerr("错误提示:%s",e)
        #将四元数转换成欧拉角
        r = R.from_quat(rotation)
        euler = r.as_euler('xyz', degrees=False)
        # print(euler)
        getRobotPose=[transform[0],
                      transform[1],
                      euler[2]]
        return getRobotPose
