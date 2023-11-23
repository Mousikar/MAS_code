#! /usr/bin/env python2.7
# encoding: utf-8
import rospy
import tf
from geometry_msgs.msg import Twist

class SwarmRobot:
    def __init__(self, swarm_robot_id):
        self.swarm_robot_id = swarm_robot_id
        self.robot_num = len(swarm_robot_id)        
        self.cmd_vel_pub = [rospy.Publisher("/robot_{}/cmd_vel".format(i+1), Twist, queue_size=1) for i in range(self.robot_num)]
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(100)

    def get_robot_pose(self, index):
        pose_cur = [0.0, 0.0, 0.0]
        robot_frame = "robot_{}/base_footprint".format(self.swarm_robot_id[index])
        base_marker = "robot_{}/odom".format(self.swarm_robot_id[index])

        try:
            self.tf_listener.waitForTransform(base_marker, robot_frame, rospy.Time(0), rospy.Duration(0.5))
            (trans, rot) = self.tf_listener.lookupTransform(base_marker, robot_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logerror(str(ex))
            return False

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
        pose_cur[0] = trans[0]
        pose_cur[1] = trans[1]
        pose_cur[2] = yaw
        # rospy.loginfo("Get pose of robot_{}: x={} y={} theta={}".format(
        #     self.swarm_robot_id[index],
        #     pose_cur[0],
        #     pose_cur[1],
        #     pose_cur[2]))
        return True, pose_cur

    def get_robot_poses(self):
        current_robot_pose = []
        self.flag_pose = [False for _ in range(self.robot_num)]
        while True:
            flag = all(flag_pose for flag_pose in self.flag_pose)
            if flag:
                break

            for i in range(self.robot_num):
                success, pose_robot = self.get_robot_pose(i)
                if success:
                    current_robot_pose.append(pose_robot)
                    self.flag_pose[i] = True

        # rospy.loginfo("Succeed getting pose!")
        return current_robot_pose

    def move_robot(self, index, v, w):
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.angular.z = w
        # for i in range(20):
        self.rate.sleep()
        self.cmd_vel_pub[index].publish(vel_msg)
        # rospy.loginfo("Move robot_{} with v={} w={}".format(
        #     self.swarm_robot_id[index],
        #     v,
        #     w
        # ))
        return True

    def move_robots(self, speed):
        if len(speed) != self.robot_num:
            rospy.loginfo("The robot number does not equal the speed number!")
            return False

        for i in range(self.robot_num):
            if not self.move_robot(i, speed[i][0], speed[i][1]):
                return False
        return True

    def stop_robot(self, index):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        # for i in range(20):
        self.rate.sleep()
        self.cmd_vel_pub[index].publish(vel_msg)
        rospy.loginfo("Stop robot_{}".format(self.swarm_robot_id[index]))
        return True

    def stop_robots(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        for i in range(self.robot_num):
            self.cmd_vel_pub[i].publish(vel_msg)
        rospy.loginfo("Stop all robots.")
        return True

    @staticmethod
    def check_vel(v, max_v, min_v):
        if max_v <= 0 or min_v <= 0:
            rospy.loginfo("Error input of checkW()")
            return v

        if v > 0:
            v = max(min(v, max_v), min_v)
        else:
            v = min(max(v, -min_v), -max_v)
        return v
