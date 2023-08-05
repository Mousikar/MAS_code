/* 
 * Date: 2021-11-29
 * Description: the basic function
 */

#include <gazebo_swarm_robot_tb3/swarm_robot_control.h>

SwarmRobot::SwarmRobot(ros::NodeHandle *nh, std::vector<int> swarm_robot_id_):
        swarm_robot_id(swarm_robot_id_)
{

    this->robot_num = swarm_robot_id.size();

    std::cout << "robot_num="<< robot_num <<std::endl;
    /* Initialize swarm robot */
    for(int i = 0; i < 10; i++) {
        std::string vel_topic = "/robot_" + std::to_string(i+1) + "/cmd_vel";
        cmd_vel_pub[i] = nh_.advertise<geometry_msgs::Twist>(vel_topic, 10);
    }

}


SwarmRobot::~SwarmRobot()
{

}


/* Get the [index]th robot's pose to vector*/
bool SwarmRobot::getRobotPose(int index, std::vector<double> &pose_cur) {

	pose_cur.resize(3,0.0);
    tf::StampedTransform transform;
    std::string robot_frame = "robot_" + std::to_string(this->swarm_robot_id[index]) + "/base_footprint";
    std::string base_marker = "robot_" + std::to_string(this->swarm_robot_id[index]) + "/odom";


    // Try to get pose of robot from tf
    try{
        this->tf_listener.waitForTransform(base_marker, robot_frame, ros::Time(0), ros::Duration(0.5));
        this->tf_listener.lookupTransform(base_marker, robot_frame, ros::Time(0), transform);
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        // ros::Duration(1.0).sleep();
        return false;
    }

    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Get pose
    pose_cur.resize(3);
    pose_cur[0] = transform.getOrigin().x();
    pose_cur[1] = transform.getOrigin().y();
    pose_cur[2] = yaw;

    ROS_INFO_STREAM("Get pose of robot_" << swarm_robot_id[index] << " is: x=" << pose_cur[0] << " y=" << pose_cur[1] << " theta=" << pose_cur[2]);
    return true;
}


/* Get gazebo pose of swarm robot */
bool SwarmRobot::getRobotPose(std::vector<std::vector<double> > &current_robot_pose) {

    current_robot_pose.resize(this->robot_num);
    std::vector<bool> flag_pose(this->robot_num, false);

    bool flag = false;

    while(! flag) {
        flag = true;
        for(int i = 0; i < this->robot_num; i++) {
            flag = flag && flag_pose[i];
        }
        for(int i = 0; i < this->robot_num; i++) {
            std::vector<double> pose_robot(3);
            if(getRobotPose(i, pose_robot)) {
                current_robot_pose[i] = pose_robot;
                flag_pose[i] = true;
            }
        }
    }
    ROS_INFO_STREAM("Succeed getting pose!");
    return true;   
}

/* Move robot: index represents the order in swarm_robot_id*/
bool SwarmRobot::moveRobot(int index, double v, double w) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = v;
    vel_msg.angular.z = w;
    cmd_vel_pub[swarm_robot_id[index]-1].publish(vel_msg);
    ROS_INFO_STREAM("Move robot_" << swarm_robot_id[index] << " with v=" << v << " w=" << w);
    return true;
}

/* Move all robots*/
bool SwarmRobot::moveRobot(std::vector< std::vector<double> > &speed) {
  if(this->robot_num != speed.size())
  {
    ROS_INFO_STREAM("The robot number does not equal the speed number!");
    return false;
  }

  for(int i=0; i<this->robot_num; i++)
  {
    if(!this->moveRobot(this->swarm_robot_id[i],speed[i][0],speed[i][1]))
    {
      return false;
    }
  }
  return true;
}


/* Stop robot */
bool SwarmRobot::stopRobot(int index) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    cmd_vel_pub[swarm_robot_id[index]-1].publish(vel_msg);
    ROS_INFO_STREAM("Stop robot_" << swarm_robot_id[swarm_robot_id[index]]);
    return true;
}

/* Stop all robot */
bool SwarmRobot::stopRobot() {

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    for(int i = 0; i < this->robot_num; i++) {
        cmd_vel_pub[this->swarm_robot_id[i]-1].publish(vel_msg);
    }
    ROS_INFO_STREAM("Stop all robots.");
    return true;
}

/* Check velocity */
double SwarmRobot::checkVel(double v, double max_v, double min_v) {
    if(max_v <= 0 || min_v <= 0) {
        std::cout << "Error input of checkW()" << std::endl;
        return v;
    }
    
    if(v > 0) {
        v = std::max(v, min_v);
        v = std::min(v, max_v);
    } else {
        v = std::min(v, -min_v);
        v = std::max(v, -max_v);
    }
    return v;
}