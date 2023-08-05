/* 
 * Date: 2021-11-29
 * Description: Stop all robots
 */


#include <gazebo_swarm_robot_tb3/swarm_robot_control.h>


/* Main function */
int main(int argc, char** argv) {
    ros::init(argc, argv, "stop_robot");
    ros::NodeHandle nh;
    
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};

    SwarmRobot swarm_robot(&nh, swarm_robot_id);


    /* while loop for stopping robots */
    while(ros::ok()) {
        swarm_robot.stopRobot();
        ros::Duration(0.5).sleep();
    }

    ROS_WARN_STREAM("Stop all robots!");
    return 0;
}

