/* 
 * Date: 2021-11-29
 * Description: to the same angle
 */

#include <gazebo_swarm_robot_tb3/swarm_robot_control.h>


/* Main function */
int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;
    
    /* First: Set ids of swarm robot based on Aruco marker */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5, 6};

    /* Initialize swarm robot */
    SwarmRobot swarm_robot(&nh, swarm_robot_id);

    /* Set L Matrix*/
    Eigen::MatrixXd lap(swarm_robot_id.size(), swarm_robot_id.size());
    lap <<  4, -1, -1, -1, -1, -1,
            -1, 4, -1, -1, -1, -1,
            -1, -1, 4, -1, -1, -1,
            -1, -1, -1, 4, -1, -1,
            -1, -1, -1, -1, 4, -1,
            -1, -1, -1, -1, -1, 4;

    /* Convergence threshold */
    double conv_th = 0.05;  // Threshold of angle, in rad

    /* Velocity scale and threshold */
    double MAX_W = 1;       // Maximum angle velocity (rad/s)
    double MIN_W = 0.05;    // Minimum angle velocity(rad/s)
    double MAX_V = 0.2;     // Maximum linear velocity(m/s)
    double MIN_V = 0.01;    // Minimum linear velocity(m/s)
    double k_w = 0.1;       // Scale of angle velocity
    double k_v = 0.1;       // Scale of linear velocity


    /* Mobile robot poses and for next poses */
    Eigen::VectorXd cur_x(swarm_robot_id.size());
    Eigen::VectorXd cur_y(swarm_robot_id.size());
    Eigen::VectorXd cur_theta(swarm_robot_id.size());
    Eigen::VectorXd del_x(swarm_robot_id.size());
    Eigen::VectorXd del_y(swarm_robot_id.size());
    Eigen::VectorXd del_theta(swarm_robot_id.size());


    /* Get swarm robot poses firstly */
    std::vector<std::vector<double> > current_robot_pose(swarm_robot_id.size());

    swarm_robot.getRobotPose(current_robot_pose);

    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_theta(i) = current_robot_pose[i][2];
    }


    /* Convergence sign */
    bool is_conv = false;   // Convergence sign of angle
    /* While loop */
    while(! is_conv) {
    // while(1) {

        /* Judge whether reached */
        del_theta = -lap * cur_theta;
        is_conv = true;
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            if(std::fabs(del_theta(i)) > conv_th) {
                is_conv = false;
                // break;
            }       
        }


        /* Swarm robot move */
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            double w = del_theta(i) * k_w;
            w = swarm_robot.checkVel(w, MAX_W, MIN_W);
            swarm_robot.moveRobot(i, 0.0, w);
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();


        /* Get swarm robot poses */
        swarm_robot.getRobotPose(current_robot_pose);
        
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_theta(i) = current_robot_pose[i][2];
        }
    }

    /* Stop all robots */
    swarm_robot.stopRobot();

    ROS_INFO_STREAM("Succeed!");
    return 0;
}

