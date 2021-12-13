/* 
 * Date: 2021-11-29
 * Description: To a line
 */

#include <swarm_robot_control.h>


/* Main function */
int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;
    
    /* First: Set ids of swarm robot based on Aruco marker */
    std::vector<int> swarm_robot_id{1, 2, 3, 4, 5};

    /* Initialize swarm robot */
    SwarmRobot swarm_robot(&nh, swarm_robot_id);

    int robot_num = swarm_robot_id.size();

    /* Set L Matrix */
    Eigen::MatrixXd lap(robot_num, robot_num);
    lap <<  4, -1, -1, -1, -1,
            -1, 4, -1, -1, -1,
            -1, -1, 4, -1, -1,
            -1, -1, -1, 4, -1,
            -1, -1, -1, -1, 4;


    /* Convergence threshold */
    double conv_th = 0.5;   // Threshold of angle, in rad
    double dis_th = 0.1;    // Threshold of distance, in m


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

    Eigen::VectorXd d(robot_num);
    Eigen::VectorXd d_(robot_num);


    /* Get swarm robot poses firstly */
    std::vector<std::vector<double> > current_robot_pose(swarm_robot_id.size());
    swarm_robot.getRobotPose(current_robot_pose);


    /* x,y,theta */
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_x(i) = current_robot_pose[i][0];
        cur_y(i) = current_robot_pose[i][1];
        cur_theta(i) = current_robot_pose[i][2];
    }


    /* Convergence sign */
    bool is_angled = false;    // Convergence sign of angle
    bool is_shaped = false;  // Convergence sign of shape


    double theta_sum;

    /* While loop */
    while(! is_angled) {

        theta_sum = 0;

        /* Judge whether reached */
        del_theta = -lap * cur_theta;
        
        
        if(del_theta.norm() < conv_th) {
                is_angled = true;
                // break;
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
            theta_sum += cur_theta(i);
        }
    }

    double theta_ave = theta_sum / robot_num;

    /* While loop */
    while(!is_shaped) {
        /* Get swarm robot poses */
        swarm_robot.getRobotPose(current_robot_pose);

        for (int i = 0; i < robot_num; i++)
        {
            cur_x(i) = current_robot_pose[i][0];
            cur_y(i) = current_robot_pose[i][1];
            d(i) = (cur_x(i)*cos(theta_ave) + cur_y(i)*sin(theta_ave))*1000;
        }

        /* Judge whether shape reached */
        d_ = -lap * d;

        /* Swarm robot move */
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            double v = d_(i) * k_v;
            v = swarm_robot.checkVel(v, MAX_W, MIN_W);
            swarm_robot.moveRobot(i, v, 0.0);
        }
        ros::Duration(0.2).sleep();

                //////
        is_shaped = true;


        for (int i = 0; i < robot_num; i++)
        {
            if (std::abs(d_(i)) > dis_th)
            {
                is_shaped = false;
            }
        }

        /* Get swarm robot poses */
        swarm_robot.getRobotPose(current_robot_pose);
    }

    /* Stop all robots */
    swarm_robot.stopRobot();

    ROS_INFO_STREAM("Succeed!");
    return 0;
}