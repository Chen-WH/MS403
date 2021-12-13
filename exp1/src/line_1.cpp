/* 
 * Date: 2021-11-30
 * Description: To a line y = Const
 * Group: 2
 */

#include <swarm_robot_control.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;

    /* Initialize swarm robot */
    for(int index = 0; index < swarm_robot_id.size(); index++) {
        std::string vel_topic = "/robot_" + std::to_string(swarm_robot_id[index]) + "/cmd_vel";
        swarm_robot_cmd_vel_pub[index] = nh.advertise<geometry_msgs::Twist>(vel_topic, 10);
    }

    /* Set L Matrix*/
    Eigen::MatrixXd lap(swarm_robot_id.size(), swarm_robot_id.size());
    lap <<  4, -1, -1, -1, -1,
            -1, 4, -1, -1, -1,
            -1, -1, 4, -1, -1,
            -1, -1, -1, 4, -1,
            -1, -1, -1, -1, 4;

    /* Convergence threshold */
    double conv_th = 0.02;  // Threshold of angle, in rad
    double conv_dis = 0.02;  // Threshold of distance

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
    std::vector<bool> flag_pose(swarm_robot_id.size(), false);
    bool flag = false;
    while(! flag) {
        flag = true;
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            flag = flag && flag_pose[i];
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            std::vector<double> pose_robot(3);
            if(getGazeboRobotPose(i, pose_robot, tf_listener)) {
                current_robot_pose[i] = pose_robot;
                flag_pose[i] = true;
            }
        }
    }
    ROS_INFO_STREAM("Succeed getting pose!");
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_theta(i) = current_robot_pose[i][2];
    }
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_y(i) = current_robot_pose[i][1];
    }
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_x(i) = current_robot_pose[i][0];
    }

    /* Convergence sign */
    bool is_conv = false;   // Convergence sign of angle
    /* While loop */
    while(! is_conv) {
        /* Judge whether reached */
        del_y = -lap * cur_y;
        del_theta = -lap * cur_theta;
        is_conv = true;
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            if(std::fabs(del_y(i)) > conv_dis || std::fabs(del_theta(i)) > conv_th) {
                is_conv = false;
            }       
        }

        /* Swarm robot move */
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            bool crash_flag = false;
            double v = del_y(i) * k_v;
            double w = del_theta(i) * k_w;
            for (int j = 0; j < swarm_robot_id.size(); j++){
                if (j == i){
                    continue;
                }
                // 判断是否有其他机器人在行进方向上
                if (std::fabs((cur_y(j) - cur_y(i))/(cur_x(j) - cur_x(i)) - cur_theta(i)) < 0.05 &&\
                 sqrt((cur_x(j) - cur_x(i))*(cur_x(j) - cur_x(i)) + (cur_y(j) - cur_y(i))*(cur_y(j) - cur_y(i))) < 1){
                    v = 0;
                }
            }
            if (cur_theta(i) < 0){
                v = -v;
            }
            v = checkVel(v, MAX_V, MIN_V);
            w = checkVel(w, MAX_W, MIN_W);
            moveRobot(i, v, w);
        }

        /* Time sleep for robot move */
        ros::Duration(0.05).sleep();

        /* Get swarm robot poses */
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            std::vector<double> pose_robot(3);
            if(getGazeboRobotPose(i, pose_robot, tf_listener)) {
                current_robot_pose[i] = pose_robot;
            }
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_theta(i) = current_robot_pose[i][2];
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_y(i) = current_robot_pose[i][1];
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            cur_x(i) = current_robot_pose[i][0];
        }
    }

    /* Stop all robots */
    stopRobot();
    ROS_INFO_STREAM("Succeed!");
    return 0;
}

