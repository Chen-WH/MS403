/* 
 * Date: 2021-10-26
 * Description: To same angle
 */

#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>

using std::cout;
using std::endl;


/* First: Set ids of swarm robot based on Aruco marker */
std::vector<int> swarm_robot_id = {1, 2, 3, 4, 5};
std::vector<ros::Publisher> swarm_robot_cmd_vel_pub(swarm_robot_id.size());


/* Get gazebo robot pose to vector*/
bool getGazeboRobotPose(int index, std::vector<double> &pose_cur, tf::TransformListener& tf_listener) {
    if(index >= swarm_robot_id.size() ) {
        ROS_ERROR_STREAM("Invalid index of getting pose request!");
        return false;
    }

    tf::StampedTransform transform;
    std::string robot_frame = "robot_" + std::to_string(swarm_robot_id[index]) + "/base_footprint";
    std::string base_marker = "robot_" + std::to_string(swarm_robot_id[index]) + "/odom";


    // Try to get pose of robot from tf
    try{
        tf_listener.waitForTransform(base_marker, robot_frame, ros::Time(0), ros::Duration(0.5));
        tf_listener.lookupTransform(base_marker, robot_frame, ros::Time(0), transform);
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
bool getGazeboRobotPose(std::vector<std::vector<double> > &swarm_pose_cur, tf::TransformListener& tf_listener) {
    swarm_pose_cur.resize(swarm_robot_id.size());
    for(int index =0; index < swarm_robot_id.size(); index++) {
        tf::StampedTransform transform;
        std::string robot_frame = "robot_" + std::to_string(swarm_robot_id[index]) + "/base_footprint";
        std::string base_marker = "robot_" + std::to_string(swarm_robot_id[index]) + "/odom";


        // Try to get pose of robot from tf
        try{
            tf_listener.waitForTransform(base_marker, robot_frame, ros::Time(0), ros::Duration(0.5));
            tf_listener.lookupTransform(base_marker, robot_frame, ros::Time(0), transform);
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
        swarm_pose_cur[index].resize(3);
        swarm_pose_cur[index][0] = transform.getOrigin().x();
        swarm_pose_cur[index][1] = transform.getOrigin().y();
        swarm_pose_cur[index][2] = yaw;

        ROS_INFO_STREAM("Get pose of robot_" << swarm_robot_id[index] << " is: x=" << swarm_pose_cur[index][0] << " y=" << swarm_pose_cur[index][1] << " theta=" << swarm_pose_cur[index][2]);
    }
}


/* Move robot */
bool moveRobot(int index, double v, double w) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = v;
    vel_msg.angular.z = w;
    swarm_robot_cmd_vel_pub[index].publish(vel_msg);
    // ros::Duration(0.5).sleep();
    // swarm_robot_cmd_vel_pub[index].publish(vel_msg);
    ROS_INFO_STREAM("Move robot_" << swarm_robot_id[index] << " with v=" << v << " w=" << w);
    return true;
}


/* Stop robot */
bool stopRobot(int index) {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    swarm_robot_cmd_vel_pub[index].publish(vel_msg);
    ROS_INFO_STREAM("Stop robot_" << swarm_robot_id[index]);
    return true;
}

/* Stop all robots */
bool stopRobot() {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.0;
    for(int i = 0; i < swarm_robot_cmd_vel_pub.size(); i++) {
        swarm_robot_cmd_vel_pub[i].publish(vel_msg);
    }
    ROS_INFO_STREAM("Stop all robots.");
    return true;
}


/* Check velocity */
double checkVel(double v, double max_v, double min_v) {
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


/* Main function */
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
    // while(1) {

        /* Judge whether reached */
        del_y = -lap * cur_y;
        del_theta = -lap * cur_theta;
        is_conv = true;
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            if(std::fabs(del_y(i)) > conv_th || std::fabs(del_theta(i)) > conv_th) {
                is_conv = false;
                // break;
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
                if (std::fabs((cur_y(j) - cur_y(i))/(cur_x(j) - cur_x(i)) - cur_theta(i)) < 0.05){
                    v = 0;
                    w = MIN_W;
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

