/* 
 * Date: 2021-10-28
 * Description: Turn all robots
 */

#include <ros/ros.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>


/* First: Set ids of swarm robot based on Aruco marker */
std::vector<int> swarm_robot_id = {1, 2, 3, 4, 5, 6, 7, 8, 9};
std::vector<ros::Publisher> swarm_robot_cmd_vel_pub(swarm_robot_id.size());


/* Turn all robots */
bool turnRobot() {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = 0.4;
    for(int i = 0; i < swarm_robot_cmd_vel_pub.size(); i++) {
        swarm_robot_cmd_vel_pub[i].publish(vel_msg);
    }
    ROS_INFO_STREAM("Turn all robots.");
    return true;
}


/* Main function */
int main(int argc, char** argv) {
    ros::init(argc, argv, "turn_robot");
    ros::NodeHandle nh;

    /* Initialize swarm robot */
    for(int index = 0; index < swarm_robot_id.size(); index++) {
        std::string vel_topic = "/robot_" + std::to_string(swarm_robot_id[index]) + "/cmd_vel";
        swarm_robot_cmd_vel_pub[index] = nh.advertise<geometry_msgs::Twist>(vel_topic, 10);
    }

    /* while loop for turning robots */
    while(ros::ok()) {
        turnRobot();
        ros::Duration(0.5).sleep();
    }


    ROS_WARN_STREAM("Turn all robots!");
    return 0;
}

