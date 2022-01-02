/*
 * proj.cpp
 * MS403 Project code1
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <cmath>

#define pi 3.1415926535

using namespace std;
using namespace Eigen;
///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

/* 参数定义 */
double a = 2, b = 6;
double c1 = 16/pi/pi, c2 = 16/pi/pi, c3 = 1/sqrt(a*a+b*b)/4;
double z_lock = 2;                             // 飞行高度
double k_v = 0.1, k_w = 1.2;  // 比例系数
double epsilon = 1e-3;
double w = 0.15;
//initial position offset
double x_ini[4] = {2.0, 0.0, -2.0, 0.0};
double y_ini[4] = {0.0, 6.0, 0.0, -6.0};
/* 定义变量 */
VectorXd theta(4);
VectorXd phi(4);
VectorXd x(4);
VectorXd y(4);
VectorXd z(4);
VectorXd u_theta(4);
VectorXd u_phi(4);

geometry_msgs::PoseStamped pose0;
geometry_msgs::PoseStamped pose1;
geometry_msgs::PoseStamped pose2;
geometry_msgs::PoseStamped pose3;
nav_msgs::Odometry pose_odom0;
nav_msgs::Odometry pose_odom1;
nav_msgs::Odometry pose_odom2;
nav_msgs::Odometry pose_odom3;
mavros_msgs::State current_state;

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void update_theta();
void update_state();
geometry_msgs::Vector3 quaternion2euler(geometry_msgs::Quaternion quater);
geometry_msgs::Quaternion euler2quaternion(geometry_msgs::Vector3 euler);
double get_lambda2(VectorXd Theta, VectorXd Phi);
void update_pub();
double get_angle(double angle1, double angle2);

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void plane_pos_cb0(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom0 = *msg;
}
void plane_pos_cb1(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom1 = *msg;
}
void plane_pos_cb2(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom2 = *msg;
}
void plane_pos_cb3(const nav_msgs::Odometry::ConstPtr &msg){
    pose_odom3 = *msg;
}

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    //uav0
    ros::NodeHandle nh0;
    ros::Subscriber state_sub0 = nh0.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub0 = nh0.subscribe<nav_msgs::Odometry>("uav0/mavros/local_position/odom", 1, plane_pos_cb0);
    ros::Publisher local_pos_pub0 = nh0.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client0 = nh0.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client0 = nh0.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");

    //uav1
    ros::NodeHandle nh1;
    ros::Subscriber state_sub1 = nh1.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub1 = nh1.subscribe<nav_msgs::Odometry>("uav1/mavros/local_position/odom", 1, plane_pos_cb1);
    ros::Publisher local_pos_pub1 = nh1.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client1 = nh1.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = nh1.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");

    //uav2
    ros::NodeHandle nh2;
    ros::Subscriber state_sub2 = nh2.subscribe<mavros_msgs::State>("uav2/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub2 = nh2.subscribe<nav_msgs::Odometry>("uav2/mavros/local_position/odom", 1, plane_pos_cb2);
    ros::Publisher local_pos_pub2 = nh2.advertise<geometry_msgs::PoseStamped>("/uav2/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client2 = nh2.serviceClient<mavros_msgs::CommandBool>("/uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client2 = nh2.serviceClient<mavros_msgs::SetMode>("/uav2/mavros/set_mode");

    //uav3
    ros::NodeHandle nh3;
    ros::Subscriber state_sub3 = nh3.subscribe<mavros_msgs::State>("uav3/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub3 = nh3.subscribe<nav_msgs::Odometry>("uav3/mavros/local_position/odom", 1, plane_pos_cb3);
    ros::Publisher local_pos_pub3 = nh3.advertise<geometry_msgs::PoseStamped>("/uav3/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client3 = nh3.serviceClient<mavros_msgs::CommandBool>("/uav3/mavros/cmd/arming");
    ros::ServiceClient set_mode_client3 = nh3.serviceClient<mavros_msgs::SetMode>("/uav3/mavros/set_mode");

    ros::Rate rate(10.0);

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>解 锁 飞 机<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    pose0.pose.position.x = 0;
    pose0.pose.position.y = 0;
    pose0.pose.position.z = z_lock;
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = z_lock;
    pose2.pose.position.x = 0;
    pose2.pose.position.y = 0;
    pose2.pose.position.z = z_lock;
    pose3.pose.position.x = 0;
    pose3.pose.position.y = 0;
    pose3.pose.position.z = z_lock;

    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub0.publish(pose0);
        local_pos_pub1.publish(pose1);
        local_pos_pub2.publish(pose2);
        local_pos_pub3.publish(pose3);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client0.call(offb_set_mode) &&
                set_mode_client1.call(offb_set_mode) &&
                set_mode_client2.call(offb_set_mode) &&
                set_mode_client3.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled : 4");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client0.call(arm_cmd) &&
                    arming_client1.call(arm_cmd) &&
                    arming_client2.call(arm_cmd) &&
                    arming_client3.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed : 4");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub0.publish(pose0);
        local_pos_pub1.publish(pose1);
        local_pos_pub2.publish(pose2);
        local_pos_pub3.publish(pose3);
        if (pose_odom0.pose.pose.position.z > 1.8) {
            ROS_INFO("plane takeoff !");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    while (ros::ok()) {
        update_state();
        VectorXd theta_tmp, phi_tmp;
        for (int i = 0; i < 4; ++i){
            theta_tmp = theta;
            theta_tmp(i) += epsilon;
            u_theta(i) = (get_lambda2(theta_tmp, phi) - get_lambda2(theta, phi))/epsilon;
            phi_tmp = phi;
            phi_tmp(i) += epsilon;
            u_phi(i) = (get_lambda2(theta, phi_tmp) - get_lambda2(theta, phi))/epsilon;
            cout << "uav" << i << ' ' << u_theta(i) << ' ' << u_phi(i) << endl;
        }
        update_pub();
        local_pos_pub0.publish(pose0);
        local_pos_pub1.publish(pose1);
        local_pos_pub2.publish(pose2);
        local_pos_pub3.publish(pose3);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void update_theta(){
    for (int i = 0; i < 4; ++i){
        if (x(i) > 0){
            theta(i) = atan(a*y(i)/b/x(i));
        }
        else {
            if (y(i) > 0){
                theta(i) = atan(a*y(i)/b/x(i)) + pi;
            }
            else {
                theta(i) = atan(a*y(i)/b/x(i)) - pi;
            }
        }
    }
}

void update_state(){
    x(0) = pose_odom0.pose.pose.position.x + x_ini[0];
    y(0) = pose_odom0.pose.pose.position.y + y_ini[0];
    z(0) = pose_odom0.pose.pose.position.z;
    phi(0) = quaternion2euler(pose_odom0.pose.pose.orientation).z;
    x(1) = pose_odom1.pose.pose.position.x + x_ini[1];
    y(1) = pose_odom1.pose.pose.position.y + y_ini[1];
    z(1) = pose_odom1.pose.pose.position.z;
    phi(1) = quaternion2euler(pose_odom1.pose.pose.orientation).z;
    x(2) = pose_odom2.pose.pose.position.x + x_ini[2];
    y(2) = pose_odom2.pose.pose.position.y + y_ini[2];
    z(2) = pose_odom2.pose.pose.position.z;
    phi(2) = quaternion2euler(pose_odom2.pose.pose.orientation).z;
    x(3) = pose_odom3.pose.pose.position.x + x_ini[3];
    y(3) = pose_odom3.pose.pose.position.y + y_ini[3];
    z(3) = pose_odom3.pose.pose.position.z;
    phi(3) = quaternion2euler(pose_odom3.pose.pose.orientation).z;
    update_theta();
}

geometry_msgs::Vector3 quaternion2euler(geometry_msgs::Quaternion quater){
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (quater.w * quater.x + quater.y * quater.z), 1.0 - 2.0 * (quater.x * quater.x + quater.y * quater.y));
    temp.y = asin(2.0 * (quater.w * quater.y - quater.z * quater.x));
    temp.z = atan2(2.0 * (quater.w * quater.z + quater.x * quater.y), 1.0 - 2.0 * (quater.y * quater.y + quater.z * quater.z));
    return temp;
}

geometry_msgs::Quaternion euler2quaternion(geometry_msgs::Vector3 euler)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(euler.x/2)*cos(euler.y/2)*cos(euler.z/2) + sin(euler.x/2)*sin(euler.y/2)*sin(euler.z/2);
    temp.x = sin(euler.x/2)*cos(euler.y/2)*cos(euler.z/2) - cos(euler.x/2)*sin(euler.y/2)*sin(euler.z/2);
    temp.y = cos(euler.x/2)*sin(euler.y/2)*cos(euler.z/2) + sin(euler.x/2)*cos(euler.y/2)*sin(euler.z/2);
    temp.z = cos(euler.x/2)*cos(euler.y/2)*sin(euler.z/2) - sin(euler.x/2)*sin(euler.y/2)*cos(euler.z/2);
    return temp;
}

double get_lambda2(VectorXd Theta, VectorXd Phi){
    VectorXd X(4);VectorXd Y(4);
    MatrixXd psi(4, 4);
    MatrixXd l(4, 4);
    MatrixXd lap(4, 4);
    for (int i = 0; i < 4; ++i){
        X(i) = a*cos(Theta(i));
        Y(i) = b*sin(Theta(i));
    }
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            if (X(j) - X(i) > 0){
                psi(i, j) = atan((Y(j) - Y(i))/(X(j) - X(i)));
            }
            else {
                if (Y(j) - Y(i) > 0){
                    psi(i, j) = atan((Y(j) - Y(i))/(X(j) - X(i))) + pi;
                }
                else {
                    psi(i, j) = atan((Y(j) - Y(i))/(X(j) - X(i))) - pi;
                }
            }
        }
    }
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            l(i, j) = sqrt((X(j) - X(i))*(X(j) - X(i)) + (Y(j) - Y(i))* (Y(j) - Y(i)));
        }
    }
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            if (i == j){
                lap(i, j) = 0;
            }
            else {
                if (j < i){
                    lap(i, j) = lap(j, i);
                }
                else {
                    lap(i, j) = -exp(-c1*(psi(i, j) - Phi(i))*(psi(i, j) - Phi(i)) - c2*Phi(i)*Phi(i)\
                     - c3*(l(i, j) - sqrt(a*a+b*b)/2)*(l(i, j) - sqrt(a*a+b*b)/2))\
                    -exp(-c1*(psi(j, i) - Phi(j))*(psi(j, i) - Phi(j)) - c2*Phi(j)*Phi(j)\
                     - c3*(l(i, j) - sqrt(a*a+b*b)/2)*(l(i, j) - sqrt(a*a+b*b)/2));
                }
            }
        }
    }
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            if (i == j){
                continue;
            }
            else {
                lap(i, i) -= lap(i, j);
            }
        }
    }
    EigenSolver<Eigen::MatrixXd> es(lap);
    MatrixXcd evals = es.eigenvalues();
    VectorXd evalsReal = evals.real();
    double lambda1, lambda2;
    if (evalsReal(0) < evalsReal(1)){
        lambda1 = evalsReal(0);
        lambda2 = evalsReal(1);
    }
    else {
        lambda1 = evalsReal(1);
        lambda2 = evalsReal(0);
    }
    if (evalsReal(2) < lambda1){
        lambda2 = lambda1;
        lambda1 = evalsReal(2);
    }
    else {
        if (evalsReal(2) < lambda2){
            lambda2 = evalsReal(2);
        }
    }
    if (evalsReal(3) < lambda1){
        lambda2 = lambda1;
        lambda1 = evalsReal(3);
    }
    else {
        if (evalsReal(3) < lambda2){
            lambda2 = evalsReal(3);
        }
    }
    return lambda2;
}

void update_pub(){
    geometry_msgs::Vector3 euler;
    euler = quaternion2euler(pose_odom0.pose.pose.orientation);
    euler.z += k_w*u_phi(0);
    pose0.pose.orientation = euler2quaternion(euler);
    euler = quaternion2euler(pose_odom1.pose.pose.orientation);
    euler.z += k_w*u_phi(1);
    pose1.pose.orientation = euler2quaternion(euler);
    euler = quaternion2euler(pose_odom2.pose.pose.orientation);
    euler.z += k_w*u_phi(2);
    pose2.pose.orientation = euler2quaternion(euler);
    euler = quaternion2euler(pose_odom3.pose.pose.orientation);
    euler.z += k_w*u_phi(3);
    pose3.pose.orientation = euler2quaternion(euler);
    pose0.pose.position.x = a*cos(theta(0) + k_v*u_theta(0) + w) - x_ini[0];
    pose1.pose.position.x = a*cos(theta(1) + k_v*u_theta(1) + w) - x_ini[1];
    pose2.pose.position.x = a*cos(theta(2) + k_v*u_theta(2) + w) - x_ini[2];
    pose3.pose.position.x = a*cos(theta(3) + k_v*u_theta(3) + w) - x_ini[3];
    pose0.pose.position.y = b*sin(theta(0) + k_v*u_theta(0) + w) - y_ini[0];
    pose1.pose.position.y = b*sin(theta(1) + k_v*u_theta(1) + w) - y_ini[1];
    pose2.pose.position.y = b*sin(theta(2) + k_v*u_theta(2) + w) - y_ini[2];
    pose3.pose.position.y = b*sin(theta(3) + k_v*u_theta(3) + w) - y_ini[3];
    pose0.pose.position.z = z_lock;
    pose1.pose.position.z = z_lock;
    pose2.pose.position.z = z_lock;
    pose3.pose.position.z = z_lock;
}