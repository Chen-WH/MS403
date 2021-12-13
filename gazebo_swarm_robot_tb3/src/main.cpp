/* 
 * Date: 2021-11-29
 * Description: Codes for reference
 */

#include <swarm_robot_control.h>

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"swarm_robot_control_node");

    ros::NodeHandle nh;

    /////////////////////////////需要调整的参数////////////////////////////////////
    //设置机器人数量
    std::vector<int> current_robot_id{ 1, 2, 3, 4, 5};

    int robot_num = current_robot_id.size();

    SwarmRobot swarm_robot(&nh, current_robot_id);

    //收敛阈值
    bool is_conv = false; //判断是否角度和直线均收敛
    bool is_th = false; //判断角度是否收敛到一致
    bool is_linear = false; //判断位置是否收敛到一条直线
    float conv_th = 0.10;  // 角度收敛的阈值in rad
    float conv_linear = 0.02; //直线收敛的阈值 in m
    float lamda = 0.05;//角速度系数
    float lamda2 = 100;//x方向速度系数

    /* Velocity scale and threshold */
    double MAX_W = 1;       // Maximum angle velocity (rad/s)
    double MIN_W = 0.05;    // Minimum angle velocity(rad/s)
    double MAX_V = 0.2;     // Maximum linear velocity(m/s)
    double MIN_V = 0.01;    // Minimum linear velocity(m/s)
    double k_w = 0.1;       // Scale of angle velocity
    double k_v = 0.1;       // Scale of linear velocity

    unsigned int time_span = 5;   // 1*100 ms = 0.1s

    std::vector<std::vector<double>> current_robot_pose;
    current_robot_pose.clear();

    
    ////
    Eigen::MatrixXd lap(robot_num, robot_num);
    // 完全图
    lap <<   4, -1, -1, -1, -1,
            -1,  4, -1, -1, -1,
            -1, -1,  4, -1, -1,
            -1, -1, -1,  4, -1,
            -1, -1, -1, -1,  4;
    // 连通图
    //lap <<   1, -1,  0,  0,  0,
            //-1,  3, -1,  0, -1,
            // 0, -1,  3, -1, -1,
            // 0,  0, -1,  2, -1,
            // 0, -1, -1, -1,  3;

    Eigen::VectorXd theta(robot_num);
    Eigen::VectorXd theta_(robot_num);
    Eigen::VectorXd x_pos(robot_num);
    Eigen::VectorXd x_pos_(robot_num);



    /////获取当前机器人的位姿 (x, y, theta // 弧度)
    swarm_robot.getRobotPose(current_robot_pose);

    for(int i=0 ;i<robot_num; i++)
    {
        /////初始化
        theta(i) = current_robot_pose[i][2];
        x_pos(i) = current_robot_pose[i][0];
    }

    while(!is_conv)
    {

        // 根据拉普拉斯矩阵更新位置
        theta_ = - lap*theta;
        x_pos_ = - lap*x_pos;

        std::vector<std::vector<double>> velc;

        for(int i=0 ; i<robot_num; i++)
        {
            std::vector<double> vec(2,0.0);
            // 确定角速度
            float w = theta_(i)*lamda;
            // 根据角速度的大小确定x方向的速度，角速度大则x方向速度小
            float vx = x_pos_(i)*lamda2;

            vx = swarm_robot.checkVel(vx, MAX_V, MIN_V);

            vec[0] = vx; vec[1] = w; 

            std::cout << "robot_"<< current_robot_id[i]<< " v w: " << vx << "  "<< w <<"   ";
            // 传入速度更新位置
            velc.push_back(vec);
        }

        swarm_robot.moveRobot(velc);

        ros::Duration(time_span*100.0/1000.0).sleep();


        ///判断是否达到收敛条件///
        is_conv = true;
        for(int i=0; i< robot_num; i++)
        {
            if(std::abs(theta_(i)) > conv_th || std::abs(x_pos_(i)) > conv_linear)
            {
                is_conv = false;
            }
        }

        ///获取当前各机器人的位置///
        swarm_robot.getRobotPose(current_robot_pose);
        for(int i=0 ;i<robot_num; i++)
        {
            theta(i) = current_robot_pose[i][2];
            x_pos(i) = current_robot_pose[i][0];
        }
    }



    swarm_robot.stopRobot();

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
