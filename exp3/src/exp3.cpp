/* 
 * Date: 2021-12-16
 * Description: Formation change through obstacles
 * 增加机器人数量只需修改编号，分布式队形选择
 */

#include <swarm_robot_control.h>
#include <cmath>
#include <algorithm>

#define pi 3.1415926

/***********add by group*********/
/* 集群初始化 */
std::vector<int> swarm_robot_id{1, 2, 3, 4, 5, 6};   // 集群机器人编号
std::vector<int> obstacle_id{9, 10};                       // 障碍机器人编号
/* 基本参数设定 */
double k_w = 0.5;               // 角速度系数
double k_v = 0.1;               // 线速度系数
double MAX_W = 1;               // 角速度最大值
double MIN_W = 0.05;            // 角速度最小值
double MAX_V = 0.2;             // 线速度最大值
double MIN_V = 0.01;            // 线速度最小值
double Delta = 2.5;             // 预设通讯范围
Eigen::MatrixXd lap(swarm_robot_id.size(), swarm_robot_id.size());  // 拉普拉斯矩阵
Eigen::MatrixXd L_x(swarm_robot_id.size(), swarm_robot_id.size());  // 连通保持x矩阵
Eigen::MatrixXd L_y(swarm_robot_id.size(), swarm_robot_id.size());  // 连通保持y矩阵
std::vector<std::vector<double> > current_robot_pose(swarm_robot_id.size());
Eigen::VectorXd cur_x(swarm_robot_id.size());    // 机器人当前x方向位置
Eigen::VectorXd cur_y(swarm_robot_id.size());    // 机器人当前y方向位置
Eigen::VectorXd cur_theta(swarm_robot_id.size());// 机器人当前偏航角
Eigen::VectorXd del_x(swarm_robot_id.size());
Eigen::VectorXd del_y(swarm_robot_id.size());
Eigen::VectorXd del_theta(swarm_robot_id.size());
/* 队形变换参数设置 */
int formationA_type = 1;            // 行进编队种类
int formationB_type = 1;            // 避障编队种类
double k_zeta = 0.2;                // 整体编队代价估计量一致性系数
double k_xi = 0.05;                 // 局部编队代价实时量系数
double characteristic_A = 0.5;      // 行进编队特征长度
double characteristic_B;            // 避障编队特征长度，自主根据障碍物宽度进行调整
double threshold = 0.16;            // 避障阈值
double Formation_centor[2];         // 队形中心位置
Eigen::VectorXi target_formation(swarm_robot_id.size());     // 各机器人选择编队序号
Eigen::VectorXd pic_x(swarm_robot_id.size());    // pic_x = cur_x - Formation_x
Eigen::VectorXd pic_y(swarm_robot_id.size());    // pic_y = cur_y - Formation_y

/* PID-control yaw */
void yaw_control(SwarmRobot& swarm, int index, double yaw_error){
    double w = swarm.checkVel(k_w*yaw_error, MAX_W, MIN_W);
    swarm.moveRobot(index, 0.0, w);
}

/* PID 位置控制 */
void pos_control(SwarmRobot& swarm_robot, int index, double x_error, double y_error, double yaw){
    double w = atan(y_error/x_error);
    double v = sqrt(x_error*x_error + y_error*y_error);
    if (x_error < 0){
        if (y_error > 0){
            w += pi;
        }
        else {
            w -= pi;
        }
    }
    w = w - yaw;
    v *= pow(cos(w), 3);
    v *= k_v;
    w *= k_w;
    v = swarm_robot.checkVel(v, MAX_V, MIN_V);
    w = swarm_robot.checkVel(w, MAX_W, MIN_W);
    swarm_robot.moveRobot(index, v, w);
}
/* 获取各机器人位置 */
void get_pos(SwarmRobot& swarm_robot){
    swarm_robot.getRobotPose(current_robot_pose);
    Formation_centor[0] = Formation_centor[1] = 0;
    for(int i = 0; i < swarm_robot_id.size(); i++) {
        cur_x(i) = current_robot_pose[i][0];
        Formation_centor[0] += cur_x(i);
        cur_y(i) = current_robot_pose[i][1];
        Formation_centor[1] += cur_y(i);
        cur_theta(i) = current_robot_pose[i][2];
    }
    Formation_centor[0] /= swarm_robot_id.size();
    Formation_centor[1] /= swarm_robot_id.size();
}
/* 更新队形实时量 */
void update_formation_cost(const int formation_num, Eigen::MatrixXd& Formation_x, Eigen::MatrixXd& Formation_y, Eigen::MatrixXd& xi){
    for (int i = 0; i < formation_num; ++i){
        for (int j = 0; j < swarm_robot_id.size(); ++j){
            double tmp = 0;
            for (int k = 0; k < swarm_robot_id.size(); ++k){
                if (k == j){
                    continue;
                }
                tmp += sqrt(pow(cur_x(j) - cur_x(k) - Formation_x(j, i) + Formation_x(k, i), 2) + pow(cur_y(j) - cur_y(k) - Formation_y(j, i) + Formation_y(k, i), 2));
            }
            xi(j, i) = tmp;
        }
    }
}
/* 迭代队形估计量 */
void conv_formation_cost(const int formation_num, Eigen::MatrixXd& Formation_x, Eigen::MatrixXd& Formation_y, Eigen::MatrixXd& zeta, Eigen::MatrixXd& xi){
    update_formation_cost(formation_num, Formation_x, Formation_y, xi);
    for (int i = 0; i < formation_num; ++i){
        zeta.block(0, i, swarm_robot_id.size(), 1) += k_xi*(xi.block(0, i, swarm_robot_id.size(), 1) - zeta.block(0, i, swarm_robot_id.size(), 1)) - k_zeta*lap*zeta.block(0, i, swarm_robot_id.size(), 1);
    }
}
/* 队形选择 */
void formation_selection(const int formation_num, Eigen::MatrixXd& zeta){
    for (int i = 0; i < swarm_robot_id.size(); ++i){
        double min_cost = 1e+9;
        for (int j = 0; j < formation_num; ++j){
            if (zeta(i, j) < min_cost){
                target_formation(i) = j;
                min_cost = zeta(i, j);
            }
        }
    }
    std::cout << "当前各机器人队形选择为： ";
    for (int i =0; i < swarm_robot_id.size(); ++i){
        std::cout << target_formation[0] << " ";
    }
    std::cout << std::endl;
}
/* 队形控制量 */
void cal_formation(const int formation_num, Eigen::MatrixXd& Formation_x, Eigen::MatrixXd& Formation_y){
    for (int i = 0; i < swarm_robot_id.size(); ++i){
        pic_x(i) = cur_x(i) - Formation_x(i, target_formation(i));
        pic_y(i) = cur_y(i) - Formation_y(i, target_formation(i));
    }
    for (int i = 0; i < swarm_robot_id.size(); ++i){
        for (int j = 0; j < swarm_robot_id.size(); ++j){
            if (j == i){
                continue;
            }
            L_x(i, j) = L_x(j, i) = -2*((Delta - std::fabs(Formation_x(i, target_formation(i)) - Formation_x(j, target_formation(j)))) - std::fabs(pic_x(i) - pic_x(j)))/\
            pow((Delta - std::fabs(Formation_x(i, target_formation(i)) - Formation_x(j, target_formation(j))) - std::fabs(pic_x(i) - pic_x(j))), 2);
        }
    }
    for (int i = 0; i < swarm_robot_id.size(); ++i){
        for (int j = 0; j < swarm_robot_id.size(); ++j){
            if (j == i){
                continue;
            }
            L_y(i, j) = L_y(j, i) = -2*((Delta - std::fabs(Formation_y(i, target_formation(i)) - Formation_y(j, target_formation(j)))) - std::fabs(pic_y(i) - pic_y(j)))/\
            pow((Delta - std::fabs(Formation_y(i, target_formation(i)) - Formation_y(j, target_formation(j))) - std::fabs(pic_y(i) - pic_y(j))), 2);
        }
    }
    for (int i = 0; i < swarm_robot_id.size(); ++i){
        double tmp1 = 0;
        double tmp2 = 0;
        for (int j = 0; j < swarm_robot_id.size(); ++j){
            if (j == i){
                continue;
            }
            tmp1 -= L_x(i, j);
            tmp2 -= L_y(i, j);
        }
        L_x(i, i) = tmp1;
        L_y(i, i) = tmp2;
    }
    del_x = -L_x * pic_x;
    del_y = -L_y * pic_y;
}
int main(int argc, char** argv) {
    /* 节点初始化 */
    ros::init(argc, argv, "swarm_robot_control_formation");
    ros::NodeHandle nh;
    SwarmRobot swarm_robot(&nh, swarm_robot_id);
    SwarmRobot obstacle(&nh, obstacle_id);
    std::vector<std::vector<double> > obstacle_pose(obstacle_id.size());
    for (int i = 0; i < swarm_robot_id.size(); ++i){
        for (int j = 0; j < swarm_robot_id.size(); ++j){
            lap(i, j) = -1;
        }
    }
    for (int i = 0; i < swarm_robot_id.size(); ++i){
        lap(i, i) = swarm_robot_id.size() - 1;
    }
    
    /* 基本参数设定 */
    int array_flag[swarm_robot_id.size()];                   // 角色分配标记
    Eigen::MatrixXd circle(swarm_robot_id.size(), 2);        // 圆形编队相对位置 [x y]
    Eigen::MatrixXd column(swarm_robot_id.size(), 2);        // 两路纵队相对位置 [x y]
    double obstacle_centor[2];                               // 障碍物中心位置
    int role_assignment = 1;                                 // 角色分配种类
    for (int i = 0; i < swarm_robot_id.size(); ++i){
        role_assignment *= (i + 1);
        array_flag[i] = i;
    }
    int formationA_num = formationA_type*role_assignment;
    int formationB_num = formationB_type*role_assignment;
    double conv_th = 0.03*swarm_robot_id.size();             // 角度收敛阈值
    double conv_pos = 0.03*swarm_robot_id.size();            // 位置收敛阈值
    bool angle_conv = false;           // 角度收敛标记
    bool pos_conv = false;             // 位置收敛标记

    /* 获取障碍物信息 */
    obstacle.getRobotPose(obstacle_pose);
    obstacle_centor[0] = (obstacle_pose[0][0] + obstacle_pose[1][0])/2;
    obstacle_centor[1] = (obstacle_pose[0][1] + obstacle_pose[1][1])/2;
    characteristic_B = std::fabs(obstacle_pose[0][1] - obstacle_pose[1][1])/2 - threshold - 0.1;

    /* 队形信息设定 */
    for (int i = 0; i < swarm_robot_id.size(); ++i){
        circle(i, 0) = characteristic_A*cos(i*2*pi/swarm_robot_id.size());
        circle(i, 1) = characteristic_A*sin(i*2*pi/swarm_robot_id.size());
    }
    for (int i = 0; i < swarm_robot_id.size(); ++i){
        column(i, 0) = characteristic_B*pow(-1, i)*(1 + 2*int(i/4));
        column(i, 1) = characteristic_B*pow(-1, int(i/2));
    }
    Eigen::MatrixXd FormationA_x(swarm_robot_id.size(), formationA_num);
    Eigen::MatrixXd FormationA_y(swarm_robot_id.size(), formationA_num);
    Eigen::MatrixXd FormationB_x(swarm_robot_id.size(), formationB_num);
    Eigen::MatrixXd FormationB_y(swarm_robot_id.size(), formationB_num);
    Eigen::MatrixXd zeta_A(swarm_robot_id.size(), formationA_num);  // 队形代价估计数值
    Eigen::MatrixXd xi_A(swarm_robot_id.size(), formationA_num);    // 队形代价实时数值
    Eigen::MatrixXd zeta_B(swarm_robot_id.size(), formationB_num);  // 队形代价估计数值
    Eigen::MatrixXd xi_B(swarm_robot_id.size(), formationB_num);    // 队形代价实时数值
    for (int i = 0; i < role_assignment; ++i){
        for (int j = 0; j < swarm_robot_id.size(); ++j){
            FormationA_x(j, i) = circle(array_flag[j], 0) - circle(array_flag[0], 0);
            FormationA_y(j, i) = circle(array_flag[j], 1) - circle(array_flag[0], 1);
            FormationB_x(j, i) = column(array_flag[j], 0) - column(array_flag[0], 0);
            FormationB_y(j, i) = column(array_flag[j], 1) - column(array_flag[0], 1);
        }
        std::next_permutation(array_flag, array_flag + swarm_robot_id.size());
    }

    /* 形成 A 队形 */
    get_pos(swarm_robot);
    update_formation_cost(formationA_num, FormationA_x, FormationA_y, xi_A);
    zeta_A = xi_A;
    while(! pos_conv) {
        get_pos(swarm_robot);
        conv_formation_cost(formationA_num, FormationA_x, FormationA_y, zeta_A, xi_A);
        formation_selection(formationA_num, zeta_A);
        cal_formation(formationA_num, FormationA_x, FormationA_y);
        pos_conv = true;
        if(std::fabs(del_x.norm()) > conv_pos || std::fabs(del_y.norm()) > conv_pos) {
            pos_conv = false;
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            pos_control(swarm_robot, i, del_x(i), del_y(i), cur_theta(i));
        }
        ros::Duration(0.05).sleep();
    }
    while(!angle_conv){
        get_pos(swarm_robot);
        cal_formation(formationA_num, FormationA_x, FormationA_y);
        angle_conv = true;
        if(std::fabs(cur_theta.norm()) > conv_th) {
            angle_conv = false;
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            yaw_control(swarm_robot, i, -cur_theta(i));
        }
    }
    ROS_INFO_STREAM("Formation A got!");
    pos_conv = false;
    angle_conv = false;

    /* 靠近障碍物 */
    get_pos(swarm_robot);
    update_formation_cost(formationA_num, FormationA_x, FormationA_y, xi_A);
    zeta_A = xi_A;
    while(! pos_conv) {
        get_pos(swarm_robot);
        conv_formation_cost(formationA_num, FormationA_x, FormationA_y, zeta_A, xi_A);
        formation_selection(formationA_num, zeta_A);
        cal_formation(formationA_num, FormationA_x, FormationA_y);
        double error_x = obstacle_centor[0] - Formation_centor[0] - 2*characteristic_A;
        double error_y = obstacle_centor[1] - Formation_centor[1];
        for (int i = 0; i < swarm_robot_id.size(); ++i){
            del_x(i) += error_x;
            del_y(i) += error_y;
        }
        pos_conv = true;
        if(std::fabs(del_x.norm()) > conv_pos || std::fabs(del_y.norm()) > conv_pos) {
            pos_conv = false;
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            pos_control(swarm_robot, i, del_x(i), del_y(i), cur_theta(i));
        }
        ros::Duration(0.05).sleep();
    }
    while(!angle_conv){
        get_pos(swarm_robot);
        cal_formation(formationA_num, FormationA_x, FormationA_y);
        angle_conv = true;
        if(std::fabs(cur_theta.norm()) > conv_th) {
            angle_conv = false;
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            yaw_control(swarm_robot, i, -cur_theta(i));
        }
    }
    ROS_INFO_STREAM("Warning, obstacle!");
    pos_conv = false;
    angle_conv = false;

    /* 形成 B 队形 */
    get_pos(swarm_robot);
    update_formation_cost(formationB_num, FormationB_x, FormationB_y, xi_B);
    zeta_B = xi_B;
    while(! pos_conv) {
        get_pos(swarm_robot);
        conv_formation_cost(formationB_num, FormationB_x, FormationB_y, zeta_B, xi_B);
        formation_selection(formationB_num, zeta_B);
        cal_formation(formationB_num, FormationB_x, FormationB_y);
        double error_x = obstacle_centor[0] - Formation_centor[0] - 2*characteristic_A;
        double error_y = obstacle_centor[1] - Formation_centor[1];
        for (int i = 0; i < swarm_robot_id.size(); ++i){
            del_x(i) += error_x;
            del_y(i) += error_y;
        }
        pos_conv = true;
        if(std::fabs(del_x.norm()) > conv_pos || std::fabs(del_y.norm()) > conv_pos) {
            pos_conv = false;
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            pos_control(swarm_robot, i, del_x(i), del_y(i), cur_theta(i));
        }
        ros::Duration(0.05).sleep();
    }
    while(!angle_conv){
        get_pos(swarm_robot);
        cal_formation(formationB_num, FormationB_x, FormationB_y);
        angle_conv = true;
        if(std::fabs(cur_theta.norm()) > conv_th) {
            angle_conv = false;
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            yaw_control(swarm_robot, i, -cur_theta(i));
        }
    }
    ROS_INFO_STREAM("Formation B got!");
    pos_conv = false;
    angle_conv = false;

    /* 穿过障碍 */
    get_pos(swarm_robot);
    update_formation_cost(formationB_num, FormationB_x, FormationB_y, xi_B);
    zeta_B = xi_B;
    while(! pos_conv) {
        get_pos(swarm_robot);
        conv_formation_cost(formationB_num, FormationB_x, FormationB_y, zeta_B, xi_B);
        formation_selection(formationB_num, zeta_B);
        cal_formation(formationB_num, FormationB_x, FormationB_y);
        double error_x = obstacle_centor[0] - Formation_centor[0] + 2*characteristic_A;
        double error_y = obstacle_centor[1] - Formation_centor[1];
        for (int i = 0; i < swarm_robot_id.size(); ++i){
            del_x(i) += error_x;
            del_y(i) += error_y;
        }
        pos_conv = true;
        if(std::fabs(del_x.norm()) > conv_pos || std::fabs(del_y.norm()) > conv_pos) {
            pos_conv = false;
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            pos_control(swarm_robot, i, del_x(i), del_y(i), cur_theta(i));
        }
        ros::Duration(0.05).sleep();
    }
    while(!angle_conv){
        get_pos(swarm_robot);
        cal_formation(formationB_num, FormationB_x, FormationB_y);
        angle_conv = true;
        if(std::fabs(cur_theta.norm()) > conv_th) {
            angle_conv = false;
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            yaw_control(swarm_robot, i, -cur_theta(i));
        }
    }
    ROS_INFO_STREAM("Successfully penetrated!");
    pos_conv = false;
    angle_conv = false;

    /* 形成 A 队形 */
    get_pos(swarm_robot);
    update_formation_cost(formationA_num, FormationA_x, FormationA_y, xi_A);
    zeta_A = xi_A;
    while(! pos_conv) {
        get_pos(swarm_robot);
        conv_formation_cost(formationA_num, FormationA_x, FormationA_y, zeta_A, xi_A);
        formation_selection(formationA_num, zeta_A);
        cal_formation(formationA_num, FormationA_x, FormationA_y);
        double error_x = obstacle_centor[0] - Formation_centor[0] + 2*characteristic_A;
        double error_y = obstacle_centor[1] - Formation_centor[1];
        for (int i = 0; i < swarm_robot_id.size(); ++i){
            del_x(i) += error_x;
            del_y(i) += error_y;
        }
        pos_conv = true;
        if(std::fabs(del_x.norm()) > conv_pos || std::fabs(del_y.norm()) > conv_pos) {
            pos_conv = false;
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            pos_control(swarm_robot, i, del_x(i), del_y(i), cur_theta(i));
        }
        ros::Duration(0.05).sleep();
    }
    while(!angle_conv){
        get_pos(swarm_robot);
        cal_formation(formationA_num, FormationA_x, FormationA_y);
        angle_conv = true;
        if(std::fabs(cur_theta.norm()) > conv_th) {
            angle_conv = false;
        }
        for(int i = 0; i < swarm_robot_id.size(); i++) {
            yaw_control(swarm_robot, i, -cur_theta(i));
        }
    }
    ROS_INFO_STREAM("Re-form the A formation!");

    /* 程序结束，停止机器人 */
    swarm_robot.stopRobot();
    ROS_INFO_STREAM("Succeed!");
    return 0;
}