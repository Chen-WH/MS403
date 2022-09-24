%% 仿真，基于优化函数，模拟n架无人机画圆时的运动规划和相机运动规划
tic
%% 基本定义
N=4;%无人机数量
theta0 = linspace(0, 2*pi, N+1); % 无人机初始位置,单位：rad：在圆周上, 1号飞机从相位为0的地方开始，逆时针分布
theta0 = theta0(1:end-1); %只取N个值
alpha0 = zeros(1,N); %相机初始位置，0，相当于机头朝向正前方,单位: rad
T_exp = 6; %无人机的期望的运动周期，单位：s
v_theta_exp = 2*pi/T_exp; %无人机期望运动角速度，单位：rad/s
v_theta_max = 4*v_theta_exp; %允许无人机最大运动角速度，单位：rad/s
theta_inter_min = 2*pi/N*0.7; %允许相邻两架无人机之间允许的最小间隔，=均布间隔*系数，用于防撞
alpha_max = 300/360*pi; %相机角度上限，单位：rad；
alpha_min = -alpha_max; %相机角度下限，单位：rad；
v_alpha_max = 2*pi/3; %相机运动角速度上限，单位：rad/s；
v_alpha_min = -v_alpha_max; %相机运动角速度下限，单位：rad/s；
T_sim = 3; %仿真时长，单位：s，每次滑窗时间
T_total = T_exp*5; %总仿真时间,需要无人机转几圈，单位：s
T_step  = 2; %滑窗的步长
dt = 1/10; %采样时间间隔，单位：s
K=T_sim/dt+1; %滑窗序列长度，+1表示从时刻0开始计算
K_step = T_step/dt+1; %每次移动的窗长
M = 10; %每个无人机的状态量的数量
% 每个无人机有M个状态量：x_i = [theta, d_theta, dd_theta, ddd_theta, alpha, d_alpha, t1, t2, t3, t4]
% 变量含义：无人机位置，速度，加速度，跃度， 相机角度，速度，对另外N-1个agent是看哪一个
% 总共有N架无人机，序列长度为K，则共有KMN个优化量，即x包含KMN个变量，从上往下代表不同无人机的状态, 不同时刻的变量；
gamma = [1,1,1,1]; %优化权重,依次为，jerk，无人机速度，相机角度，相机角速度


%% 优化变量定义

%给定初始值
x0_start = zeros(M*N, 1);%初始时刻的状态
for i=1:N %每个无人机
    x0_start(M*(i-1)+1) = theta0(i); %在圆周上的起始相位
    x0_start(M*(i-1)+6+mod(i,N)+1) = 1;%盯着哪一个无人机看，初始定为依次看下一个
end
x0 = [];
for i = 1:K %所有时刻
    x0 = [x0; x0_start];
end

%% 变量范围约束
lb_single = [0 ; 0; -Inf; -Inf; alpha_min; v_alpha_min; 0; 0; 0; 0]; % 对单个无人机的约束下界
ub_single = [Inf ; v_theta_max; Inf; Inf; alpha_max; v_alpha_max; 1; 1; 1; 1]; % 对单个无人机的约束上界
lb_all = [];
ub_all = [];
for i = 1:N %每个无人机
    lb_all = [lb_all; lb_single];
    ub_all = [ub_all; ub_single];
end
lb = [];
ub = [];
for i = 1:K %所有时刻
    lb = [lb; lb_all];
    ub = [ub; ub_all];
end

%% 线性不等式约束，无人机相位之间的关系，避撞
A_all = zeros(M*N, M*N);
b_all = zeros(M*N, 1);
for i = 1:N %每个无人机
    if i==N %第n架无人机与第一架无人机相对，特殊处理
      b_all(1+M*(i-1)) = 2*pi-theta_inter_min;
      A_all(1+M*(i-1), 1+M*(i-1)) = 1;
      A_all(1+M*(i-1), 1) = -1;
    else
      b_all(1+M*(i-1)) = -theta_inter_min;
      A_all(1+M*(i-1), 1+M*(i-1)) = 1;
      A_all(1+M*(i-1), 1+M*(i)) = -1;
    end
end
b = [];
A = [];
for i = 1:K %所有时刻
    b = [b; b_all];
    A = blkdiag(A, A_all);
end
A(all(A==0,2),:) = [];
b(all(b==0,2),:) = [];

%% 线性等式约束，无人机状态转移方程+变量约束
Aeq = [];
beq = [];
for i = 1:K
    for j = 1:N
        if i==1%特殊情况，初始时刻，取值固定
            %无人机角度，角速度，角加速度的等式约束
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+1) = 1;
            Aeq = [Aeq; A_part];
            beq = [beq; theta0(j);];
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+2) = 1;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+3) = 1;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            %相机角度约束
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+5) = 1;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            %目标选择约束
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+6+j) = 1;%自己看不到自己
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+6+1) = 1;%看其中一架飞机
            A_part((i-1)*M*N+(j-1)*M+6+2) = 1;
            A_part((i-1)*M*N+(j-1)*M+6+3) = 1;
            A_part((i-1)*M*N+(j-1)*M+6+4) = 1;
            A_part((i-1)*M*N+(j-1)*M+6+j) = 0;
            Aeq = [Aeq; A_part];
            beq = [beq; 1];
        else
            %无人机角度，角速度，角加速度的等式约束
            A_part = zeros(1, K*M*N);%角度
            A_part((i-1)*M*N+(j-1)*M+1) = -1;
            A_part((i-2)*M*N+(j-1)*M+1) = 1;
            A_part((i-2)*M*N+(j-1)*M+2) = dt;
            A_part((i-2)*M*N+(j-1)*M+3) = 0.5*dt*dt;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            A_part = zeros(1, K*M*N);%速度
            A_part((i-1)*M*N+(j-1)*M+2) = -1;
            A_part((i-2)*M*N+(j-1)*M+2) = 1;
            A_part((i-2)*M*N+(j-1)*M+3) = dt;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            A_part = zeros(1, K*M*N);%角加速度
            A_part((i-1)*M*N+(j-1)*M+3) = -1;
            A_part((i-2)*M*N+(j-1)*M+3) = 1;
            A_part((i-2)*M*N+(j-1)*M+4) = dt;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            %相机角度约束
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+5) = -1;
            A_part((i-2)*M*N+(j-1)*M+5) = 1;
            A_part((i-2)*M*N+(j-1)*M+6) = dt;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            %目标选择约束
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+6+j) = 1;%自己看不到自己
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+6+1) = 1;%看其中一架飞机
            A_part((i-1)*M*N+(j-1)*M+6+2) = 1;
            A_part((i-1)*M*N+(j-1)*M+6+3) = 1;
            A_part((i-1)*M*N+(j-1)*M+6+4) = 1;
            A_part((i-1)*M*N+(j-1)*M+6+j) = 0;
            Aeq = [Aeq; A_part];
            beq = [beq; 1];
        end
    end
end
disp("init time = ");
toc

%%
T_now = 0;
result = [];
smooth_result = [];
options = optimoptions(@fmincon,'Algorithm','sqp','MaxFunctionEvaluations',10000);
% x0 = zeros(M*N*K,1);
% % options = optimoptions(@fmincon,'Algorithm','sqp','MaxIterations',1000);
% x = fmincon(@(x)func_circle(x, x0_start, M, N, K, v_theta_exp, gamma, dt), x0, A, b, Aeq, beq, lb, ub, [], options);
% tmp = reshape(x, M*N, K);
% tmp = [tmp; T_now:dt:(T_now+T_sim)];
% result = [result, tmp];

while T_now <T_total
    % 调用x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options) 
    tic
    x = fmincon(@(x)func_circle2(x, M, N, K, v_theta_exp, gamma, dt), x0, A, b, Aeq, beq, lb, ub, [], options);
    tmp = reshape(x, M*N, K);
    tmp = [tmp; T_now:dt:(T_now+T_sim)];
    result = [result, tmp];
    smooth_result = [smooth_result, tmp(:, 1:T_step/dt)];
    T_now = T_now + T_step
    x0_start = x(T_step/dt*M*N+1 : T_step/dt*M*N+M*N);
    %重新给定等式约束的初值
    for j = 1:N
        %无人机角度，角速度，角加速度的等式约束
        beq((j-1)*6+1) = x0_start((j-1)*M+1);%数字6源于上面的等式约束中迭代时一个批次的量
        beq((j-1)*6+2) = x0_start((j-1)*M+2);
        beq((j-1)*6+3) = x0_start((j-1)*M+3);    
        %相机角度约束
        beq((j-1)*6+4) = x0_start((j-1)*M+5);
        %目标选择约束不变
    end   
    x0 = zeros(M*N*K,1);
    %x0 = [x(T_step/dt*M*N+1 : end) ; zeros((T_step/dt)*M*N, 1) ];
    toc
end
disp("opt time = ");
toc

