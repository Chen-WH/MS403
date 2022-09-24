%% ���棬�����Ż�������ģ��n�����˻���Բʱ���˶��滮������˶��滮
tic
%% ��������
N=4;%���˻�����
theta0 = linspace(0, 2*pi, N+1); % ���˻���ʼλ��,��λ��rad����Բ����, 1�ŷɻ�����λΪ0�ĵط���ʼ����ʱ��ֲ�
theta0 = theta0(1:end-1); %ֻȡN��ֵ
alpha0 = zeros(1,N); %�����ʼλ�ã�0���൱�ڻ�ͷ������ǰ��,��λ: rad
T_exp = 6; %���˻����������˶����ڣ���λ��s
v_theta_exp = 2*pi/T_exp; %���˻������˶����ٶȣ���λ��rad/s
v_theta_max = 4*v_theta_exp; %�������˻�����˶����ٶȣ���λ��rad/s
theta_inter_min = 2*pi/N*0.7; %���������������˻�֮���������С�����=�������*ϵ�������ڷ�ײ
alpha_max = 300/360*pi; %����Ƕ����ޣ���λ��rad��
alpha_min = -alpha_max; %����Ƕ����ޣ���λ��rad��
v_alpha_max = 2*pi/3; %����˶����ٶ����ޣ���λ��rad/s��
v_alpha_min = -v_alpha_max; %����˶����ٶ����ޣ���λ��rad/s��
T_sim = 3; %����ʱ������λ��s��ÿ�λ���ʱ��
T_total = T_exp*5; %�ܷ���ʱ��,��Ҫ���˻�ת��Ȧ����λ��s
T_step  = 2; %�����Ĳ���
dt = 1/10; %����ʱ��������λ��s
K=T_sim/dt+1; %�������г��ȣ�+1��ʾ��ʱ��0��ʼ����
K_step = T_step/dt+1; %ÿ���ƶ��Ĵ���
M = 10; %ÿ�����˻���״̬��������
% ÿ�����˻���M��״̬����x_i = [theta, d_theta, dd_theta, ddd_theta, alpha, d_alpha, t1, t2, t3, t4]
% �������壺���˻�λ�ã��ٶȣ����ٶȣ�Ծ�ȣ� ����Ƕȣ��ٶȣ�������N-1��agent�ǿ���һ��
% �ܹ���N�����˻������г���ΪK������KMN���Ż�������x����KMN���������������´���ͬ���˻���״̬, ��ͬʱ�̵ı�����
gamma = [1,1,1,1]; %�Ż�Ȩ��,����Ϊ��jerk�����˻��ٶȣ�����Ƕȣ�������ٶ�


%% �Ż���������

%������ʼֵ
x0_start = zeros(M*N, 1);%��ʼʱ�̵�״̬
for i=1:N %ÿ�����˻�
    x0_start(M*(i-1)+1) = theta0(i); %��Բ���ϵ���ʼ��λ
    x0_start(M*(i-1)+6+mod(i,N)+1) = 1;%������һ�����˻�������ʼ��Ϊ���ο���һ��
end
x0 = [];
for i = 1:K %����ʱ��
    x0 = [x0; x0_start];
end

%% ������ΧԼ��
lb_single = [0 ; 0; -Inf; -Inf; alpha_min; v_alpha_min; 0; 0; 0; 0]; % �Ե������˻���Լ���½�
ub_single = [Inf ; v_theta_max; Inf; Inf; alpha_max; v_alpha_max; 1; 1; 1; 1]; % �Ե������˻���Լ���Ͻ�
lb_all = [];
ub_all = [];
for i = 1:N %ÿ�����˻�
    lb_all = [lb_all; lb_single];
    ub_all = [ub_all; ub_single];
end
lb = [];
ub = [];
for i = 1:K %����ʱ��
    lb = [lb; lb_all];
    ub = [ub; ub_all];
end

%% ���Բ���ʽԼ�������˻���λ֮��Ĺ�ϵ����ײ
A_all = zeros(M*N, M*N);
b_all = zeros(M*N, 1);
for i = 1:N %ÿ�����˻�
    if i==N %��n�����˻����һ�����˻���ԣ����⴦��
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
for i = 1:K %����ʱ��
    b = [b; b_all];
    A = blkdiag(A, A_all);
end
A(all(A==0,2),:) = [];
b(all(b==0,2),:) = [];

%% ���Ե�ʽԼ�������˻�״̬ת�Ʒ���+����Լ��
Aeq = [];
beq = [];
for i = 1:K
    for j = 1:N
        if i==1%�����������ʼʱ�̣�ȡֵ�̶�
            %���˻��Ƕȣ����ٶȣ��Ǽ��ٶȵĵ�ʽԼ��
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
            %����Ƕ�Լ��
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+5) = 1;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            %Ŀ��ѡ��Լ��
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+6+j) = 1;%�Լ��������Լ�
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+6+1) = 1;%������һ�ܷɻ�
            A_part((i-1)*M*N+(j-1)*M+6+2) = 1;
            A_part((i-1)*M*N+(j-1)*M+6+3) = 1;
            A_part((i-1)*M*N+(j-1)*M+6+4) = 1;
            A_part((i-1)*M*N+(j-1)*M+6+j) = 0;
            Aeq = [Aeq; A_part];
            beq = [beq; 1];
        else
            %���˻��Ƕȣ����ٶȣ��Ǽ��ٶȵĵ�ʽԼ��
            A_part = zeros(1, K*M*N);%�Ƕ�
            A_part((i-1)*M*N+(j-1)*M+1) = -1;
            A_part((i-2)*M*N+(j-1)*M+1) = 1;
            A_part((i-2)*M*N+(j-1)*M+2) = dt;
            A_part((i-2)*M*N+(j-1)*M+3) = 0.5*dt*dt;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            A_part = zeros(1, K*M*N);%�ٶ�
            A_part((i-1)*M*N+(j-1)*M+2) = -1;
            A_part((i-2)*M*N+(j-1)*M+2) = 1;
            A_part((i-2)*M*N+(j-1)*M+3) = dt;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            A_part = zeros(1, K*M*N);%�Ǽ��ٶ�
            A_part((i-1)*M*N+(j-1)*M+3) = -1;
            A_part((i-2)*M*N+(j-1)*M+3) = 1;
            A_part((i-2)*M*N+(j-1)*M+4) = dt;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            %����Ƕ�Լ��
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+5) = -1;
            A_part((i-2)*M*N+(j-1)*M+5) = 1;
            A_part((i-2)*M*N+(j-1)*M+6) = dt;
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            %Ŀ��ѡ��Լ��
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+6+j) = 1;%�Լ��������Լ�
            Aeq = [Aeq; A_part];
            beq = [beq; 0];
            A_part = zeros(1, K*M*N);
            A_part((i-1)*M*N+(j-1)*M+6+1) = 1;%������һ�ܷɻ�
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
    % ����x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options) 
    tic
    x = fmincon(@(x)func_circle2(x, M, N, K, v_theta_exp, gamma, dt), x0, A, b, Aeq, beq, lb, ub, [], options);
    tmp = reshape(x, M*N, K);
    tmp = [tmp; T_now:dt:(T_now+T_sim)];
    result = [result, tmp];
    smooth_result = [smooth_result, tmp(:, 1:T_step/dt)];
    T_now = T_now + T_step
    x0_start = x(T_step/dt*M*N+1 : T_step/dt*M*N+M*N);
    %���¸�����ʽԼ���ĳ�ֵ
    for j = 1:N
        %���˻��Ƕȣ����ٶȣ��Ǽ��ٶȵĵ�ʽԼ��
        beq((j-1)*6+1) = x0_start((j-1)*M+1);%����6Դ������ĵ�ʽԼ���е���ʱһ�����ε���
        beq((j-1)*6+2) = x0_start((j-1)*M+2);
        beq((j-1)*6+3) = x0_start((j-1)*M+3);    
        %����Ƕ�Լ��
        beq((j-1)*6+4) = x0_start((j-1)*M+5);
        %Ŀ��ѡ��Լ������
    end   
    x0 = zeros(M*N*K,1);
    %x0 = [x(T_step/dt*M*N+1 : end) ; zeros((T_step/dt)*M*N, 1) ];
    toc
end
disp("opt time = ");
toc

