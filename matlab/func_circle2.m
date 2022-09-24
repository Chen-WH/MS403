function J = func_circle(x, M, N, K, v_theta_exp, gamma, dt)

%%  minimum jerk
J1 = 0;
for i = 1: K %����ʱ��
    for j = 1:N %�������˻�
        J1 = J1+x((i-1)*N*M+(j-1)*M+4)^2;
    end
end
J1 = gamma(1)*J1;

%% �ٶȽӽ�ָ���ٶ�
J2 = 0;
for i = 1: K %����ʱ��
    for j = 1:N %�������˻�
        J2 = J2+(x((i-1)*N*M+(j-1)*M+2)-v_theta_exp)^2;
    end
end
J2 = gamma(2)*J2;

%% ����ǶȽӽ�ָ���Ƕ�
J3 = 0;
% tic
for i = 1: K %����ʱ��
    for j = 1:N %�������˻�
        for k = 1:N %ÿ�����˻��������⼸�����˻���ΪĿ��
            if k==j
                continue;%���˻������Լ�
            end
            alpha_exp = atan2(sin(x((i-1)*N*M+(k-1)*M+1))-sin(x((i-1)*N*M+(j-1)*M+1)), cos(x((i-1)*N*M+(k-1)*M+1))-cos(x((i-1)*N*M+(j-1)*M+1)));
            J3 = J3+x((i-1)*N*M+(j-1)*M+6+k)*(x((i-1)*N*M+(j-1)*M+5)-alpha_exp)^2;
        end
    end
end
J3 = gamma(3)*J3;

%% ����ٶȽӽ�ָ���ٶ�
J4 = 0;
for i = 1: K %����ʱ��
    for j = 1:N %�������˻�
       J4 = J4+x((i-1)*N*M+(j-1)*M+6)^2;
    end
end
J4 = gamma(4)*J4;
%% �������
J = J1+J2+J3+J4;

end



