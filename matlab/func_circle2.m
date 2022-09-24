function J = func_circle(x, M, N, K, v_theta_exp, gamma, dt)

%%  minimum jerk
J1 = 0;
for i = 1: K %所有时刻
    for j = 1:N %所有无人机
        J1 = J1+x((i-1)*N*M+(j-1)*M+4)^2;
    end
end
J1 = gamma(1)*J1;

%% 速度接近指定速度
J2 = 0;
for i = 1: K %所有时刻
    for j = 1:N %所有无人机
        J2 = J2+(x((i-1)*N*M+(j-1)*M+2)-v_theta_exp)^2;
    end
end
J2 = gamma(2)*J2;

%% 相机角度接近指定角度
J3 = 0;
% tic
for i = 1: K %所有时刻
    for j = 1:N %所有无人机
        for k = 1:N %每个无人机都有另外几个无人机作为目标
            if k==j
                continue;%无人机不看自己
            end
            alpha_exp = atan2(sin(x((i-1)*N*M+(k-1)*M+1))-sin(x((i-1)*N*M+(j-1)*M+1)), cos(x((i-1)*N*M+(k-1)*M+1))-cos(x((i-1)*N*M+(j-1)*M+1)));
            J3 = J3+x((i-1)*N*M+(j-1)*M+6+k)*(x((i-1)*N*M+(j-1)*M+5)-alpha_exp)^2;
        end
    end
end
J3 = gamma(3)*J3;

%% 相机速度接近指定速度
J4 = 0;
for i = 1: K %所有时刻
    for j = 1:N %所有无人机
       J4 = J4+x((i-1)*N*M+(j-1)*M+6)^2;
    end
end
J4 = gamma(4)*J4;
%% 代价求和
J = J1+J2+J3+J4;

end



