%%  连续的估计
fontsize = 16;
row_num = 2;
col_num = 4;
figure;
subplot(row_num, col_num, 1);
% 无人机角度
% figure;
hold on;
for i = 1:N
    plot(smooth_result(end, :), smooth_result((i-1)*M+1, :));
end
title('$\theta$', 'interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', fontsize);
%legend('UAV1','UAV2','UAV3','UAV4');
xlabel('Time (s)','FontName', 'Times New Roman', 'FontSize', fontsize);
set(gca, 'FontName', 'Times New Roman', 'FontSize', fontsize);
grid on;
% 无人机角速度
subplot(row_num, col_num, 2);
% figure;
hold on;
for i = 1:N
    plot(smooth_result(end, :), smooth_result((i-1)*M+2, :));
end
title('$\dot \theta$', 'interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', fontsize);
%legend('UAV1','UAV2','UAV3','UAV4');
xlabel('Time (s)','FontName', 'Times New Roman', 'FontSize', fontsize);
set(gca, 'FontName', 'Times New Roman', 'FontSize', fontsize);
grid on;
% 无人机角加速度
subplot(row_num, col_num, 3);
% figure;
hold on;
for i = 1:N
    plot(smooth_result(end, :), smooth_result((i-1)*M+3, :));
end
title('$\ddot \theta$', 'interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', fontsize);
%legend('UAV1','UAV2','UAV3','UAV4');
xlabel('Time (s)','FontName', 'Times New Roman', 'FontSize', fontsize);
set(gca, 'FontName', 'Times New Roman', 'FontSize', fontsize);
grid on;
% 无人机跃度
subplot(row_num, col_num, 4);
% figure;
hold on;
for i = 1:N
    plot(smooth_result(end, :), smooth_result((i-1)*M+4, :));
end
title('$\theta^{(3)}$', 'interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', fontsize);
%legend('UAV1','UAV2','UAV3','UAV4');
xlabel('Time (s)','FontName', 'Times New Roman', 'FontSize', fontsize);
set(gca, 'FontName', 'Times New Roman', 'FontSize', fontsize);
grid on;
% 相机角度
subplot(row_num, col_num, 5);
% figure;
hold on;
for i = 1:N
    plot(smooth_result(end, :), smooth_result((i-1)*M+5, :));
end
title('$\alpha$', 'interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', fontsize);
%legend('UAV1','UAV2','UAV3','UAV4');
xlabel('Time (s)','FontName', 'Times New Roman', 'FontSize', fontsize);
set(gca, 'FontName', 'Times New Roman', 'FontSize', fontsize);
grid on;
% 相机角速度
subplot(row_num, col_num, 6);
% figure;
hold on;
for i = 1:N
    plot(smooth_result(end, :), smooth_result((i-1)*M+6, :));
end
title('$\dot \alpha$', 'interpreter','latex', 'FontName', 'Times New Roman', 'FontSize', fontsize);
%legend('UAV1','UAV2','UAV3','UAV4');
xlabel('Time (s)','FontName', 'Times New Roman', 'FontSize', fontsize);
set(gca, 'FontName', 'Times New Roman', 'FontSize', fontsize);
grid on;

%% 目标选择的绘制+夹角变化
% 无人机夹角随着时间的变化
inter_angle = [];
for i = 1:N
    if i == N
        inter_angle = [inter_angle; smooth_result(1, :)+2*pi-smooth_result((i-1)*M+1, :)];
    else
        inter_angle = [inter_angle; smooth_result((i)*M+1, :)-smooth_result((i-1)*M+1, :)];
    end
end
subplot(row_num, col_num, 7);
hold on;
for i = 1:N
    plot(smooth_result(end, :), inter_angle(i, :));
end
title('Inter Angle', 'FontName', 'Times New Roman', 'FontSize', fontsize);
%legend('UAV1','UAV2','UAV3','UAV4');
xlabel('Time (s)','FontName', 'Times New Roman', 'FontSize', fontsize);
set(gca, 'FontName', 'Times New Roman', 'FontSize', fontsize);
grid on;

% 无人机目标随着时间的变化
target_id = [];
for i = 1:N
        target_id = [target_id; 1*smooth_result((i-1)*M+7, :)+2*smooth_result((i-1)*M+8, :)+3*smooth_result((i-1)*M+9, :)+4*smooth_result((i-1)*M+10, :)];
end

subplot(row_num, col_num, 8);
% figure;
hold on;
for i = 1:N
    plot(smooth_result(end, :), target_id(i, :));
end
title('Target ID', 'FontName', 'Times New Roman', 'FontSize', fontsize);
%legend('UAV1','UAV2','UAV3','UAV4');
xlabel('Time (s)','FontName', 'Times New Roman', 'FontSize', fontsize);
set(gca, 'FontName', 'Times New Roman', 'FontSize', fontsize);
grid on;