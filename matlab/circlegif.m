%% �������˻���Բ�����˶��Ķ�ͼ
% dt = 1/10; %�����ʱ����
frame_t = 1/10; %gif��֡���
savegif = true;
text_right = -0.15;
arrow_length = 1;
%�������˻���Բ�ϵ�λ��
pos = [];
angle = [];
half_range = 45/180*pi;
vnum = 6;
for i = 1:N
    pos = [pos; cos(smooth_result((i-1)*M+1,:)); sin(smooth_result((i-1)*M+1,:))];
    angle = [angle; cos(smooth_result((i-1)*M+5,:)); sin(smooth_result((i-1)*M+5,:));
                              cos(smooth_result((i-1)*M+5,:)-half_range); sin(smooth_result((i-1)*M+5,:)-half_range);
                              cos(smooth_result((i-1)*M+5,:)+half_range); sin(smooth_result((i-1)*M+5,:)+half_range)];
end
angle = angle*arrow_length;

%�����˶����������˻�λ��������۲�ǵĹ�ϵ������ȷ���Ƿ���ͨ
M_connect=[];%��ʾ��ͨ�Եľ���
for i = 1:N
    for j = 1:N
        if i==j%�Լ����ж��Ƿ���Լ���ͨ,ȫ��Ϊ0
            M_connect = [M_connect; 0*pos(1,:)];
        else
            pos_dif_vec = pos((2*(j-1)+1):2*j, :)-pos((2*(i-1)+1):2*i, :);%�����˻�λ��������i��j
            vec_x = dot(pos_dif_vec, angle((vnum*(i-1)+1):(vnum*(i-1)+2),:));%���˻�i������۲�Ǻ�λ���������ڻ�
            cos_vec = vec_x./vecnorm(pos_dif_vec);% 
            M_connect = [M_connect; acos(cos_vec)<half_range];
        end    
    end
end

%����Բ��ͼ��
figure;
rectangle('position',[0-1,0-1,1*2,1*2],'curvature',[1,1]);
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
set(gca,'xlim',[-1.25,1.25]);%x�����᷶Χ
set(gca,'ylim',[-1.25,1.25]);%y�����᷶Χ
%���Ƴ�ʼͼ��
hold on;
%���˻�λ��
h1 = scatter(pos(1, 1), pos(2,1), 256, 'o', 'filled');
h2 = scatter(pos(3, 1), pos(4,1), 256, 'o', 'filled');
h3 = scatter(pos(5, 1), pos(6,1), 256, 'o', 'filled');
h4 = scatter(pos(7, 1), pos(8,1), 256, 'o', 'filled');

% ���˻���ʶ
ht1 = text(pos(1,1)+text_right, pos(2,1), '1','FontSize', 10 );
ht2 = text(pos(3,1)+text_right, pos(4,1), '2','FontSize', 10 );
ht3 = text(pos(5,1)+text_right, pos(6,1), '3','FontSize', 10 );
ht4 = text(pos(7,1)+text_right, pos(8,1), '4','FontSize', 10 );
% ���˻�����Ƕ�

hq1 = patch([pos(1, 1);pos(1, 1)+angle((vnum*0+3):2:(vnum*0+6),1)],[pos(2,1);pos(2,1)+angle((vnum*0+4):2:(vnum*0+6),1)],[0 1 0],'FaceAlpha',.3);
hq2 = patch([pos(3, 1);pos(3, 1)+angle((vnum*1+3):2:(vnum*1+6),1)],[pos(4,1);pos(4,1)+angle((vnum*1+4):2:(vnum*1+6),1)],[0 1 0],'FaceAlpha',.3);
hq3 = patch([pos(5, 1);pos(5, 1)+angle((vnum*2+3):2:(vnum*2+6),1)],[pos(6,1);pos(6,1)+angle((vnum*2+4):2:(vnum*2+6),1)],[0 1 0],'FaceAlpha',.3);
hq4 = patch([pos(7, 1);pos(7, 1)+angle((vnum*3+3):2:(vnum*3+6),1)],[pos(8,1);pos(8,1)+angle((vnum*3+4):2:(vnum*3+6),1)],[0 1 0],'FaceAlpha',.3);
hq1.EdgeColor = 'none';
hq2.EdgeColor = 'none';
hq3.EdgeColor = 'none';
hq4.EdgeColor = 'none';
% ���˻���ͷ����
hqhead = quiver(pos(1:2:8,1), pos(2:2:8,1), ones(4,1), zeros(4,1));
hqhead.LineWidth = 3;
hqhead.AutoScaleFactor = 0.2;
hqhead.Color = 'r';
% hqhead.AutoScale = 'off';
%���˻������Ұä������
hqcam = quiver([pos(1:2:8, 1); pos(1:2:8, 1)], [pos(2:2:8, 1); pos(2:2:8, 1)], [cos(alpha_max)*ones(4,1); cos(alpha_min)*ones(4,1)], [sin(alpha_max)*ones(4,1); sin(alpha_min)*ones(4,1)]);
hqcam.LineWidth = 2;
hqcam.AutoScaleFactor = 0.25;
hqcam.Color = 'r';
hqcam.ShowArrowHead = 'off';
% hqcam.AutoScale = 'off';
grid off
axis off
%���˻�֮�����ͨ�����ͼ
x_connect = [];
y_connect = [];
u_connect = [];
v_connect = [];
for i = 1:N%������ͨ�ĵ�
    for j=1:N
        if M_connect((i-1)*N+j,1)==1%��ʾi������j,����ͨ����
            x_connect = [x_connect; pos(2*(i-1)+1, 1)];
            y_connect = [y_connect; pos(2*(i-1)+2, 1)];
            u_connect = [u_connect; pos(2*(j-1)+1, 1)-pos(2*(i-1)+1, 1)];
            v_connect = [v_connect; pos(2*(j-1)+2, 1)-pos(2*(i-1)+2, 1)];
        end
    end
end
hqconnect = quiver(x_connect, y_connect, u_connect, v_connect);
hqconnect.LineWidth = 1;
% hqconnect.Color = 'g';
hqconnect.AutoScale = 'off';
hqconnect.LineStyle = '--';

%����gif
if savegif
gif_name = [datestr(datetime('now'), 30), 'circle.gif'];
[A,pic] = rgb2ind(frame2im(getframe),256);
imwrite(A,pic,gif_name,'gif', 'Loopcount',inf,'DelayTime',frame_t);
end

for i=1:length(smooth_result(end,:))
    hold on;
    h1.XData = pos(1, i);
    h1.YData = pos(2, i);
    h2.XData = pos(3, i);
    h2.YData = pos(4, i);
    h3.XData = pos(5, i);
    h3.YData = pos(6, i);
    h4.XData = pos(7, i);
    h4.YData = pos(8, i);
    ht1.Position = [pos(1, i)+text_right, pos(2, i)];
    ht2.Position = [pos(3, i)+text_right, pos(4, i)];
    ht3.Position = [pos(5, i)+text_right, pos(6, i)];
    ht4.Position = [pos(7, i)+text_right, pos(8, i)];

   
    hq1.XData = [pos(1, i);pos(1, i)+angle((vnum*0+3):2:(vnum*0+6),i)];  
    hq1.YData = [pos(2, i);pos(2, i)+angle((vnum*0+4):2:(vnum*0+6),i)];
    hq2.XData = [pos(3, i);pos(3, i)+angle((vnum*1+3):2:(vnum*1+6),i)]; 
    hq2.YData = [pos(4, i);pos(4, i)+angle((vnum*1+4):2:(vnum*1+6),i)];
    hq3.XData = [pos(5, i);pos(5, i)+angle((vnum*2+3):2:(vnum*2+6),i)];  
    hq3.YData = [pos(6, i);pos(6, i)+angle((vnum*2+4):2:(vnum*2+6),i)];
    hq4.XData = [pos(7, i);pos(7, i)+angle((vnum*3+3):2:(vnum*3+6),i)]; 
    hq4.YData = [pos(8, i);pos(8, i)+angle((vnum*3+4):2:(vnum*3+6),i)];
    
    hqhead.XData =  pos(1:2:8, i);
    hqhead.YData =  pos(2:2:8, i);
    
    hqcam.XData =  [pos(1:2:8, i); pos(1:2:8, i)];
    hqcam.YData =  [pos(2:2:8, i); pos(2:2:8, i)];
    
    %���˻�֮�����ͨ�����ͼ
    x_connect = [];
    y_connect = [];
    u_connect = [];
    v_connect = [];
    for k = 1:N%������ͨ�ĵ�
        for j=1:N
            if M_connect((k-1)*N+j,i)==1%��ʾi������j,����ͨ����
                x_connect = [x_connect; pos(2*(k-1)+1, i)];
                y_connect = [y_connect; pos(2*(k-1)+2, i)];
                u_connect = [u_connect; pos(2*(j-1)+1, i)-pos(2*(k-1)+1, i)];
                v_connect = [v_connect; pos(2*(j-1)+2, i)-pos(2*(k-1)+2, i)];
            end
        end
    end
    hqconnect.XData = x_connect;
    hqconnect.YData = y_connect;
    hqconnect.UData = u_connect;
    hqconnect.VData = v_connect;

    
    drawnow;
    pause(frame_t);
    %����ʱд��gif
    if savegif
         [A,pic] = rgb2ind(frame2im(getframe),256);
         imwrite(A,pic,gif_name,'WriteMode','append','DelayTime',frame_t);
    end
end
