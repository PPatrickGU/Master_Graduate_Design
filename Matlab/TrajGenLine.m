clear;
clc;
close all;

%模型导入
mdl_ur5

% T1T2直线起点和终点
T1=transl(0.4,-0.6,-0.3);
T2=transl(0.4,0,0.8);

t=[0:0.4:5]';

Ts1=ctraj(T1,T2,length(t)); %利用匀加速匀减速规划轨迹
q_s1=ur5.ikine(Ts1);

% 建模、画图
q = [q_s1];
x=squeeze(Ts1(1,4,:)); y=squeeze(Ts1(2,4,:)); z=squeeze(Ts1(3,4,:));
figure('Name','Ur机器人末端轨迹图');
plot3(x,y,z,'b')

% 机械臂运动演示
% ur5.plot(q,'trail','b');

% 机械臂运动逐帧演示并保存成gif
for i = 1:1:length(q)  
    ur5.plot(q(i,:)); 
    
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    % Write to the GIF File 
    if i == 1
        imwrite(I,map,'TrajGenLine.gif','gif', 'Loopcount',inf,'DelayTime',0.2);
    else
        imwrite(I,map,'TrajGenLine.gif','gif','WriteMode','append','DelayTime',0.2);
    end
    hold on
 end

