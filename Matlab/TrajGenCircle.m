clear;
clc;
close all;

%模型导入
mdl_ur5

%定义圆，分成N份
N = (0:0.5:20)'; 
center = [0 0 0.2];
radius = 0.9;
theta = ( N/N(end))*2*pi;
points = (center + radius*[cos(theta) sin(theta) zeros(size(theta))])';  
plot3(points(1,:),points(2,:),points(3,:),'b');
T = transl(points');
q = ur5.ikine(T);

% % 机械臂运动演示与画图
% x=squeeze(T(1,4,:)); y=squeeze(T(2,4,:)); z=squeeze(T(3,4,:)); 
% figure('Name','Ur机器人末端轨迹图'); plot3(x,y,z,'c.');
% 
% % 
% ur5.plot(q,'trail','b');

% 机械臂运动演示与画图
for i = 1:1:length(q)  
    ur5.plot(q(i,:),'trail','b'); 
    
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    % Write to the GIF File 
    if i == 1
        imwrite(I,map,'./image/TrajGenCircle.gif','gif', 'Loopcount',inf,'DelayTime',0.2);
    else
        imwrite(I,map,'./image/TrajGenCircle.gif','gif','WriteMode','append','DelayTime',0.2);
    end
    hold on
 end
