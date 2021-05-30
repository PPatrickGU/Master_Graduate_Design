clear;
clc;
close all;

%模型导入
mdl_ur5

% 圆弧分成n份
n = 2;
h=zeros(4,4,n);
for i = 1:n
    h(1,1,i)=1;
    h(2,2,i)=1;
    h(3,3,i)=1;
    alpht = -pi/2+2*pi*(i-1)/(n-1);
    h(1,4,i)=0+0.4*cos(alpht);
    h(2,4,i)=0-0.6*sin(alpht);
    h(3,4,i)=0.3;   
end

q_c=ur5.ikine(h); 

% 建模、画图
q = [q_c];
x=squeeze(h(1,4,:)); y=squeeze(h(2,4,:)); z=squeeze(h(3,4,:)); 
figure('Name','Ur机器人末端轨迹图'); plot3(x,y,z,'c.');

% 机械臂运动演示
ur5.plot(q,'trail','b');
