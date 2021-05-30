clear;
clc;
close all;

%模型导入
mdl_ur5
format short;
    
%角度转换
deg=pi/180;  %度
radian=180/pi; %弧度

%预设的角度值，保证一定能够求解
%给定末端执行器的初始位置，运动学正解fkine求解末端运动状态
init_ang=[0,pi/3,0,-pi/2,0,-pi/6];
p1 = ur5.fkine(init_ang);

%给定末端执行器的终止位置
targ_ang=[pi/4,-pi/3,pi/5,pi/2,-pi/4,pi/6];
p2 = ur5.fkine(targ_ang);

%利用运动学反解ikine求解各关节转角
init_ang = ur5.ikine(p1);%使用运动学迭代反解的算法计算得到初始的关节角度
targ_ang = ur5.ikine(p2);%使用运动学迭代反解的算法计算得到目标关节角度

%利用五次多项式计算关节速度和加速度
step=40;
[q,qd,qdd] = jtraj(init_ang, targ_ang, step);


figure('Name','UR5机器人路径规划仿真演示');
% 显示机器人姿态随时间的变化
% subplot(3,3,[1,4,7]);
ur5.plot(q,'trail','b');     %trail是提醒计算机跟踪轨迹

figure('Name','UR5机器人运动状态');
%显示机器人关节运动状态
subplot(3,2,1);
i=1:6;
plot(q(:,i));
title('初始位置      各关节角度随时间的变化      目标位置');
grid on;
subplot(3,2,2);
i=1:6;
plot(qd(:,i));
title('各关节角速度随时间的变化');
grid on;
subplot(3,2,3);
i=1:6;
plot(qdd(:,i));
title('各关节角加速度随时间的变化');
grid on;

%显示末端执行器的位置
subplot(3,2,4);
hold on
grid on
title('末端执行器在三维空间中的位置变化');
for i=1:step
position = ur5.fkine(q(i,:));
plot3(position.t(1),position.t(2),position.t(3),'b.','MarkerSize',5);
end

%显示末端执行器的线速度与角速度
subplot(3,2,5);
hold on
grid on
title('末端执行器速度大小随时间的变化');
vel = zeros(step,6);
vel_velocity = zeros(step,1);
vel_angular_velocity = zeros(step,1);
for i=1:step
    vel(i,:) = ur5.jacob0(q(i,:))*qd(i,:)';
    vel_velocity(i) = sqrt(vel(i,1)^2+vel(i,2)^2+vel(i,3)^2);
    vel_angular_velocity(i) = sqrt(vel(i,4)^2+vel(i,5)^2+vel(i,3)^6);
end
x = linspace(1,step,step);
plot(x,vel_velocity);

subplot(3,2,6);
hold on
grid on
title('末端执行器角速度大小随时间的变化');
x = linspace(1,step,step);
plot(x,vel_angular_velocity);

