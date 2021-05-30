%EndSpaceAnimaite.m
%机械臂可达空间动画求解
%using  Robotic Toolbox 10.4
%update 2020.05
clc;
clear;
close all;

%% 准备
%保留精度
format short;
    
%角度转换
deg=pi/180;  %度
radian=180/pi; %弧度
    
%模型导入
mdl_ur5

%% 参数
%关节角限位
q1_s=-180; q1_end=180;
q2_s=0;    q2_end=90;
q3_s=-90;  q3_end=90;
q4_s=-180; q4_end=180;
q5_s=-90;  q5_end=90;
q6_s=0;    q6_end=360;
    
 %计算点数
num=250;
 
%% 求取工作空间
%设置轴关节随机分布,轴6不对工作范围产生影响，设置为0
q1_rand = q1_s + rand(num,1)*(q1_end - q1_s);%rand产生num行1列，在0~1之间的随机数
q2_rand = q2_s + rand(num,1)*(q2_end - q2_s);
q3_rand = q3_s + rand(num,1)*(q3_end - q3_s);
q4_rand = q4_s + rand(num,1)*(q4_end - q4_s);
q5_rand = q5_s + rand(num,1)*(q5_end - q5_s);
q6_rand = rand(num,1)*0;
figure('name','End Space')
 
for n=1:1:num
    
    qq = [q1_rand(n) q2_rand(n) q3_rand(n) q4_rand(n) q5_rand(n) q6_rand(n)]*deg;
    ur5.plot(qq);%动画显示
    hold on;
    Mricx=ur5.fkine(qq);
    plot3(Mricx.t(1),Mricx.t(2),Mricx.t(3),'b.','MarkerSize',0.5);%画出落点

    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    % Write to the GIF File 
    if n == 1
        imwrite(I,map,'EndSpaceAnimate.gif','gif', 'Loopcount',inf,'DelayTime',0.1);
    else
        imwrite(I,map,'EndSpaceAnimate.gif','gif','WriteMode','append','DelayTime',0.1);
    end
    
    
end
