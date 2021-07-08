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
q2_s=-180;    q2_end=180;
q3_s=-180;  q3_end=180;
q4_s=-180; q4_end=180;
q5_s=-180;  q5_end=180;
q6_s=-180;    q6_end=180;
    
 %计算点数
 num=20000;
 
%% 求取工作空间
%设置轴关节随机分布,轴6不对工作范围产生影响，设置为0
q1_rand = q1_s + rand(num,1)*(q1_end - q1_s);%rand产生num行1列，在0~1之间的随机数
q2_rand = q2_s + rand(num,1)*(q2_end - q2_s);
q3_rand = q3_s + rand(num,1)*(q3_end - q3_s);
q4_rand = q4_s + rand(num,1)*(q4_end - q4_s);
q5_rand = q5_s + rand(num,1)*(q5_end - q5_s);
q6_rand = rand(num,1)*0;
q = [q1_rand q2_rand q3_rand q4_rand q5_rand q6_rand]*deg;
    
%正运动学计算工作空间
tic;
T_cell = cell(num,1);
for i=1:1:num
    [T_cell{i}]=ur5.fkine(q(i,:));%正向运动学仿真函数
end
    %[T_cell{:,1}]=ur5.fkine(q);%正向运动学仿真函数
disp(['运行时间：',num2str(toc)]);
 
%% 分析结果
%绘制工作空间
t1=clock;
figure('name','机械臂工作空间')
hold on
plotopt = {'noraise', 'nowrist', 'nojaxes', 'delay',0};
ur5.plot([0 30*deg 0 0 40*deg 0], plotopt{:});
figure_x=zeros(num,1);
figure_y=zeros(num,1);
figure_z=zeros(num,1);
for cout=1:1:num
    figure_x(cout,1)=T_cell{cout}.t(1);
    figure_y(cout,1)=T_cell{cout}.t(2);
    figure_z(cout,1)=T_cell{cout}.t(3);
 end
 plot3(figure_x,figure_y,figure_z,'b.','MarkerSize',0.5);
 hold off
 disp(['绘制工作空间运行时间：',num2str(etime(clock,t1))]);  
     
 %获取X,Y,Z空间坐标范围
 Point_range=[min(figure_x) max(figure_x) min(figure_y) max(figure_y) min(figure_z) max(figure_z)];
