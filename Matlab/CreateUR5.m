clear;
clc;
close all;

% %%自己构建
% %UR5参数
% theta = [0 -pi/2 0 -pi/2 0 0];
% alpha = [pi/2 0 0 pi/2 -pi/2 0];
% a = [0 425 392.25 0 0 0]*10^(-3);
% d = [89.16 0 0 109.15 94.56 82.3]*10^(-3);
% 
% %L(i)=Link([theta,D,A,alpha,sigma],'convention')
% %使用Link类函数，基于DH法建模（改进型）
% for i=1:6
%     L(i)=Link([0,d(i),a(i),alpha(i)],'modified');
% end 
% 
% %使用Seriallink类函数把我们上面使用Link函数建立的连杆连成一个整体，生成一个串联机械臂模型
% UR5=SerialLink ([L(1),L(2),L(3),L(4),L(5),L(6)],'name','UR5');
% 
% %使用.plot绘制出某组关节变量的机械臂三维模型
% UR5.plot(theta)
% 
% %使用.display显示出我们建立的这个机械臂模型的信息
% UR5.display
% 
% % %使用.teach查看我们建立机械臂三维模型，可以对关节变量的值进行修改
% UR5.teach

%%使用自带的模型
mdl_ur5
% ur5.teach;
ur5.plot3d([0 0 0 0 0 0],'path','./UR5')
