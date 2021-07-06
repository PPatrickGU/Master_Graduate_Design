%%
%%逆运动学验证
clear;
clc;
close all;
%建立机器人模型
% theta d a alpha offset
L1=Link([0 0 0 0 0 ],'modified'); %连杆的D-H参数
L2=Link([0 149.09 0 -pi/2 0 ],'modified');
L3=Link([0 0 431.8 0 0 ],'modified');
L4=Link([0 433.07 20.32 -pi/2 0 ],'modified');
L5=Link([0 0 0 pi/2 0 ],'modified');
L6=Link([0 0 0 -pi/2 0 ],'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','puma560','base' , ...
transl(0, 0, 0.62)* trotz(0)); %连接连杆，机器人取名puma560
% robot.plot([0,pi/2,0,0,0,pi/2]);%输出机器人模型，后面的六个角为输出时的theta姿态
robot.plot([0,0,0,0,0,0])
figure(1)
% robot.teach() 
robot.display(); %显示D-H表
hold on
T1 = transl(500,0,-400);	%起点
T2 = transl(-300,-800,-100);	%终点
%ctraj 利用匀加速匀减速规划轨迹%
T = ctraj(T1,T2,50);
Tj = transl(T);
%输出末端轨迹%
plot3(Tj(:,1),Tj(:,2),Tj(:,3));
grid on;
% 
% T=Tj;
% G = [];  %用来存储新的障碍节点，用于栅格地图的更新
% % N = ndims(T);
% N = size(T,1);
% Q = []; %可行路径上的最终各点的关节角
% c = [0 0 0 0 0 0 0 0]; %不可行点判别数组  % A为路径可行判别数，可行为1，不可行为0
% t0=[0 0 0 0 0 0];
% e0 = [];
% temp0=0;
% sum = [];
% for i =1:N    %分别对各点进行碰撞检测
%     t = T(i,:);
%     q = transl(t);
%     TH = niyundx_change(q);
%     for n=1:8
%         for m=1:6
%             if TH(n,m)>pi
%                 TH(n,m)=TH(n,m)-2*pi;
%             elseif TH(n,m)<-pi
%                 TH(n,m)=TH(n,m)+2*pi;
%             end
%         end
%     end
% %     TH
%     for j=1:8
%         for k=1:6
%            temp0=temp0+abs(TH(j,k)-t0(k));            
%         end
%         e0(j)=temp0;
%         temp0=0;        
%     end
%     [e1,index]=sort(e0);
%     THH = [TH(index(1),:);TH(index(2),:);TH(index(3),:);TH(index(4),:);TH(index(5),:);
%         TH(index(6),:);TH(index(7),:);TH(index(8),:)];
%     
% %     for j = 1:8   %八组关节角分别进行碰撞检测，若有可行的关节角，则后面的不继续检查
% %          B = pengzhuangjiance(THH(j,:));
% %          if B == 0
% %                 Q(i,:) = THH(j,:);
% %                 t0=Q(i,:);
% %                 break
% %          else
% %              c(j) = 1; %把不可行的关节角相应的判别数组里的位置置1，便于最后的关节可行检测
% %          end
% %     end
% 
%     Q(i,:) = THH(1,:);
%     t0=Q(i,:);
%     cc = 1;
%     for n = 1:8
%         cc = cc*c(n);
%     end
%     if cc ==1  %判别i坐标处是否所有关节角都不可行
%         G = [G;T(i,:)];
%         A = 0;  %若都不可行，表示路径不可行，该点纳入障碍节点，需重新规划
%         break
%     else if i == N  %若可行，判断是否已经最后一个路径点
%             A = 1;   %若是，则路径可行，输出各点的最优关节角
%             Q;
%             break
%         end
%     end
% end
% for i=1:50;
%     plot3(Tj(i,1),Tj(i,2),Tj(i,3),'r.');
%     hold on;
%     robot.plot(Q(i,:));
% end
