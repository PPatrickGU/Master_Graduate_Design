%%
%%���˶�ѧ��֤
clear;
clc;
close all;
%����������ģ��
% theta d a alpha offset
L1=Link([0 0 0 0 0 ],'modified'); %���˵�D-H����
L2=Link([0 149.09 0 -pi/2 0 ],'modified');
L3=Link([0 0 431.8 0 0 ],'modified');
L4=Link([0 433.07 20.32 -pi/2 0 ],'modified');
L5=Link([0 0 0 pi/2 0 ],'modified');
L6=Link([0 0 0 -pi/2 0 ],'modified');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','puma560','base' , ...
transl(0, 0, 0.62)* trotz(0)); %�������ˣ�������ȡ��puma560
% robot.plot([0,pi/2,0,0,0,pi/2]);%���������ģ�ͣ������������Ϊ���ʱ��theta��̬
robot.plot([0,0,0,0,0,0])
figure(1)
% robot.teach() 
robot.display(); %��ʾD-H��
hold on
T1 = transl(500,0,-400);	%���
T2 = transl(-300,-800,-100);	%�յ�
%ctraj �����ȼ����ȼ��ٹ滮�켣%
T = ctraj(T1,T2,50);
Tj = transl(T);
%���ĩ�˹켣%
plot3(Tj(:,1),Tj(:,2),Tj(:,3));
grid on;
% 
% T=Tj;
% G = [];  %�����洢�µ��ϰ��ڵ㣬����դ���ͼ�ĸ���
% % N = ndims(T);
% N = size(T,1);
% Q = []; %����·���ϵ����ո���Ĺؽڽ�
% c = [0 0 0 0 0 0 0 0]; %�����е��б�����  % AΪ·�������б���������Ϊ1��������Ϊ0
% t0=[0 0 0 0 0 0];
% e0 = [];
% temp0=0;
% sum = [];
% for i =1:N    %�ֱ�Ը��������ײ���
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
% %     for j = 1:8   %����ؽڽǷֱ������ײ��⣬���п��еĹؽڽǣ������Ĳ��������
% %          B = pengzhuangjiance(THH(j,:));
% %          if B == 0
% %                 Q(i,:) = THH(j,:);
% %                 t0=Q(i,:);
% %                 break
% %          else
% %              c(j) = 1; %�Ѳ����еĹؽڽ���Ӧ���б��������λ����1���������Ĺؽڿ��м��
% %          end
% %     end
% 
%     Q(i,:) = THH(1,:);
%     t0=Q(i,:);
%     cc = 1;
%     for n = 1:8
%         cc = cc*c(n);
%     end
%     if cc ==1  %�б�i���괦�Ƿ����йؽڽǶ�������
%         G = [G;T(i,:)];
%         A = 0;  %���������У���ʾ·�������У��õ������ϰ��ڵ㣬�����¹滮
%         break
%     else if i == N  %�����У��ж��Ƿ��Ѿ����һ��·����
%             A = 1;   %���ǣ���·�����У������������Źؽڽ�
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
