%�����е�۹����ռ����
clc;
clear;
close all;

format short;%��������
 
%�Ƕ�ת��
deg=pi/180;  %��
radian=180/pi; %����
 
%% ģ�͵���
mdl_ur5
 
%% ��ȡ�����ռ�
%�ؽڽ���λ
q1_s=-180; q1_end=180;
q2_s=0;    q2_end=90;
q3_s=-90;  q3_end=90;
q4_s=-180; q4_end=180;
q5_s=-90;  q5_end=90;
q6_s=0;    q6_end=360;
    
%����step
%stepԽ�󣬵�Խϡ�裬����ʱ��Խ��
step=20;%���㲽��
step1= (q1_end -q1_s) / step;
step2= (q2_end -q2_s) / step;
step3= (q3_end -q3_s) / step;
step4= (q4_end -q4_s) / step;
step5= (q5_end -q5_s) / step;
step6= (q6_end -q6_s) / step;
    
%���㹤���ռ�
tic;%tic1
i=1;
T = zeros(3,1);
T_x = zeros(1,step1*step2*step3*step4*step5);
T_y = zeros(1,step1*step2*step3*step4*step5);
T_z = zeros(1,step1*step2*step3*step4*step5);  
for  q1=q1_s:step:q1_end
    for  q2=q2_s:step:q2_end
          for  q3=q3_s:step:q3_end
              for  q4=q4_s:step:q4_end
                  for q5=q5_s:step:q5_end
                          T=ur5.fkine([q1*deg q2*deg q3*deg q4*deg q5*deg 0]);%�����˶�ѧ���溯��
%                         T = T.T;
                          T_x(1,i) = T.t(1); 
                          T_y(1,i) = T.t(2); 
                          T_z(1,i) = T.t(3); 
                          i=i+1;
                  end
             end
         end
    end 
 end
 disp(['ѭ������ʱ�䣺',num2str(toc)]); 
 t1=clock;
     
%% ���ƹ����ռ�
figure('name','�����е�۹����ռ�')
hold on
plotopt = {'noraise', 'nowrist', 'nojaxes', 'delay',0};
ur5.plot([0 20*deg 0 0 0 0], plotopt{:});
plot3(T_x,T_y,T_z,'r.','MarkerSize',3);
disp(['���ƹ����ռ�����ʱ�䣺',num2str(etime(clock,t1))]);  
 
%��ȡX,Y,Z�ռ����귶Χ
Point_range=[min(T_x) max(T_x) min(T_y) max(T_y) min(T_z) max(T_z)]
hold off