clear;
clc;
close all;

% %%�Լ�����
% %UR5����
% theta = [0 -pi/2 0 -pi/2 0 0];
% alpha = [pi/2 0 0 pi/2 -pi/2 0];
% a = [0 425 392.25 0 0 0]*10^(-3);
% d = [89.16 0 0 109.15 94.56 82.3]*10^(-3);
% 
% %L(i)=Link([theta,D,A,alpha,sigma],'convention')
% %ʹ��Link�ຯ��������DH����ģ���Ľ��ͣ�
% for i=1:6
%     L(i)=Link([0,d(i),a(i),alpha(i)],'modified');
% end 
% 
% %ʹ��Seriallink�ຯ������������ʹ��Link������������������һ�����壬����һ��������е��ģ��
% UR5=SerialLink ([L(1),L(2),L(3),L(4),L(5),L(6)],'name','UR5');
% 
% %ʹ��.plot���Ƴ�ĳ��ؽڱ����Ļ�е����άģ��
% UR5.plot(theta)
% 
% %ʹ��.display��ʾ�����ǽ����������е��ģ�͵���Ϣ
% UR5.display
% 
% % %ʹ��.teach�鿴���ǽ�����е����άģ�ͣ����ԶԹؽڱ�����ֵ�����޸�
% UR5.teach

%%ʹ���Դ���ģ��
mdl_ur5
% ur5.teach;
ur5.plot3d([0 0 0 0 0 0],'path','./UR5')
