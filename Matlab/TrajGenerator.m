L1(1)=Link([0,0,0,pi/2]);
L1(2)=Link([0,0,0.4318,0]);
L1(3)=Link([0, 0.15005,0.0203,-pi/2]);
L1(4)=Link([0,0.4318,0,pi/2]);
L1(5)=Link([0,0,0,-pi/2]);
L1(6)=Link([0,0,0,0]);
robot=SerialLink(L1,'name','six link');

% T1T2直线起点和终点
T1=transl(0.4,0.4,-0.3);
T2=transl(0.4,0,-0.3);
T3=transl(0.4,-0.6,-0.3);
T4=transl(0.4,-0.4,0.4);
T5=transl(0.4,0.6,0.4);
T6=transl(0.4,0.4,-0.3);
t=[0:0.4:5]';
Ts1=ctraj(T1,T2,length(t));
Ts2=ctraj(T2,T3,length(t));
Ts3=ctraj(T3,T4,length(t));
Ts4=ctraj(T4,T5,length(t));
Ts5=ctraj(T5,T6,length(t));
q_s1=robot.ikine6s(Ts1);
q_s2=robot.ikine6s(Ts2);
q_s3=robot.ikine6s(Ts3);
q_s4=robot.ikine6s(Ts4);
q_s5=robot.ikine6s(Ts5);
Ts1=ctraj(T1,T3,length(t));
Ts2=ctraj(T3,T4,length(t));
Ts3=ctraj(T4,T5,length(t));
Ts4=ctraj(T5,T6,length(t));
% 圆弧分成n份
n = 50;
h=zeros(4,4,n);
for i = 1:n
    h(1,1,i)=1;
    h(2,2,i)=1;
    h(3,3,i)=1;
    
    alpht = -pi/2+2*pi*(i-1)/(n-1);
    h(1,4,i)=0.4;
    h(2,4,i)=0+0.3*cos(alpht);
    h(3,4,i)=0+0.3*sin(alpht);   
end
q_c=robot.ikine6s(h); 


% 建模、画图
q = [q_s1;q_c;q_s2;q_s3;q_s4;q_s5];
x=squeeze(Ts1(1,4,:)); y=squeeze(Ts1(2,4,:)); z=squeeze(Ts1(3,4,:));
x1=squeeze(Ts2(1,4,:)); y1=squeeze(Ts2(2,4,:)); z1=squeeze(Ts2(3,4,:));
x2=squeeze(Ts3(1,4,:)); y2=squeeze(Ts3(2,4,:)); z2=squeeze(Ts3(3,4,:));
x3=squeeze(Ts4(1,4,:)); y3=squeeze(Ts4(2,4,:)); z3=squeeze(Ts4(3,4,:));
x4=squeeze(h(1,4,:)); y4=squeeze(h(2,4,:)); z4=squeeze(h(3,4,:)); 
x2=[x;x1;x2;x3;x4];y2=[y;y1;y2;y3;y4];z2=[z;z1;z2;z3;z4];
figure('Name','Ur机器人末端轨迹图'); plot3(x2,y2,z2);robot.plot(q);
