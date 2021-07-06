% 机械臂——键盘控制运动
%操作指令：
%机械臂末端：      W:前  A:左  S:后  D:右  Q:上  E:下
% 单关节角度增加： T:1   Y:2   U:3   I:4   O:5   P:6
% 单关节角度减小： G:1   H:2   J:3   K:4   L:5   ;:6
%                 +:所有速度增加  -:所有速度减小
%机械臂各关节运动范围：   -3pi~3pi     ！！！！！！！！！
startup_rvc       %启动Robotic Toolbox 工具箱
clc
clear
speed=1;          %机械臂运动速度（相对值）
speed_r=0.1;      %速度单次增大（或减小）数值
threshold_err=3;  %误差阈值
steptime=0.08;    %请根据电脑配置设置时间，如果按键无反应，或者指示灯闪烁，将此参数尝试改为0.1 重要！重要！重要！！！！
%定义位姿变换矩阵匿名函数
T=@(theta,d,a,alpha)[cos(theta),-sin(theta)*cos(alpha),  sin(theta)*sin(alpha),a*cos(theta);
    sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha),a*sin(theta);
    0,            sin(alpha),             cos(alpha),           d;
    0,                     0,                      0,           1];
%%
%构造机械臂模型并画出当前位姿
a=    [          0,         -52,       -40,          0,        0,    0 ];
d=    [         18,          20,       -20,         18,       20,   11 ];
alpha=[       pi/2,           0,         0,       pi/2,    -pi/2,    0 ];
% theta_init=[ 160/180*pi, -145/180*pi, 40/180*pi, -75/180*pi, 20/180*pi,   0 ];
theta_init=[ -20/180*pi, -160/180*pi, 40/180*pi, -60/180*pi, 20/180*pi,   0 ];
T01=T(theta_init(1),d(1),a(1),alpha(1));
T12=T(theta_init(2),d(2),a(2),alpha(2));
T23=T(theta_init(3),d(3),a(3),alpha(3));
T34=T(theta_init(4),d(4),a(4),alpha(4));
T45=T(theta_init(5),d(5),a(5),alpha(5));
T56=T(theta_init(6),d(6),a(6),alpha(6));
T_actuator=T01*T12*T23*T34*T45*T56;                             %正解得到位姿矩阵
L1 = Link([theta_init(1), d(1), a(1), alpha(1)]);
L2 = Link([theta_init(2), d(2), a(2), alpha(2)]);
L3 = Link([theta_init(3), d(3), a(3), alpha(3)]);
L4 = Link([theta_init(4), d(4), a(4), alpha(4)]);
L5 = Link([theta_init(5), d(5), a(5), alpha(5)]);
L6 = Link([theta_init(6), d(6), a(6), alpha(6)]);
robot = SerialLink([L1 L2 L3 L4 L5 L6],'name',' ');
figure
set(gcf,'WindowStyle','modal','Position',get(0,'ScreenSize'));  %设置全屏并置于屏幕最上方
string1={ '机械臂末端';                                          %说明信息展示
    '前：W';
    '后：S';
    '左：A';
    '右：D';
    '上：Q';
    '下：E'};
string2={ '单关节角度增加';
    'T：1';
    'Y：2';
    'U：3';
    'I：4';
    'O：5';
    'P：6'};
string3={ '单关节角度减小';
    'G：1';
    'H：2';
    'J：3';
    'K：4';
    'L：5';
    '; ：6';};
string4=['速度变化按键：','  速度增大：+',' 速度减小：-'];
annotation('textbox',[0.75,0.5,0.08,0.2],'LineStyle','-', 'LineWidth',1,'String',string1)
annotation('textbox',[0.83,0.5,0.08,0.2],'LineStyle','-', 'LineWidth',1,'String',string2)
annotation('textbox',[0.91,0.5,0.08,0.2],'LineStyle','-', 'LineWidth',1,'String',string3)
annotation('textbox',[0.91,0.7,0.08,0.08],'LineStyle','-', 'LineWidth',1,'String',string4)
annotation('ellipse',[0.78,0.79,0.1,0.1],'FaceColor','g')
%%
%程序初始化运行
robot.plot(theta_init,'jointdiam',1,'view',[-80 30])
T01=T(theta_init(1),d(1),a(1),alpha(1));
T12=T(theta_init(2),d(2),a(2),alpha(2));
T23=T(theta_init(3),d(3),a(3),alpha(3));
T34=T(theta_init(4),d(4),a(4),alpha(4));
T45=T(theta_init(5),d(5),a(5),alpha(5));
T56=T(theta_init(6),d(6),a(6),alpha(6));
T_actuator=T01*T12*T23*T34*T45*T56;
h(1)=annotation('textbox',[0.05,0.73,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节1角度：',num2str(theta_init(1)/pi*180);]);
h(2)=annotation('textbox',[0.05,0.68,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节2角度：',num2str(theta_init(2)/pi*180);]);
h(3)=annotation('textbox',[0.05,0.63,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节3角度：',num2str(theta_init(3)/pi*180);]);
h(4)=annotation('textbox',[0.05,0.58,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节4角度：',num2str(theta_init(4)/pi*180);]);
h(5)=annotation('textbox',[0.05,0.53,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节5角度：',num2str(theta_init(5)/pi*180);]);
h(6)=annotation('textbox',[0.05,0.48,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节6角度：',num2str(theta_init(6)/pi*180);]);
h(7)=annotation('textbox',[0.75,0.7,0.16,0.08],'LineStyle','-', 'LineWidth',1,'FontSize',30,'String',['速度：',num2str(speed)]);
h(8)=annotation('textbox',[0.05,0.43,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ 'X方向位置：',num2str(T_actuator(1,4));]);
h(9)=annotation('textbox',[0.05,0.38,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ 'Y方向位置：',num2str(T_actuator(2,4));]);
h(10)=annotation('textbox',[0.05,0.33,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ 'Z方向位置：',num2str(T_actuator(3,4));]);
trailx=[];traily=[];trailz=[];
theta_init__=theta_init;
Array1=zeros(1,6);
program=0;
theta_save=[];
hold on;
%%
%主程序循环
while 1
    pause(steptime)
    theta_save=[theta_save;theta_init];
    T01=T(theta_init(1),d(1),a(1),alpha(1));
    T12=T(theta_init(2),d(2),a(2),alpha(2));
    T23=T(theta_init(3),d(3),a(3),alpha(3));
    T34=T(theta_init(4),d(4),a(4),alpha(4));
    T45=T(theta_init(5),d(5),a(5),alpha(5));
    T56=T(theta_init(6),d(6),a(6),alpha(6));
    T_actuator=T01*T12*T23*T34*T45*T56;
    trailx=[trailx,T_actuator(1,4)];                                         %记录运动轨迹
    traily=[traily,T_actuator(2,4)];
    trailz=[trailz,T_actuator(3,4)];
    [az,el] = view;
    plot3(trailx,traily,trailz,'LineWidth',3,'Color','b')                    %画出运动轨迹
    quiver3(0,0,0,150,0,0,'LineWidth',3,'Color',[1 0 0],'MaxHeadSize',10)   %画出正方向箭头
    view(az,el)
    theta_init__=theta_init;
    %更新角度和速度数据
    delete(h);
    h(1)=annotation('textbox',[0.05,0.73,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节1角度：',num2str(theta_init(1)/pi*180);]);
    h(2)=annotation('textbox',[0.05,0.68,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节2角度：',num2str(theta_init(2)/pi*180);]);
    h(3)=annotation('textbox',[0.05,0.63,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节3角度：',num2str(theta_init(3)/pi*180);]);
    h(4)=annotation('textbox',[0.05,0.58,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节4角度：',num2str(theta_init(4)/pi*180);]);
    h(5)=annotation('textbox',[0.05,0.53,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节5角度：',num2str(theta_init(5)/pi*180);]);
    h(6)=annotation('textbox',[0.05,0.48,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ '关节6角度：',num2str(theta_init(6)/pi*180);]);
    h(7)=annotation('textbox',[0.75,0.7,0.16,0.08],'LineStyle','-', 'LineWidth',1,'FontSize',30,'String',['速度：',num2str(speed)]);
    h(8)=annotation('textbox',[0.05,0.43,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ 'X方向位置：',num2str(T_actuator(1,4));]);
    h(9)=annotation('textbox',[0.05,0.38,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ 'Y方向位置：',num2str(T_actuator(2,4));]);
    h(10)=annotation('textbox',[0.05,0.33,0.2,0.05],'LineStyle','-', 'FontSize',15,'LineWidth',1,'String',[ 'Z方向位置：',num2str(T_actuator(3,4));]);
    %按键输入判断
    if     strcmpi(get(gcf,'CurrentCharacter'),'=') speed=speed+speed_r;set(gcf,'CurrentCharacter','F');continue;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'-') speed=speed-speed_r;set(gcf,'CurrentCharacter','F');continue;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'w') T_actuator(1,4)=T_actuator(1,4)+speed;set(gcf,'CurrentCharacter','F');program=1;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'s') T_actuator(1,4)=T_actuator(1,4)-speed;set(gcf,'CurrentCharacter','F');program=1;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'a') T_actuator(2,4)=T_actuator(2,4)+speed;set(gcf,'CurrentCharacter','F');program=1;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'d') T_actuator(2,4)=T_actuator(2,4)-speed;set(gcf,'CurrentCharacter','F');program=1;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'q') T_actuator(3,4)=T_actuator(3,4)+speed;set(gcf,'CurrentCharacter','F');program=1;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'e') T_actuator(3,4)=T_actuator(3,4)-speed;set(gcf,'CurrentCharacter','F');program=1;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'t') theta_init(1)=theta_init(1)+speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'g') theta_init(1)=theta_init(1)-speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'y') theta_init(2)=theta_init(2)+speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'h') theta_init(2)=theta_init(2)-speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'u') theta_init(3)=theta_init(3)+speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'j') theta_init(3)=theta_init(3)-speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'i') theta_init(4)=theta_init(4)+speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'k') theta_init(4)=theta_init(4)-speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'o') theta_init(5)=theta_init(5)+speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'l') theta_init(5)=theta_init(5)-speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    elseif strcmpi(get(gcf,'CurrentCharacter'),'p') theta_init(6)=theta_init(6)+speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    elseif strcmpi(get(gcf,'CurrentCharacter'),';') theta_init(6)=theta_init(6)-speed/180*pi;set(gcf,'CurrentCharacter','F');program=2;
    else   program=0;
    end
    if program==0
        robot.plot(theta_init,'jointdiam',1)
        title('机械臂正常运行');
        annotation('ellipse',[0.78,0.79,0.1,0.1],'FaceColor','g')
    elseif program==2
        if max(abs(theta_init)>=3*pi)               %-3pi~3pi 运动范围限定
            theta_init=theta_init__;
            title('机械臂超出运动范围，请向其他方向运动');
            annotation('ellipse',[0.78,0.79,0.1,0.1],'FaceColor','r')
            continue;
        end
        robot.plot(theta_init,'jointdiam',1)
        title('机械臂正常运行');
        annotation('ellipse',[0.78,0.79,0.1,0.1],'FaceColor','g')
    elseif program==1
        %解析法计算逆运动学
        nx=T_actuator(1,1);ny=T_actuator(2,1);nz=T_actuator(3,1);
        ox=T_actuator(1,2);oy=T_actuator(2,2);oz=T_actuator(3,2);
        ax=T_actuator(1,3);ay=T_actuator(2,3);az=T_actuator(3,3);
        px=T_actuator(1,4);py=T_actuator(2,4);pz=T_actuator(3,4);
        %求解关节角1
        m=d(6)*ay-py;  n=ax*d(6)-px;
        if m^2+n^2-(d(4))^2>=0
            theta1(1,1)=atan2(m,n)-atan2(d(4),sqrt(m^2+n^2-(d(4))^2));
            theta1(1,2)=atan2(m,n)-atan2(d(4),-sqrt(m^2+n^2-(d(4))^2));
            %求解关节角5
            theta5(1,1:2)=acos(ax*sin(theta1)-ay*cos(theta1));
            theta5(2,1:2)=-acos(ax*sin(theta1)-ay*cos(theta1));
            %求解关节角6
            mm=nx*sin(theta1)-ny*cos(theta1); nn=ox*sin(theta1)-oy*cos(theta1);
            theta6=ones(2,1)*atan2(mm,nn)-atan2(sin(theta5),0);
            %求解关节角3
            mmm=d(5)*(sin(theta6).*(ones(2,1)*(nx*cos(theta1)+ny*sin(theta1)))+cos(theta6).*(ones(2,1)*(ox*cos(theta1)+oy*sin(theta1))))+ones(2,1)*(-d(6)*(ax*cos(theta1)+ay*sin(theta1))+px*cos(theta1)+py*sin(theta1));
            nnn=pz-d(1)-az*d(6)+d(5)*(oz*cos(theta6)+nz*sin(theta6));
            theta3(1:2,:)=acos((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)));
            theta3(3:4,:)=-acos((mmm.^2+nnn.^2-(a(2))^2-(a(3))^2)/(2*a(2)*a(3)));
            %求解关节角2
            mmm_s2(1:2,:)=mmm;
            mmm_s2(3:4,:)=mmm;
            nnn_s2(1:2,:)=nnn;
            nnn_s2(3:4,:)=nnn;
            s2=((a(3)*cos(theta3)+a(2)).*nnn_s2-a(3)*sin(theta3).*mmm_s2)./ ((a(2))^2+(a(3))^2+2*a(2)*a(3)*cos(theta3));
            c2=(mmm_s2+a(3)*sin(theta3).*s2)./(a(3)*cos(theta3)+a(2));
            theta2=atan2(real(s2),real(c2));  %此处可能会出现虚数，因此下部分程序用来记录该步骤s2或c2存在虚数所对应的解
            %记录s2或c2存在虚数所对应的解位置
            A_record=[1,5;3,7;2,6;4,8];
            [a1,b1]=size(s2);
            r=1;
            n_record=[];
            for n1=1:a1
                for n2=1:b1
                    if imag(s2(n1,n2))~=0||imag(c2(n1,n2))~=0
                        n_record(r)=A_record(n1,n2);
                        r=r+1;
                    end
                end
            end
            %整理关节角1 5 6 3 2
            theta(1:4,1)=theta1(1,1);theta(5:8,1)=theta1(1,2);
            theta(:,2)=[theta2(1,1),theta2(3,1),theta2(2,1),theta2(4,1),theta2(1,2),theta2(3,2),theta2(2,2),theta2(4,2)]';
            theta(:,3)=[theta3(1,1),theta3(3,1),theta3(2,1),theta3(4,1),theta3(1,2),theta3(3,2),theta3(2,2),theta3(4,2)]';
            theta(1:2,5)=theta5(1,1);theta(3:4,5)=theta5(2,1);
            theta(5:6,5)=theta5(1,2);theta(7:8,5)=theta5(2,2);
            theta(1:2,6)=theta6(1,1);theta(3:4,6)=theta6(2,1);
            theta(5:6,6)=theta6(1,2);theta(7:8,6)=theta6(2,2);
            %求解关节角4
            theta(:,4)=atan2(-sin(theta(:,6)).*(nx*cos(theta(:,1))+ny*sin(theta(:,1)))-cos(theta(:,6)).* (ox*cos(theta(:,1))+oy*sin(theta(:,1))),oz*cos(theta(:,6))+nz*sin(theta(:,6)))-theta(:,2)-theta(:,3);
            if length(n_record)~=0
                theta(n_record,:)=[]; %将前面的s2或c2中含有虚数部分所对应的解删掉
            end
            % 将含有虚数部分的解集删掉，并将结果换算到-pi~pi范围内
            [a2,b2]=size(theta);
            theta_=theta;
            r=0;
            for num1=1:a2
                r=r+1;
                for num2=1:b2
                    if imag(theta_(r,num2))~=0
                        theta_(r,:)=[];
                        r=r-1;
                        break;
                    else
                        theta_(r,num2)=mod(theta_(r,num2),2*pi);
                        if theta_(r,num2)>pi
                            theta_(r,num2)=theta_(r,num2)-2*pi;
                        end
                    end
                end
            end
            %找到角度相对上次计算结果变化量最小的解
            [a3,b3]=size(theta_);
            theta__=theta_;
            for q=1:6
                Array1(q)=1;
                theta__=[theta__;theta_-2*pi*ones(a3,1)*Array1;theta_+2*pi*ones(a3,1)*Array1]; %矩阵扩充，在-3pi~3pi范围内进行角度变化量求解
                Array1(q)=0;
            end
            [a3,b3]=size(theta__);
            theta_err=[];
            theta_err=abs(ones(a3,1)*theta_init-theta__)*[6:-1:1]';
            if min(theta_err(:))<threshold_err*speed          %找到最合适的解
                [R,C]=find(theta_err==min(theta_err(:)));
                theta_init_=theta__(R(1),:);
                if length(theta_init_)==0
                    theta_init=theta_init__;
                    title('机械臂运动到奇异位置，请向其他方向运动');
                    annotation('ellipse',[0.78,0.79,0.1,0.1],'FaceColor','r')
                    continue;
                elseif max(abs(theta_init_)>=3*pi)
                    theta_init=theta_init__;
                    title('机械臂超出运动范围，请向其他方向运动');
                    annotation('ellipse',[0.78,0.79,0.1,0.1],'FaceColor','r')
                    continue;
                else
                    theta_init=theta_init_;
                    robot.plot(theta_init,'jointdiam',1)
                    title('机械臂正常运行');
                    annotation('ellipse',[0.78,0.79,0.1,0.1],'FaceColor','g')
                end
            else
                theta_init=theta_init__;
                title('机械臂运动到奇异位置，请向其他方向运动');
                annotation('ellipse',[0.78,0.79,0.1,0.1],'FaceColor','r')
                continue;
            end
        else
            theta_init=theta_init__;
            title('机械臂运动到奇异位置，请向其他方向运动');
            annotation('ellipse',[0.78,0.79,0.1,0.1],'FaceColor','r')
            continue;
        end
    end
end