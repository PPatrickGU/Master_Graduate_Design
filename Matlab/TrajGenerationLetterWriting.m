clear;
clc;
close all;

mdl_puma560
aplha=pi/4:pi/40:2*pi-pi/4;
k=inf;                  % 斜面斜率k 
r=1;
letterShrink=0.2;       %缩放倍数
letterTranslX=0.4;      % 沿X平移量
letterTranslY=0;        % 沿Y平移量
letterTranslZ=0;        % 沿Z平移量
if k==inf   % 垂直平面
    z=r*cos(aplha);
    x= diag(zeros(length(z)));
    y=r*sin(aplha);
    path=[diag(zeros(length(y))),y',z'];
    path=[path(1,:)+0.5*[-1,0,0];path;path(end,:)+0.5*[-1,0,0]];   % 垂直平面上的C
    y=-0.5:0.1:0.5;
    z=-0.5:0.1:0.5;
    [Y, Z] = meshgrid(y, z);
    X=ones(size(Y))*letterTranslX;
    mesh(X,Y,Z)
    hold on
    initPoint=path(1,:)+0.5*[-1,0,0];
else
    x=r*cos(aplha);
    y=r*sin(aplha);
    z=k*x;
    path=[x',y',z'];
    path=[path(1,:)+0.5*[-k,0,1];path;path(end,:)+0.5*[-k,0,1]];  % 斜率为k的斜面上的C
    x=0:0.1:1;
    y=-0.5:0.1:0.5;
    [X,Y]=meshgrid(x,y);
    z=k*(X-letterTranslX);
    mesh(X,Y,z)
    hold on
    initPoint=path(1,:)+0.5*[-k,0,1];
end
plot3(letterShrink*path(:,1)+letterTranslX,letterShrink*path(:,2)...
    +letterTranslY,letterShrink*path(:,3)++letterTranslZ,'color','k','LineWidth',1)
p=mstraj(path,[0.8,0.8,0.6],[],initPoint,0.4,0.1); 
Tp=transl(letterShrink*p); 
Tp=homtrans(transl(letterTranslX, letterTranslY, letterTranslZ),Tp);
p560.tool=transl([0,0,0.2])*trotx(pi); 
q=p560.ikine6s(Tp); 
p560.plot(q)

