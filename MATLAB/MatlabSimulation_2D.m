% Simulation method : RSS + Trilateration
clc; clf; clear; close all;
LedPos=[0 0;0 100;100 100;100 0]; % position of 4 LEDs
imageSize = 400;
DisEr=zeros(100,1);
N = 20; % number of test

for i=1:N
    x = imageSize*rand(1) - imageSize/2; 
    y = imageSize*rand(1) - imageSize/2; 
    Coord_sim=[x y]; % Coordiantion of simulation
        
    d1 = sqrt(x.^2+y.^2)+5*randn(1)-.5;
    d2 = sqrt(x.^2+(y-100).^2)+5*randn(1)-.5;
    d3 = sqrt((x-100).^2+(y-100).^2)+5*rand(1)-.5;
    d4 = sqrt(y.^2+(x-100).^2)+5*rand(1)-.5;
    d = [d1 d2 d3 d4];
   
    L = (d1.^2-d4.^2+100^2)/200;
    M = (d1.^2+100.^2-d2.^2)/200;

    X=[mean(L) mean(M)];
                
    f(1)=figure(1);
    clf
    plot(LedPos(:,1),LedPos(:,2),'co','MarkerSize',12,'lineWidth',2,'MarkerFaceColor','y');
    grid on
    hold on
    plot(x,y,'k+','MarkerSize',12,'lineWidth',2,'MarkerFaceColor','k');
    plot(X(:,1),X(:,2),'ro','MarkerSize',8,'lineWidth',2);
    legend('LEDs Position','Object true location','Object estimated location',...
                   'Location','Best');
    x=X(:,1);
    y=X(:,2);
    
    r=10;        
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    plot(xunit, yunit)
            
          
    % Compute the Root Mean Squred Error
    Err = abs(mean(sqrt(sum((X-Coord_sim).^2))));
    DisEr(i,:)=real([Err]);
    i = i+1;
    title(['Mean Estimation error is ',num2str(Err),' cm']);
    axis([50-imageSize/2 50+imageSize/2 50-imageSize/2 50+imageSize/2]);
%     axis equal;
    
    pause(1);
end 
mean(DisEr)*100/N
   
