% Simulation method : RSS + Trilateration 3D
clc; clf; clear; close all;
L1 = [0, 0, 200];
L2 = [0, 100, 200];
L3 = [100, 100, 200];
L4 = [100, 0, 200];
LedPos=[L1;L2;L3;L4]; % position of 4 LEDs
imageSize = 400;
DisEr=zeros(100,1);
num = 20; % number of test

for i=1:num
    x = (imageSize-50)*rand(1) - (imageSize-50)/2; 
    y = (imageSize-50)*rand(1) - (imageSize-50)/2; 
    z = (imageSize-10)/4*rand(1) + imageSize/4; 
    Coord_sim=[x y z]; % Coordiantion of simulation
        
    d1 = sqrt(x.^2+y.^2+(z-200).^2)+5*randn-.5;
    d2 = sqrt(x.^2+(y-100).^2+(z-200).^2)+5*randn-.5;
    d4 = sqrt(y.^2+(x-100).^2+(z-200).^2)+5*rand(1)-.5;
    d3 = sqrt((x-100).^2+(y-100).^2+(z-200).^2)+5*rand(1)-.5;
    
%     d1 = sqrt(x.^2+y.^2+(z-200).^2);
%     d2 = sqrt(x.^2+(y-100).^2+(z-200).^2);
%     d4 = sqrt(y.^2+(x-100).^2+(z-200).^2);
%     d3 = sqrt((x-100).^2+(y-100).^2+(z-200).^2);
    
    d = [d1 d2 d3 d4];

    y1 = (d1.^2+100.^2-d2.^2)/200; %intersection betweenn 1 & 2 & 4
    x1 = (d1.^2+100.^2-d4.^2)/200;
    z1 = 200-sqrt(d1.^2-x1.^2-y1.^2);
    L(1) = real(x1);
    M(1) = real(y1);
    N(1) = real(z1);
    P1 = [x1,y1,z1];
    
    x2 = (100.^2-d4.^2+d1.^2)/200;     %1&3&4
    y2 = (100.^2+d4.^2-d3.^2)/200;
    z2 = 200-sqrt(d1.^2-x2.^2-y2.^2);
    L(2) = real(x2);
    M(2) = real(y2);
    N(2) = real(z2);
    P2 = [x2,y2,z2];
%     
    x3 = (100.^2+d2.^2-d3.^2)/200;     %1&2&3
    y3 = (100.^2+d1.^2-d2.^2)/200;
    z3 = 200-sqrt(d1.^2-x3.^2-y3.^2);
    L(3) = real(x3);
    M(3) = real(y3);
    N(3) = real(z3);
    P3 = [x3,y3,z3];
    
    x4 = (100.^2+d2.^2-d3.^2)/200;     %2&3&4
    y4 = (100.^2+d4.^2-d3.^2)/200;
    z4 = 200-sqrt(d1.^2-x4.^2-y4.^2);
    L(4) = real(x4);
    M(4) = real(y4);
    N(4) = real(z4);
    P4= [x4,y4,z4];
         
    X=[mean(L) mean(M) mean(N)];
                
    f(1)=figure(1);
    clf
    plot3(LedPos(:,1),LedPos(:,2),LedPos(:,3),'co','MarkerSize',12,'lineWidth',2,'MarkerFaceColor','y');
    grid on
    hold on
    plot3(x,y,z,'k+','MarkerSize',10,'lineWidth',2,'MarkerFaceColor','k');
    
    x=X(:,1);
    y=X(:,2);
    z=X(:,3);
    
    plot3(x,y,z,'ro','MarkerSize',12,'lineWidth',2);
    legend(  'LEDs Position','Object true location','Object estimated location',...
                   'Location','Best');
               
    [x_e,y_e,z_e] = ellipsoid(x,y,z,6,6,6);
    Ellipsoide = surf(x_e,y_e,z_e);
    alpha 0.3;
    Ellipsoide.EdgeColor = 'none';   
          
    % Compute the Root Mean Squred Error
    Err = abs(mean(sqrt(sum((X-Coord_sim).^2))));
    DisEr(i,:)=[Err];
    i = i+1;
    title(['Mean Estimation error is ',num2str(Err),' cm']);
    axis([50-imageSize/2-5 50+imageSize/2+5 50-imageSize/2-5 50+imageSize/2+5 0-5 imageSize/2+5]);
%     axis equal;
    
    pause(1);
end 

meanErr = real(mean(DisEr))*100/num
