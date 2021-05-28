    close all;clear;clc;     
figure('Position',[71 239 921 420]);
set(gcf,'DoubleBuffer','on','NumberTitle','off','name',...
    'Simulation experiment with dodging handicaps by robot');
axes('Position',[0.05,0.3,0.9,0.4]);
% Author's email:zjliu2001@163.com
xlim([0,400]);ylim([0,80]);hold on; box on;
for k=10:10:400;
    plot([k,k],[0,80],'k');
end
for k=10:10:80;
    plot([0,400],[k,k],'k');
end
set(gca,'ytick',[]);
title('Simulation experiment with dodging handicaps by robot');
fill([86,94,94,86,86],[28,28,40,40,28],'g','EdgeColor','r');
fill([196,204,204,196,196],[38,38,60,60,38],'b','EdgeColor','M');
fill([196,204,204,196,196],[7,7,16,16,7],'c','EdgeColor','M');
fill([296,304,304,296,296],[30,30,55,55,30],'M','EdgeColor','M');
Cf1=inline('[[x>85]&[x<95]]&[[y<41]&[y>27]]','x','y');
Cf2=inline('[[x>195]&[x<205]]&[[y<61]&[y>37]]','x','y');
Cf3=inline('[[x>195]&[x<205]]&[[y<17]&[y>6]]','x','y');
Cf4=inline('[[x>295]&[x<305]]&[[y<56]&[y>29]]','x','y');
zmn=[0,0,0,0];
zm1=10+40i;zm2=42i;zm3=38i;zm=[zm1,zm2,zm3,zm1];
fill(real(zm),imag(zm),'M','EdgeColor','M');
A=0; % angle of direction
p=[1,-1,-1];t=1;R=0;
while real(zm(1))<400;
    zc=mean2(zm);
    zmn=zc+[zm-zc]*exp(i*R)+10*exp(i*A);
    if Cf1(real(zmn(1)),imag(zmn(1)))|Cf1(real(zmn(2)),imag(zmn(2)))|...
            Cf1(real(zmn(3)),imag(zmn(3)))|Cf1(real(zmn(4)),imag(zmn(4)));
        zc=mean2(zmn);zmn=zc+[zmn-zc]*exp(i*[A+pi/4*p(1)]);
        zmn=zmn+5i*([imag(zmn(1))>40]*2-1);A=A+pi/4;
    end
        if Cf2(real(zmn(1)),imag(zmn(1)))|Cf2(real(zmn(2)),imag(zmn(2)))|...
            Cf2(real(zmn(3)),imag(zmn(3)))|Cf2(real(zmn(4)),imag(zmn(4)));
        zc=mean2(zmn);zmn=zc+[zmn-zc]*exp(i*[A+pi/4*p(2)]);
        zmn=zmn+5i*([imag(zmn(1))>40]*2-1);A=A-pi/4;
    end
    if Cf3(real(zmn(1)),imag(zmn(1)))|Cf3(real(zmn(2)),imag(zmn(2)))|...
            Cf3(real(zmn(3)),imag(zmn(3)))|Cf3(real(zmn(4)),imag(zmn(4)));
        zc=mean2(zmn);zmn=zc+[zmn-zc]*exp(i*[A+pi/4*p(t)]);
        zmn=zmn+5i*([imag(zmn(1))>40]*2-1);t=t+1;
    end
    if Cf4(real(zmn(1)),imag(zmn(1)))|Cf4(real(zmn(2)),imag(zmn(2)))|...
            Cf4(real(zmn(3)),imag(zmn(3)))|Cf4(real(zmn(4)),imag(zmn(4)));
        zc=mean2(zmn);zmn=zc+[zmn-zc]*exp(i*[A+pi/2*p(3)])-5;
        zmn=zmn+5i*([imag(zmn(1))>40]*2-1);A=pi/2;
    end
    if abs(imag(zmn(1))*2-imag(zmn(2))-imag(zmn(3)))<1;
        zmn=zmn-[imag(zmn(1))-40]*i;
    end
    fill(real(zmn),imag(zmn),'M','EdgeColor','M');
    zm=zmn;
    if abs(A)>0.5 & R==0 &A<1;
        R=pi/2*[1-[A>0]*2];A=0;
    elseif abs(A)==0 & R~=0 & abs(imag(zm(1))-40)>0.1 & real(zmn(1))<220;
        R=-R/2;A=0;
    elseif abs(A)==0 & R~=0 & abs(imag(zm(1))-40)<0.1;
        R=0;A=0;
    elseif abs(A)>1& R==0;
        A=-pi/3;R=pi/4;
    elseif abs(A)>1 & R>0.5;
        A=0;R=pi/2;
    elseif A==0 & R>1.5;
        A=0;R=-pi/4;
    end
    pause(0.2);
end

     

