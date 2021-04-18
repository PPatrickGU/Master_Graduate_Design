% 从Excel中读取数据
[NUM] = xlsread('E:\Graduate_Design\Arduino\arduino_data\4.16_15.21.29-4.16_15.22.04.xls'); 
N = length(NUM);
Fs = 92;
figure(1);
plot(NUM);
xlabel('Time'); 
ylabel('Amplitude'); 

% FFT时域变频域
y0 = abs(fft(NUM));
N=length(NUM);
f = (0:N-1)*Fs/N;
figure(2);
plot(f,y0);
xlabel('Frequency'); 
ylabel('Amplitude'); 

