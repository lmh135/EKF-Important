function f=lowpass(x)
 Wp=8*pi;
 Ws=12*pi;
 Rp=3;
 Rs=15;%设计低通滤波器：
 Fs=50;
[N,Wc]=buttord(Wp, Ws, Rp, Rs,'s');  
 %估算得到Butterworth低通滤波器的最小阶数N和3dB截止频率Wc
 %如果是数字滤波，没有‘s’,且参数为数字归一化（除以π）之后的参数
[Cs,Ca]=butter(N,Wc,'low','s');
%如果是数字滤波，没有‘s’
[b,a]=bilinear(Cs,Ca,Fs);
f=filter(b,a,x);