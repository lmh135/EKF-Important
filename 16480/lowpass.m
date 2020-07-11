function f=lowpass(x)
 Wp=8*pi;
 Ws=12*pi;
 Rp=3;
 Rs=15;%��Ƶ�ͨ�˲�����
 Fs=50;
[N,Wc]=buttord(Wp, Ws, Rp, Rs,'s');  
 %����õ�Butterworth��ͨ�˲�������С����N��3dB��ֹƵ��Wc
 %����������˲���û�С�s��,�Ҳ���Ϊ���ֹ�һ�������ԦУ�֮��Ĳ���
[Cs,Ca]=butter(N,Wc,'low','s');
%����������˲���û�С�s��
[b,a]=bilinear(Cs,Ca,Fs);
f=filter(b,a,x);