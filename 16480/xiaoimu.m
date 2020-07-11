clc;
close all
clear all
shuju=load('xiaoimu.txt');
N=6189;
p=shuju(1:N,1).*3.1415926./180;
q=shuju(1:N,2).*3.1415926./180;
r=shuju(1:N,3).*3.1415926./180;
ax=shuju(1:N,4);
ay=shuju(1:N,5);
az=shuju(1:N,6);
xh=shuju(1:N,7);
yh=shuju(1:N,8);
zh=shuju(1:N,9);
T=1/150;


g=sqrt(ax.^2+ay.^2+az.^2);
roll_acc_hu=atan2(ay,az);
pitch_acc_hu=-asin(ax./g);

roll_acc=roll_acc_hu.*180./3.1415926;
pitch_acc=pitch_acc_hu.*180./3.1415926;
figure(1);
plot(roll_acc);
figure(2);
plot(pitch_acc);


yaw_acc_hu=zeros(N,1);
Xh=xh.*cos(pitch_acc_hu)+yh.*sin(roll_acc_hu).*sin(pitch_acc_hu)-zh.*cos(roll_acc_hu).*sin(pitch_acc_hu);
Yh=yh.*cos(roll_acc_hu)+zh.*sin(roll_acc_hu);
for k=1:N
if Xh(k)<0
    yaw_acc_hu(k)=pi-atan(Yh(k)/Xh(k));
else if ((Xh(k)>0) && (Yh(k)<0))
        yaw_acc_hu(k)=-atan(Yh(k)/Xh(k));
    else if Yh(k)>0
             yaw_acc_hu(k)=2*pi-atan(Yh(k)/Xh(k));
    end
end
end
if Xh(k)==0
    if Yh(k)<0
         yaw_acc_hu(k)=pi/2;
    else if Yh(k)>0
             yaw_acc_hu(k)=3*pi/2;
        end
    end
end
end

for k=1:N
    if roll_acc_hu(k)>pi
        roll_acc_hu(k)=roll_acc_hu(k)-2*pi;
    end
    if roll_acc_hu(k)<=-pi
        roll_acc_hu(k)=roll_acc_hu(k)+2*pi;
    end
    if pitch_acc_hu(k)>pi
        pitch_acc_hu(k)=pitch_acc_hu(k)-2*pi;
    end
    if pitch_acc_hu(k)<=-pi
        pitch_acc_hu(k)=pitch_acc_hu(k)+2*pi;
    end
     if yaw_acc_hu(k)>pi
        yaw_acc_hu(k)=yaw_acc_hu(k)-2*pi;
     end
    if yaw_acc_hu(k)<=-pi
        yaw_acc_hu(k)=yaw_acc_hu(k)+2*pi;
     end
end
yaw_acc=yaw_acc_hu.*180./3.1415926;
figure(3);
plot(yaw_acc);

%�����õ�����Ԫ��
q0=cos(roll_acc_hu./2).*cos(pitch_acc_hu./2).*cos(yaw_acc_hu./2)+sin(roll_acc_hu./2).*sin(pitch_acc_hu./2).*sin(yaw_acc_hu./2);
q1=(-1).*cos(roll_acc_hu./2).*sin(pitch_acc_hu./2).*sin(yaw_acc_hu./2)+sin(roll_acc_hu./2).*cos(pitch_acc_hu./2).*cos(yaw_acc_hu./2);
q2=cos(roll_acc_hu./2).*sin(pitch_acc_hu./2).*cos(yaw_acc_hu./2)+sin(roll_acc_hu./2).*cos(pitch_acc_hu./2).*sin(yaw_acc_hu./2);
q3=cos(roll_acc_hu./2).*cos(pitch_acc_hu./2).*sin(yaw_acc_hu./2)-sin(roll_acc_hu./2).*sin(pitch_acc_hu./2).*cos(yaw_acc_hu./2);
% ��һ��
qsquare1=sqrt(q0.^2+q1.^2+q2.^2+q3.^2);
q0=q0./qsquare1;
q1=q1./qsquare1;
q2=q2./qsquare1;
q3=q3./qsquare1;

x1=[q0';q1';q2';q3';g'];%5*N״̬��������
z1=[q0';q1';q2';q3';g'];
xhat1=zeros(5,N);%״̬����
xhat1(:,1)=x1(:,1);
%%%EKF�ɵ�������%%%
Q1=diag([0.06 0.1 0.1 0.06 0.1]);
R1=diag([1 0.8 0.8 1 1 ]);
Pk1=diag([0.01 0.01 0.01 0.01 0.01]);

%%%EKF���ƹ���%%%%


for k=2:N
    
    
    
    %һ��Ԥ��
     xkk_1hat1=xhat1(:,k-1)+T*fx1(xhat1(:,k-1),p(k-1),q(k-1),r(k-1),0,0,0);
      %��Ԫ����һ��
     q00=xkk_1hat1(1);q01=xkk_1hat1(2);q02=xkk_1hat1(3);q03=xkk_1hat1(4);
     qsquare2=sqrt(q00^2+q01^2+q02^2+q03^2);
     q00=q00/qsquare2;
     q01=q01/qsquare2;
     q02=q02/qsquare2;
     q03=q03/qsquare2;
     xkk_1hat1(1)=q00;xkk_1hat1(2)=q01;xkk_1hat1(3)=q02;xkk_1hat1(4)=q03;
    %�������    
    

    Hk1=eye(5,5);
    p_kk_1=fx_dot1(xhat1(:,k-1),p(k-1),q(k-1),r(k-1));
    
    Pkk_1=p_kk_1*Pk1*p_kk_1'+Q1;
        
    if (det(Hk1*Pkk_1*Hk1'+R1)~=0)                   %%�ж����Ƿ���ڣ������ڵĻ�ֱ��������ֵ��Ϊ����ֵ
        Kk1=Pkk_1*Hk1'*inv((Hk1*Pkk_1*Hk1'+R1));     %%�˲�����

    xhat1(:,k)=(xkk_1hat1+Kk1*(z1(:,k)- xkk_1hat1));
       
            %%��һ��
            q000=xhat1(1,k);q100=xhat1(2,k);q200=xhat1(3,k);q300=xhat1(4,k);
            qsquare3=sqrt(q000^2+q100^2+q200^2+q300^2);
            q000=q000/qsquare3;
            q100=q100/qsquare3;
            q200=q200/qsquare3;
            q300=q300/qsquare3;
            xhat1(1,k)=q000;xhat1(2,k)=q100;xhat1(3,k)=q200;xhat1(4,k)=q300;
            %�������
        Pk1=(eye(5,5)-Kk1*Hk1)*Pkk_1;

      
    else   %%%%�治���ڵ����
     xhat1(:,k)=x1(:,k);
     display(k);
    end
    display(Pk1);
end
%%%EKF����%%%%%%%%%%%
%���Ƶ���Ԫ�������,ת����������
    q0_out = xhat1(1,:)';
    q1_out = xhat1(2,:)';
    q2_out = xhat1(3,:)';
    q3_out = xhat1(4,:)'; 
    g_out = xhat1(5,:)'; 
%�����̬��,ת��Ϊ�Ƕ�ֵ
    roll_out = atan2((2.*(q2_out.*q3_out+q0_out.*q1_out)),(1-2.*(q1_out.*q1_out+q2_out.*q2_out))).*180./3.14159265;
    theta = 2.*(q1_out.*q3_out-q0_out.*q2_out);
    pitch_out =  -1*asin(theta).*180./3.14159265;
    yaw_out = atan2((2.*(q1_out.*q2_out+q0_out.*q3_out)),(1-2.*(q2_out.*q2_out+q3_out.*q3_out))).*180./3.14159265;
    
 
   t=0:1:N ;
  y=13.5+20*sin(5*t-2.7);
  figure(30);
  plot(t,y);
  
    %%%%%%�Ƚ��˲�Ч��%%%%%%%%%%%%%%%
%      k=1:N;
% figure(23);
% plot(T*k,roll_acc(k),'--b',T*k,roll_out(k),'-r');
% hold on
% plot(0.01*k,roll_out(k),'-r');
% hold on
% plot(roll_true,'-g','LineWidth',1);
% legend('EKFǰ����ֵ','EKF��','��ʵֵ');
% title('roll�˲�Ч��ͼ');
figure(24);
plot(pitch_acc,'--b');
hold on
plot(pitch_out,'-r');
% % hold on
% % plot(pitch_true,'-g','LineWidth',1);
legend('EKFǰ����ֵ','EKF��');
title('pitch�˲�Ч��ͼ');
figure(25);
plot(yaw_acc,'--b');
hold on
plot(yaw_out,'-r');
% hold on
% % plot(yaw_true,'-g','LineWidth',1);
legend('EKFǰ����ֵ','EKF��');
title('yaw�˲�Ч��ͼ');
% figure(26);
% plot(q0,'--b');
% hold on
% plot(q0_out,'-r');
% % hold on
% % plot(q0_true,'-g','LineWidth',1);
% legend('EKFǰ����ֵ','EKF��','��ʵֵ');
% title('q0�˲�Ч��ͼ');
% figure(27);
% plot(q1,'--b');
% hold on
% plot(q1_out,'-r');
% % hold on
% % plot(q1_true,'-g','LineWidth',1);
% legend('EKFǰ����ֵ','EKF��','��ʵֵ');
% title('q1�˲�Ч��ͼ');
% figure(28);
% plot(q2,'--b');
% hold on
% plot(q2_out,'-r');
% % hold on
% % plot(q2_true,'-g','LineWidth',1);
% legend('EKFǰ����ֵ','EKF��','��ʵֵ');
% title('q2�˲�Ч��ͼ');
% figure(29);
% plot(q3,'--b');
% hold on
% plot(q3_out,'-r');
% % hold on
% % plot(q3_true,'-g','LineWidth',1);
% legend('EKFǰ����ֵ','EKF��','��ʵֵ');
% title('q3�˲�Ч��ͼ');
% figure(30);
% plot(g,'--b');
% hold on
% plot(g_out,'-r');
% legend('EKFǰ����ֵ','EKF��');
% title('g�˲�Ч��ͼ');
