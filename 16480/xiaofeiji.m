%%%%%%%%%%�۲ⷽ��Ҳ����Ԫ����kalman filter
%%%%%%%%%��ȡ����%%%%%%%%%%%
clc;
clear all
close all
shuju=load('RN2.txt');
T=1/30;%%����ʱ��
m=1;
N=18390;%%��������
% p=shuju.data(m:m+N-1,2).*3.1415926./180;%%����
% q=shuju.data(m:m+N-1,3).*3.1415926./180;
% r=shuju.data(m:m+N-1,4).*3.1415926./180;
% ax=shuju.data(m:m+N-1,5).*9.8;
% ay=shuju.data(m:m+N-1,6).*9.8;
% az=shuju.data(m:m+N-1,7).*9.8;
% xh=shuju.data(m:m+N-1,8);
% yh=shuju.data(m:m+N-1,9);
% zh=shuju.data(m:m+N-1,10);
% roll_true=shuju.data(m:m+N-1,11);
% pitch_true=shuju.data(m:m+N-1,12);
% yaw_true=shuju.data(m:m+N-1,13);

p=shuju(m:m+N-1,1);%%����
q=shuju(m:m+N-1,2);
r=shuju(m:m+N-1,3);
ax=shuju(m:m+N-1,4);
ay=shuju(m:m+N-1,5);
az=shuju(m:m+N-1,6);
xh=shuju(m:m+N-1,7);
yh=shuju(m:m+N-1,8);
zh=shuju(m:m+N-1,9);

for k=1:N
    if ax(k)==0
        ax(k)=ax(k-1);
    end
     if ay(k)==0
        ay(k)=ay(k-1);
    end
     if az(k)==0
        az(k)=az(k-1);
    end
end
% roll_true=shuju(m:m+N-1,10);
% pitch_true=-shuju(m:m+N-1,11);
% yaw_true=-shuju(m:m+N-1,12);

% 
% roll_true_hu=roll_true.*3.1415926./180;%%������
% pitch_true_hu=pitch_true.*3.1415926./180;
% yaw_true_hu=yaw_true.*3.1415926./180;



% %%%�ɽ��ٶȻ��ֵýǶ�ֵ%%%%%%%
% %%roll
% roll_p(1)=roll_true(1);
% for k=2:N
%     roll_p(k)=roll_p(k-1)+p(k)*T*180/3.1415926;
% end
% %%pitch
% pitch_q(1)=pitch_true(1);
% for k=2:N
%     pitch_q(k)=pitch_q(k-1)+q(k)*T*180/3.1415926;
% end
% %%yaw
% yaw_r(1)=yaw_true(1);
% for k=2:N
%     yaw_r(k)=yaw_r(k-1)+r(k)*T*180/3.1415926;
%     if yaw_r(k) < -180
%         yaw_r(k)=yaw_r(k)+360;
%     else if yaw_r(k) > 180
%              yaw_r(k)=yaw_r(k)-360;
%         end
%     end
% 
% end
% %%%���ٶȻ��ֵõ�����̬������ʵ�ǶȱȽ�%%%%%%%%%
% figure(13);
% plot(roll_true,'r');
% hold on
% plot(roll_p,'b');
% legend('roll��ʵֵ','���ٶȻ��ֵõ���roll');
% figure(14);
% plot(pitch_true,'r');
% hold on
% plot(pitch_q,'b');
% legend('pitch��ʵֵ','���ٶȻ��ֵõ���pitch');
% figure(15);
% plot(yaw_true,'r');
% hold on
% plot(yaw_r,'b');
% legend('yaw��ʵֵ','���ٶȻ��ֵõ���yaw');
%%%%���ٶȼ���õ�����̬��(����)%%%%%%%%%%%%
g=sqrt(ax.^2+ay.^2+az.^2);
roll_acc_hu=atan2(ay,az);
pitch_acc_hu=-asin(ax./g);

for k=2:N
if (roll_acc_hu(k)-roll_acc_hu(k-1)>0.3)||(roll_acc_hu(k)-roll_acc_hu(k-1)<-0.3)
    roll_acc_hu(k)=roll_acc_hu(k-1);
end
end
figure(1)
plot(roll_acc_hu);

%%ƫ���ǽ���%%%%%%%
yaw_acc_hu=zeros(N,1);
%%%��ˮƽ����������̬����ƫ��%%%%%%%%
%%%%�Ƚ����0-2pi��Χ��ƫ����
for k=1:N
    if roll_acc_hu(k)<0
       roll_acc_hu(k)=roll_acc_hu(k)+2*pi;
    end
end
for k=1:N
    if pitch_acc_hu(k)<0
        pitch_acc_hu(k)=pitch_acc_hu(k)+2*pi;
    end
end
% yaw_acc_hu=atan2(xh./cos(pitch_acc_hu),zh.*sin(roll_acc_hu)-yh.*cos(roll_acc_hu));
% plot(yaw_acc_hu);

%%%%%%�������̽��㹫ʽ%%%%%%%%%%
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
% error=yaw_acc_hu(1)-(-13)/180*3.1415926;
% yaw_acc_hu=yaw_acc_hu-error;
%%%%%%%%%ת����-pi-pi��Χ%%%%%%%%%%
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

%���Ƚ�ת��Ϊ�Ƕ�
yaw_acc=yaw_acc_hu.*180./3.1415926;
roll_acc=roll_acc_hu.*180./3.1415926;
pitch_acc=pitch_acc_hu.*180./3.1415926;




%%%���ٶȼ���õ�����̬������ٶȻ��ֵ���̬���Լ���ʵ��̬�ǶԱ�%%%%
% % figure(16);
% % plot(roll_acc,'r');
% % hold on
% % plot(roll_p,'g');
% % hold on
% % plot(roll_true,'b');
% % legend('���ٶȵõ���roll','���ٶȻ��ֵõ���roll','��ʵroll');
% % figure(17);
% % plot(pitch_acc,'r');
% % hold on
% % plot(pitch_q,'g');
% % hold on
% % plot(pitch_true,'b');
% % legend('���ٶȵõ���pitch','���ٶȻ��ֵõ���pitch','��ʵpitch');
% % figure(18);
% % plot(yaw_acc,'r');
% % hold on
% % plot(yaw_r,'g');
% % hold on
% % plot(yaw_true,'b');
% % legend('���ٶȵõ���yaw','���ٶȻ��ֵõ���yaw','��ʵyaw');
% % 
% % 
offest=0;
for k=2:N
   yaw_acc_hu(k)=yaw_acc_hu(k)+offest;
   if yaw_acc_hu(k)-yaw_acc_hu(k-1)> 0.5*pi
       yaw_acc_hu(k)=yaw_acc_hu(k)-2*pi;
       offest=offest-2*pi;
   end
    if yaw_acc_hu(k)-yaw_acc_hu(k-1)< -0.5*pi
       yaw_acc_hu(k)=yaw_acc_hu(k)+2*pi;
       offest=offest+2*pi;
       end
   end

plot(yaw_acc_hu);


%%%%%%%%%��̬ת��Ϊ��Ԫ��%%%%%%%%%%

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
% %%��ʵ��Ԫ��
% q0_true=cos(roll_true_hu./2).*cos(pitch_true_hu./2).*cos(yaw_true_hu./2)+sin(roll_true_hu./2).*sin(pitch_true_hu./2).*sin(yaw_true_hu./2);
% q1_true=(-1).*cos(roll_true_hu./2).*sin(pitch_true_hu./2).*sin(yaw_true_hu./2)+sin(roll_true_hu./2).*cos(pitch_true_hu./2).*cos(yaw_true_hu./2);
% q2_true=cos(roll_true_hu./2).*sin(pitch_true_hu./2).*cos(yaw_true_hu./2)+sin(roll_true_hu./2).*cos(pitch_true_hu./2).*sin(yaw_true_hu./2);
% q3_true=cos(roll_true_hu./2).*cos(pitch_true_hu./2).*sin(yaw_true_hu./2)-sin(roll_true_hu./2).*sin(pitch_true_hu./2).*cos(yaw_true_hu./2);
% qsquare1_true=sqrt(q0_true.^2+q1_true.^2+q2_true.^2+q3_true.^2);
% q0_true=q0_true./qsquare1_true;
% q1_true=q1_true./qsquare1_true;
% q2_true=q2_true./qsquare1_true;
% q3_true=q3_true./qsquare1_true;
%%%��������Ԫ������ʵ��Ԫ���Ա�%%%%%%
% figure(19);
% plot(q0,'r');
% hold on
% % plot(q0_true,'b');
% % legend('������q0','��ʵq0');
% figure(20);
% plot(q1,'r');
% hold on
% plot(q1_true,'b');
% legend('������q1','��ʵq1');
% figure(21);
% plot(q2,'r');
% hold on
% plot(q2_true,'b');
% legend('������q2','��ʵq2');
% figure(22);
% plot(q3,'r');
% hold on
% plot(q3_true,'b');
% legend('������q3','��ʵq3');

%%%%%%%EKF%%%%%%%%%%
%%��ʼ��%%
%%%%��̬����%%%%%%%
x1=[q0';q1';q2';q3';g'];%5*N״̬��������
z1=[q0';q1';q2';q3';g'];

xhat1=zeros(5,N);%״̬����
xhat1(:,1)=x1(:,1);
%%%EKF�ɵ�������%%%
Q1=diag([0.3 0.3 0.3 0.3 0.01]);
R1=diag([10 10 10 10 1 ]);
Pk1=diag([0.1 0.1 0.1 0.1 0.1]);



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
    
    roll_out = medfilt1( roll_out,3);
    pitch_out = medfilt1(pitch_out,3);
    
%%%%%%�Ƚ��˲�Ч��%%%%%%%%%%%%%%%
figure(23);
plot(roll_acc,'--b');
hold on
plot(roll_out,'-r');
% hold on
% plot(roll_true,'-g','LineWidth',1);
legend('EKFǰ����ֵ','EKF��','��ʵֵ');
title('roll�˲�Ч��ͼ');
figure(24);
plot(pitch_acc,'--b');
hold on
plot(pitch_out,'-r');
% hold on
% plot(pitch_true,'-g','LineWidth',1);
legend('EKFǰ����ֵ','EKF��','��ʵֵ');
title('pitch�˲�Ч��ͼ');
figure(25);
plot(yaw_acc,'--b');
hold on
plot(yaw_out,'-r');
% hold on
% plot(yaw_true,'-g','LineWidth',1);
% legend('EKFǰ����ֵ','EKF��','��ʵֵ');
title('yaw�˲�Ч��ͼ');
% figure(26);
% plot(q0,'--b');
% hold on
% plot(q0_out,'-r');
% hold on
% plot(q0_true,'-g','LineWidth',1);
% legend('EKFǰ����ֵ','EKF��','��ʵֵ');
% title('q0�˲�Ч��ͼ');
% figure(27);
% plot(q1,'--b');
% hold on
% plot(q1_out,'-r');
% hold on
% plot(q1_true,'-g','LineWidth',1);
% legend('EKFǰ����ֵ','EKF��','��ʵֵ');
% title('q1�˲�Ч��ͼ');
% figure(28);
% plot(q2,'--b');
% hold on
% plot(q2_out,'-r');
% hold on
% plot(q2_true,'-g','LineWidth',1);
% legend('EKFǰ����ֵ','EKF��','��ʵֵ');
% title('q2�˲�Ч��ͼ');
% figure(29);
% plot(q3,'--b');
% hold on
% plot(q3_out,'-r');
% hold on
% plot(q3_true,'-g','LineWidth',1);
% legend('EKFǰ����ֵ','EKF��','��ʵֵ');
% title('q3�˲�Ч��ͼ');
% figure(30);
% plot(g,'--b');
% hold on
% plot(g_out,'-r');
% legend('EKFǰ����ֵ','EKF��');
% title('g�˲�Ч��ͼ');