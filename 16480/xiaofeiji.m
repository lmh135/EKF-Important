%%%%%%%%%%观测方程也用四元数的kalman filter
%%%%%%%%%读取数据%%%%%%%%%%%
clc;
clear all
close all
shuju=load('RN2.txt');
T=1/30;%%采样时间
m=1;
N=18390;%%采样次数
% p=shuju.data(m:m+N-1,2).*3.1415926./180;%%弧度
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

p=shuju(m:m+N-1,1);%%弧度
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
% roll_true_hu=roll_true.*3.1415926./180;%%弧度制
% pitch_true_hu=pitch_true.*3.1415926./180;
% yaw_true_hu=yaw_true.*3.1415926./180;



% %%%由角速度积分得角度值%%%%%%%
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
% %%%角速度积分得到的姿态角与真实角度比较%%%%%%%%%
% figure(13);
% plot(roll_true,'r');
% hold on
% plot(roll_p,'b');
% legend('roll真实值','角速度积分得到的roll');
% figure(14);
% plot(pitch_true,'r');
% hold on
% plot(pitch_q,'b');
% legend('pitch真实值','角速度积分得到的pitch');
% figure(15);
% plot(yaw_true,'r');
% hold on
% plot(yaw_r,'b');
% legend('yaw真实值','角速度积分得到的yaw');
%%%%加速度计算得到的姿态角(弧度)%%%%%%%%%%%%
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

%%偏航角解算%%%%%%%
yaw_acc_hu=zeros(N,1);
%%%非水平情况，结合姿态解算偏航%%%%%%%%
%%%%先结算成0-2pi范围的偏航角
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

%%%%%%电子罗盘解算公式%%%%%%%%%%
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
%%%%%%%%%转换成-pi-pi范围%%%%%%%%%%
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

%弧度角转换为角度
yaw_acc=yaw_acc_hu.*180./3.1415926;
roll_acc=roll_acc_hu.*180./3.1415926;
pitch_acc=pitch_acc_hu.*180./3.1415926;




%%%加速度计算得到的姿态角与角速度积分得姿态角以及真实姿态角对比%%%%
% % figure(16);
% % plot(roll_acc,'r');
% % hold on
% % plot(roll_p,'g');
% % hold on
% % plot(roll_true,'b');
% % legend('加速度得到的roll','角速度积分得到的roll','真实roll');
% % figure(17);
% % plot(pitch_acc,'r');
% % hold on
% % plot(pitch_q,'g');
% % hold on
% % plot(pitch_true,'b');
% % legend('加速度得到的pitch','角速度积分得到的pitch','真实pitch');
% % figure(18);
% % plot(yaw_acc,'r');
% % hold on
% % plot(yaw_r,'g');
% % hold on
% % plot(yaw_true,'b');
% % legend('加速度得到的yaw','角速度积分得到的yaw','真实yaw');
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


%%%%%%%%%姿态转换为四元数%%%%%%%%%%

%测量得到的四元数
q0=cos(roll_acc_hu./2).*cos(pitch_acc_hu./2).*cos(yaw_acc_hu./2)+sin(roll_acc_hu./2).*sin(pitch_acc_hu./2).*sin(yaw_acc_hu./2);
q1=(-1).*cos(roll_acc_hu./2).*sin(pitch_acc_hu./2).*sin(yaw_acc_hu./2)+sin(roll_acc_hu./2).*cos(pitch_acc_hu./2).*cos(yaw_acc_hu./2);
q2=cos(roll_acc_hu./2).*sin(pitch_acc_hu./2).*cos(yaw_acc_hu./2)+sin(roll_acc_hu./2).*cos(pitch_acc_hu./2).*sin(yaw_acc_hu./2);
q3=cos(roll_acc_hu./2).*cos(pitch_acc_hu./2).*sin(yaw_acc_hu./2)-sin(roll_acc_hu./2).*sin(pitch_acc_hu./2).*cos(yaw_acc_hu./2);
% 归一化
qsquare1=sqrt(q0.^2+q1.^2+q2.^2+q3.^2);
q0=q0./qsquare1;
q1=q1./qsquare1;
q2=q2./qsquare1;
q3=q3./qsquare1;
% %%真实四元数
% q0_true=cos(roll_true_hu./2).*cos(pitch_true_hu./2).*cos(yaw_true_hu./2)+sin(roll_true_hu./2).*sin(pitch_true_hu./2).*sin(yaw_true_hu./2);
% q1_true=(-1).*cos(roll_true_hu./2).*sin(pitch_true_hu./2).*sin(yaw_true_hu./2)+sin(roll_true_hu./2).*cos(pitch_true_hu./2).*cos(yaw_true_hu./2);
% q2_true=cos(roll_true_hu./2).*sin(pitch_true_hu./2).*cos(yaw_true_hu./2)+sin(roll_true_hu./2).*cos(pitch_true_hu./2).*sin(yaw_true_hu./2);
% q3_true=cos(roll_true_hu./2).*cos(pitch_true_hu./2).*sin(yaw_true_hu./2)-sin(roll_true_hu./2).*sin(pitch_true_hu./2).*cos(yaw_true_hu./2);
% qsquare1_true=sqrt(q0_true.^2+q1_true.^2+q2_true.^2+q3_true.^2);
% q0_true=q0_true./qsquare1_true;
% q1_true=q1_true./qsquare1_true;
% q2_true=q2_true./qsquare1_true;
% q3_true=q3_true./qsquare1_true;
%%%测量的四元数与真实四元数对比%%%%%%
% figure(19);
% plot(q0,'r');
% hold on
% % plot(q0_true,'b');
% % legend('测量的q0','真实q0');
% figure(20);
% plot(q1,'r');
% hold on
% plot(q1_true,'b');
% legend('测量的q1','真实q1');
% figure(21);
% plot(q2,'r');
% hold on
% plot(q2_true,'b');
% legend('测量的q2','真实q2');
% figure(22);
% plot(q3,'r');
% hold on
% plot(q3_true,'b');
% legend('测量的q3','真实q3');

%%%%%%%EKF%%%%%%%%%%
%%初始化%%
%%%%姿态部分%%%%%%%
x1=[q0';q1';q2';q3';g'];%5*N状态变量矩阵
z1=[q0';q1';q2';q3';g'];

xhat1=zeros(5,N);%状态估计
xhat1(:,1)=x1(:,1);
%%%EKF可调参数阵%%%
Q1=diag([0.3 0.3 0.3 0.3 0.01]);
R1=diag([10 10 10 10 1 ]);
Pk1=diag([0.1 0.1 0.1 0.1 0.1]);



%%%EKF递推过程%%%%


for k=2:N
    
    
    
    %一步预测
     xkk_1hat1=xhat1(:,k-1)+T*fx1(xhat1(:,k-1),p(k-1),q(k-1),r(k-1),0,0,0);
      %四元数归一化
     q00=xkk_1hat1(1);q01=xkk_1hat1(2);q02=xkk_1hat1(3);q03=xkk_1hat1(4);
     qsquare2=sqrt(q00^2+q01^2+q02^2+q03^2);
     q00=q00/qsquare2;
     q01=q01/qsquare2;
     q02=q02/qsquare2;
     q03=q03/qsquare2;
     xkk_1hat1(1)=q00;xkk_1hat1(2)=q01;xkk_1hat1(3)=q02;xkk_1hat1(4)=q03;
    %修正完成    
    

    Hk1=eye(5,5);
    p_kk_1=fx_dot1(xhat1(:,k-1),p(k-1),q(k-1),r(k-1));
    
    Pkk_1=p_kk_1*Pk1*p_kk_1'+Q1;
        
    if (det(Hk1*Pkk_1*Hk1'+R1)~=0)                   %%判断逆是否存在，不存在的话直接用量测值作为估计值
        Kk1=Pkk_1*Hk1'*inv((Hk1*Pkk_1*Hk1'+R1));     %%滤波增益
   
        
    xhat1(:,k)=(xkk_1hat1+Kk1*(z1(:,k)- xkk_1hat1));
       
            %%归一化
            q000=xhat1(1,k);q100=xhat1(2,k);q200=xhat1(3,k);q300=xhat1(4,k);
            qsquare3=sqrt(q000^2+q100^2+q200^2+q300^2);
            q000=q000/qsquare3;
            q100=q100/qsquare3;
            q200=q200/qsquare3;
            q300=q300/qsquare3;
            xhat1(1,k)=q000;xhat1(2,k)=q100;xhat1(3,k)=q200;xhat1(4,k)=q300;
            %修正完成
        Pk1=(eye(5,5)-Kk1*Hk1)*Pkk_1;

      
    else   %%%%逆不存在的情况
     xhat1(:,k)=x1(:,k);
     display(k);
    end
end
%%%EKF结束%%%%%%%%%%%
%估计的四元数提出来,转换成列向量
    q0_out = xhat1(1,:)';
    q1_out = xhat1(2,:)';
    q2_out = xhat1(3,:)';
    q3_out = xhat1(4,:)'; 
    g_out = xhat1(5,:)'; 
%算出姿态角,转换为角度值
    roll_out = atan2((2.*(q2_out.*q3_out+q0_out.*q1_out)),(1-2.*(q1_out.*q1_out+q2_out.*q2_out))).*180./3.14159265;
    theta = 2.*(q1_out.*q3_out-q0_out.*q2_out);
    pitch_out =  -1*asin(theta).*180./3.14159265;
    yaw_out = atan2((2.*(q1_out.*q2_out+q0_out.*q3_out)),(1-2.*(q2_out.*q2_out+q3_out.*q3_out))).*180./3.14159265;
    
    roll_out = medfilt1( roll_out,3);
    pitch_out = medfilt1(pitch_out,3);
    
%%%%%%比较滤波效果%%%%%%%%%%%%%%%
figure(23);
plot(roll_acc,'--b');
hold on
plot(roll_out,'-r');
% hold on
% plot(roll_true,'-g','LineWidth',1);
legend('EKF前测量值','EKF后','真实值');
title('roll滤波效果图');
figure(24);
plot(pitch_acc,'--b');
hold on
plot(pitch_out,'-r');
% hold on
% plot(pitch_true,'-g','LineWidth',1);
legend('EKF前测量值','EKF后','真实值');
title('pitch滤波效果图');
figure(25);
plot(yaw_acc,'--b');
hold on
plot(yaw_out,'-r');
% hold on
% plot(yaw_true,'-g','LineWidth',1);
% legend('EKF前测量值','EKF后','真实值');
title('yaw滤波效果图');
% figure(26);
% plot(q0,'--b');
% hold on
% plot(q0_out,'-r');
% hold on
% plot(q0_true,'-g','LineWidth',1);
% legend('EKF前测量值','EKF后','真实值');
% title('q0滤波效果图');
% figure(27);
% plot(q1,'--b');
% hold on
% plot(q1_out,'-r');
% hold on
% plot(q1_true,'-g','LineWidth',1);
% legend('EKF前测量值','EKF后','真实值');
% title('q1滤波效果图');
% figure(28);
% plot(q2,'--b');
% hold on
% plot(q2_out,'-r');
% hold on
% plot(q2_true,'-g','LineWidth',1);
% legend('EKF前测量值','EKF后','真实值');
% title('q2滤波效果图');
% figure(29);
% plot(q3,'--b');
% hold on
% plot(q3_out,'-r');
% hold on
% plot(q3_true,'-g','LineWidth',1);
% legend('EKF前测量值','EKF后','真实值');
% title('q3滤波效果图');
% figure(30);
% plot(g,'--b');
% hold on
% plot(g_out,'-r');
% legend('EKF前测量值','EKF后');
% title('g滤波效果图');