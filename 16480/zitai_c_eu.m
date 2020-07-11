%%%%%%%欧拉角进行kalman filter%%%%%%%%%%%%%



%%%%%%%%%读取数据%%%%%%%%%%%
clc;
clear all
close all
shuju=importdata('2016-6-3_13_9_42_arm.txt');

T=1/754;%%采样时间
m=1;
N=1839;%%采样次数
% p=shuju.data(m:m+N-1,2).*3.1415926./180;%%弧度
% q=shuju.data(m:m+N-1,3).*3.1415926./180;
% r=shuju.data(m:m+N-1,4).*3.1415926./180;
% ax=shuju.data(m:m+N-1,5).*9.8;
% ay=shuju.data(m:m+N-1,6).*9.8;
% az=shuju.data(m:m+N-1,7).*9.8;
% xh=shuju.data(m:m+N-1,8);
% yh=shuju.data(m:m+N-1,9);
% zh=shuju.data(m:m+N-1,10);
% roll_true=shuju.data(m:m+N-1,15);
% pitch_true=shuju.data(m:m+N-1,16);
% yaw_true=shuju.data(m:m+N-1,17);

p=shuju(m:m+N-1,1);%%弧度
q=-shuju(m:m+N-1,2);
r=-shuju(m:m+N-1,3);
ax=shuju(m:m+N-1,4);
ay=-shuju(m:m+N-1,5);
az=-shuju(m:m+N-1,6);
xh=shuju(m:m+N-1,7);
yh=-shuju(m:m+N-1,8);
zh=-shuju(m:m+N-1,9);
roll_true=shuju(m:m+N-1,10);
pitch_true=-shuju(m:m+N-1,11);
yaw_true=-shuju(m:m+N-1,12);

tic;
roll_true_hu=roll_true.*3.1415926./180;%%弧度制
pitch_true_hu=pitch_true.*3.1415926./180;
yaw_true_hu=yaw_true.*3.1415926./180;






% figure(1);plot(p);
% figure(2);plot(q);
% figure(3);plot(r);
% figure(4);plot(ax);
% figure(5);plot(ay);
% figure(6);plot(az);
% figure(7);plot(xh);
% figure(8);plot(yh);
% figure(9);plot(zh);
% figure(10);plot(roll_true);
% figure(11);plot(pitch_true);
% figure(12);
% plot(yaw_true);
%%%由角速度积分得角度值%%%%%%%
%%roll
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


%%偏航角解算%%%%%%%
yaw_acc_hu=zeros(N,1);
%%%非水平情况，结合姿态解算偏航%%%%%%%%
%%%%先解算成0-2pi范围的偏航角
% for k=1:N
%     if roll_acc_hu(k)<0
%        roll_acc_hu(k)=roll_acc_hu(k)+2*pi;
%     end
% end
% for k=1:N
%     if pitch_acc_hu(k)<0
%         pitch_acc_hu(k)=pitch_acc_hu(k)+2*pi;
%     end
% end
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
% error=yaw_acc_hu(1)-yaw_true(1)/180*3.1415926;
% yaw_acc_hu=yaw_acc_hu-error;

% yaw_acc_hu=atan2(-1.*(yh.*cos(roll_acc_hu)+zh.*sin(roll_acc_hu)),xh.*cos(pitch_acc_hu)+yh.*sin(roll_acc_hu).*sin(pitch_acc_hu)+zh.*cos(roll_acc_hu).*sin(pitch_acc_hu))

error=yaw_acc_hu(1)-yaw_true(1)/180*3.1415926;
yaw_acc_hu=yaw_acc_hu-error;
% %%%%%%%%%转换成-pi-pi范围%%%%%%%%%%
% for k=1:N
%     if roll_acc_hu(k)>pi
%         roll_acc_hu(k)=roll_acc_hu(k)-2*pi;
%     end
%     if roll_acc_hu(k)<=-pi
%         roll_acc_hu(k)=roll_acc_hu(k)+2*pi;
%     end
%     if pitch_acc_hu(k)>pi
%         pitch_acc_hu(k)=pitch_acc_hu(k)-2*pi;
%     end
%     if pitch_acc_hu(k)<=-pi
%         pitch_acc_hu(k)=pitch_acc_hu(k)+2*pi;
%     end
%      if yaw_acc_hu(k)>pi
%         yaw_acc_hu(k)=yaw_acc_hu(k)-2*pi;
%      end
%     if yaw_acc_hu(k)<=-pi
%         yaw_acc_hu(k)=yaw_acc_hu(k)+2*pi;
%      end
% end

%弧度角转换为角度
yaw_acc=yaw_acc_hu.*180./3.1415926;
roll_acc=roll_acc_hu.*180./3.1415926;
pitch_acc=pitch_acc_hu.*180./3.1415926;

% figure(1)
% plot(yaw_acc_hu);

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

   
% for k=2:N
% if (roll_acc_hu(k)-roll_acc_hu(k-1)>0.4)&&(roll_acc_hu(k+1)-roll_acc_hu(k)>0.4)
%     roll_acc_hu(k)=roll_acc_hu(k-1);
% end
% if (pitch_acc_hu(k)-pitch_acc_hu(k-1)>0.4)&&(pitch_acc_hu(k+1)-pitch_acc_hu(k)>0.4)
%     pitch_acc_hu(k)=pitch_acc_hu(k-1);
% end
% if (yaw_acc_hu(k)-yaw_acc_hu(k-1)>0.4)&&(yaw_acc_hu(k+1)-yaw_acc_hu(k)>0.4)
%     yaw_acc_hu(k)=yaw_acc_hu(k-1);
% end
% end
% figure(2);
% plot(yaw_acc_hu);
% yaw_acc=yaw_acc_hu.*180./3.1415926;

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
% figure(18);
% plot(yaw_acc,'r');
% hold on
% plot(yaw_r,'g');
% hold on
% plot(yaw_true,'b');
% % legend('加速度得到的yaw','角速度积分得到的yaw','真实yaw');
% % 
% % 

%%%%%%%EKF%%%%%%%%%%
%%初始化%%
%%%%姿态部分%%%%%%%

x1=[roll_acc_hu';pitch_acc_hu';yaw_acc_hu'];%3*N观测量矩阵
z1=x1;
% z1_c=z1;%%修正观测量
xhat1=zeros(3,N);%状态估计
xhat1(:,1)=x1(:,1);
%%%EKF可调参数阵%%%
Q1=diag([0.01 0.01 0.01]);
R1=diag([5 5 5 ]);
Pk1=diag([0.1 0.1 0.1 ]);



%%%EKF递推过程%%%%
Hk1=eye(3,3);

for k=2:N
%    tic;
    %一步预测
     xkk_1hat1=xhat1(:,k-1)+T*fx1_eu(xhat1(:,k-1),p(k-1),q(k-1),r(k-1),0,0,0);
     
 
     
    
    p_kk_1=fx_dot1_eu(xhat1(:,k-1),p(k-1),q(k-1),r(k-1));
  
    Pkk_1=p_kk_1*Pk1*p_kk_1'+Q1;
 
        
    if (det(Hk1*Pkk_1*Hk1'+R1)~=0)                   %%判断逆是否存在，不存在的话直接用量测值作为估计值
        Kk1=Pkk_1*Hk1'*inv((Hk1*Pkk_1*Hk1'+R1));     %%滤波增益
       
        xhat1(:,k)=xkk_1hat1+Kk1*(z1(:,k)- xkk_1hat1);

        Pk1=(eye(3,3)-Kk1*Hk1)*Pkk_1;

      
    else   %%%%逆不存在的情况
     xhat1(:,k)=x1(:,k);
     display(k);
    end
%     toc;
end

%%%EKF结束%%%%%%%%%%%
%估计的四元数提出来,转换成列向量
    roll_out = xhat1(1,:)'.*180./3.1415926;
    pitch_out = xhat1(2,:)'.*180./3.1415926;
    yaw_out = xhat1(3,:)';
    
%     
    
% for k=2:N
%    while yaw_out(k)> 2*pi 
%        yaw_acc_out(k)=yaw_out(k)-2*pi;
%    end
%    while yaw_out(k)< -2*pi
%        yaw_out(k)=yaw_out(k)+2*pi;
%    end
%    end
%    yaw_out=yaw_out.*180./3.1415926;
%    toc;
%%%%%%比较滤波效果%%%%%%%%%%%%%%%
figure(23);
plot(roll_acc,'--b');
hold on
plot(roll_out,'-r');
hold on
plot(roll_true,'-g','LineWidth',1);
legend('EKF前测量值','EKF后','真实值');
title('roll滤波效果图');
figure(24);
plot(pitch_acc,'--b');
hold on
plot(pitch_out,'-r');
hold on
plot(pitch_true,'-g','LineWidth',1);
legend('EKF前测量值','EKF后','真实值');
title('pitch滤波效果图');
figure(25);
plot(yaw_acc,'--b');
hold on
plot(yaw_out,'-r');
hold on
plot(yaw_true,'-g','LineWidth',1);
legend('EKF前测量值','EKF后','真实值');
title('yaw滤波效果图');
