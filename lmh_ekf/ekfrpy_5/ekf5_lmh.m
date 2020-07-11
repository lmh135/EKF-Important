clear;close all;clc;

%% 读入数据
data=load('armdata.txt');
sz = size(data);
num = sz(1,1);
T=1/754;
p = data(:,1);
q = data(:,2);
r = data(:,3);
ax = data(:,4);
ay = data(:,5);
az = data(:,6);
mx = data(:,7);
my = data(:,8);
mz = data(:,9);
roll_imu = data(:,10);
pitch_imu = data(:,11);
yaw_imu = data(:,12);

%% 去除跳变点
for i=2:num
    if abs(p(i)-p(i-1))>3
        p(i) = p(i-1);
    end
    if abs(q(i)-q(i-1))>5
        q(i) = q(i-1);
    end
    if abs(r(i)-r(i-1))>5
        r(i) = r(i-1);
    end
    if abs(ax(i)-ax(i-1))>1
        ax(i) = ax(i-1);
    end
    if abs(ay(i)-ay(i-1))>1
        ay(i) = ay(i-1);
    end
    if abs(az(i)-az(i-1))>2
        az(i) = az(i-1);
    end
    if abs(mx(i)-mx(i-1))>50
        mx(i) = mx(i-1);
    end
    if abs(my(i)-my(i-1))>50
        my(i) = my(i-1);
    end
    if abs(mz(i)-mz(i-1))>50
        mz(i) = mz(i-1);
    end
    if abs(roll_imu(i)-roll_imu(i-1))>5
        roll_imu(i) = roll_imu(i-1);
    end
    if abs(pitch_imu(i)-pitch_imu(i-1))>5
        pitch_imu(i) = pitch_imu(i-1);
    end
    if abs(yaw_imu(i)-yaw_imu(i-1))>5
        yaw_imu(i) = yaw_imu(i-1);
    end
end

%% 低通滤波
ax=lowp(ax,2,8);
ay=lowp(ay,2,8);
az=lowp(az,2,8);
p=lowp(p,2,8);
q=lowp(q,2,8);
r=lowp(r,2,8);

%% 加速度计、磁力计解算姿态角
g=sqrt(ax.^2+ay.^2+az.^2);
roll_acc=atan(ay./az);
pitch_acc=asin(ax./g);
Hy=my.*cos(roll_acc)-mz.*sin(roll_acc);
Hx=mx.*cos(pitch_acc)+my.*sin(roll_acc).*sin(pitch_acc)+mz.*cos(roll_acc).*sin(pitch_acc);
yaw_acc=atan(-Hy./Hx);
% figure;plot(roll_imu,'-r');hold on;plot(roll_acc*57.2958,'-g');
% figure;plot(pitch_imu,'-r');hold on;plot(pitch_acc*57.2958,'-g');
% figure;plot(yaw_imu,'-r');hold on;plot(yaw_acc*57.2958,'-g');

%% 测量得到的四元数及归一化
q4_acc=eu2q4andnormal(roll_acc,pitch_acc,yaw_acc);
q0_acc=q4_acc(:,1);
q1_acc=q4_acc(:,2);
q2_acc=q4_acc(:,3);
q3_acc=q4_acc(:,4);

%% EKF
R_q0_acc=var(q0_acc);%方差阵
R_q1_acc=var(q1_acc);
R_q2_acc=var(q2_acc);
R_q3_acc=var(q3_acc);
R_g_acc=var(g);

X1=[q0_acc';q1_acc';q2_acc';q3_acc';g']; %5*N状态矩阵
Z1=[q0_acc';q1_acc';q2_acc';q3_acc';g'];
%状态误差阵Q  观测误差阵R
Q=diag([R_q0_acc R_q1_acc R_q2_acc R_q3_acc R_g_acc]);
R=diag([R_q0_acc R_q1_acc R_q2_acc R_q3_acc R_g_acc]);
KQ=diag([0.05 0.05 0.05 0.05 1]);
KR=diag([1 1 1 1 1]);
Q=Q*KQ;
R=R*KR;
xhat1=zeros(5,num);%状态估计
xhat1(:,1)=X1(:,1);
Pk1=diag([0.01 0.01 0.01 0.01 0.01]);%初始估计均方误差阵

for k=2:num
    %状态一步预测
    xkk_1hat1=xhat1(:,k-1)+T*fx1(xhat1(:,k-1),p(k-1),q(k-1),r(k-1),0,0,0);
    %对预测的四元数归一化
    q00=xkk_1hat1(1);q01=xkk_1hat1(2);q02=xkk_1hat1(3);q03=xkk_1hat1(4);
    qsquare2=sqrt(q00^2+q01^2+q02^2+q03^2);
    q00=q00/qsquare2;
    q01=q01/qsquare2;
    q02=q02/qsquare2;
    q03=q03/qsquare2;
    xkk_1hat1(1)=q00;xkk_1hat1(2)=q01;xkk_1hat1(3)=q02;xkk_1hat1(4)=q03;
    %一步预测均方误差
    p_kk_1=fx_dot1(xhat1(:,k-1),p(k-1),q(k-1),r(k-1));   
    Pkk_1=p_kk_1*Pk1*p_kk_1'+Q;
    %滤波增益
    Hk1=eye(5,5);
    if (det(Hk1*Pkk_1*Hk1'+R)~=0)                   %%判断逆是否存在，不存在的话直接用量测值作为估计值
        Kk1=Pkk_1*Hk1'/((Hk1*Pkk_1*Hk1'+R));     %%滤波增益
        %状态估计
        xhat1(:,k)=(xkk_1hat1+Kk1*(Z1(:,k)- xkk_1hat1));  %更新       
        q000=xhat1(1,k);q100=xhat1(2,k);q200=xhat1(3,k);q300=xhat1(4,k);
        qsquare3=sqrt(q000^2+q100^2+q200^2+q300^2);
        q000=q000/qsquare3;
        q100=q100/qsquare3;
        q200=q200/qsquare3;
        q300=q300/qsquare3;
        xhat1(1,k)=q000;xhat1(2,k)=q100;xhat1(3,k)=q200;xhat1(4,k)=q300;        %修正完成
        %估计均方误差
        Pk1=(eye(5,5)-Kk1*Hk1)*Pkk_1;
    else   %逆不存在的情况
        xhat1(:,k)=x1(:,k);
        display(k);
    end
end

g_out = xhat1(5,:)'; 
eu = q42eu(xhat1(1,:),xhat1(2,:),xhat1(3,:),xhat1(4,:));
roll_EKF = eu(:,1)*57.2958;
pitch_EKF = eu(:,2)*57.2958;
yaw_EKF = eu(:,3)*57.2958;
figure;plot(roll_imu,'-r');hold on;plot(roll_acc*57.2958,'-g');hold on;plot(roll_EKF,'-b');legend('真实值','EKF前测量值','EKF后');title('roll滤波效果图');
figure;plot(pitch_imu,'-r');hold on;plot(pitch_acc*57.2958,'-g');hold on;plot(pitch_EKF,'-b');
figure;plot(yaw_imu,'-r');hold on;plot(yaw_acc*57.2958,'-g');hold on;plot(yaw_EKF,'-b');


% tuoluo=zeros(5,num);%陀螺预测
% tuoluo(:,1)=X1(:,1);
% for k=2:num
%     tuoluo(:,k)=tuoluo(:,k-1)+T*fx1(tuoluo(:,k-1),p(k-1),q(k-1),r(k-1),0,0,0);
%     q00=tuoluo(1);q01=tuoluo(2);q02=tuoluo(3);q03=tuoluo(4);
%     qsquare2=sqrt(q00^2+q01^2+q02^2+q03^2);
%     q00=q00/qsquare2;
%     q01=q01/qsquare2;
%     q02=q02/qsquare2;
%     q03=q03/qsquare2;
%     tuoluo(1)=q00;tuoluo(2)=q01;tuoluo(3)=q02;tuoluo(4)=q03;   
% end
% eu = q42eu(tuoluo(1,:),tuoluo(2,:),tuoluo(3,:),tuoluo(4,:));
% roll_EKF = eu(:,1)*57.2958;
% pitch_EKF = eu(:,2)*57.2958;
% yaw_EKF = eu(:,3)*57.2958;
% figure;plot(roll_imu,'-r');hold on;plot(roll_acc*57.2958,'-g');hold on;plot(roll_EKF,'-b');legend('真实值','EKF前测量值','EKF后');title('roll滤波效果图');
% figure;plot(pitch_imu,'-r');hold on;plot(pitch_acc*57.2958,'-g');hold on;plot(pitch_EKF,'-b');
% figure;plot(yaw_imu,'-r');hold on;plot(yaw_acc*57.2958,'-g');hold on;plot(yaw_EKF,'-b');

