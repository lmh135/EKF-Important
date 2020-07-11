clear;close all;clc;

%% 读入数据
data = load('armdata.txt');
sz = size(data);
num = sz(1,1);
T = 1/752; %采样周期
p = data(:,1)./180.*3.1415926;
q = data(:,2)./180.*3.1415926;
r = data(:,3)./180.*3.1415926;
ax = data(:,4);
ay = data(:,5);
az = data(:,6);
mx = data(:,7);
my = data(:,8);
mz = data(:,9);
roll_imu = data(:,10);
pitch_imu = data(:,11);
yaw_imu = data(:,12);
ned_x = data(:,13);
ned_y = data(:,14);
ned_z = data(:,15);
ned_u = data(:,16);
ned_v = data(:,17);
ned_w = data(:,18);
gps_heading = data(:,19)./180.*3.1415926;

ned_xt = data(:,13);
ned_yt = data(:,14);
ned_zt = data(:,15);
ned_ut = data(:,16);
ned_vt = data(:,17);
ned_wt = data(:,18);

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
roll_imu = roll_imu./180.*3.1415926;
pitch_imu = pitch_imu./180.*3.1415926;
yaw_imu = yaw_imu./180.*3.1415926;

%% 用GPS输出航向修正IMU输出YAW
last_heading = 0;yaw0 = 0;yaw1_imu = zeros(num,1);
for i=1:num
    if(abs(last_heading-gps_heading(i))>0.0001)
        yaw0 = gps_heading(i)-yaw_imu(i);
        yaw1_imu(i) = gps_heading(i);
    else
        yaw1_imu(i) = yaw_imu(i)+yaw0;
    end
    last_heading = gps_heading(i);
end
% figure;plot(yaw1_imu,'-r');hold on;plot(gps_heading,'-g');hold on;plot(yaw_imu,'-b');
 
%% 初始化一些数组，数据处理用
p_filter=zeros(num,1);
q_filter=zeros(num,1);
r_filter=zeros(num,1);
ax_filter=zeros(num,1);
ay_filter=zeros(num,1);
az_filter=zeros(num,1);
mx_filter=zeros(num,1);
my_filter=zeros(num,1);
mz_filter=zeros(num,1);
roll_hu=zeros(num,1);
pitch_hu=zeros(num,1);
yaw_hu=zeros(num,1);
roll_hu_filter=zeros(num,1);
pitch_hu_filter=zeros(num,1);
yaw_hu_filter=zeros(num,1);

%% 低通滤波
p_filter(1)=p(1);
q_filter(1)=q(1);
r_filter(1)=r(1);
ax_filter(1)=ax(1);
ay_filter(1)=ay(1);
az_filter(1)=az(1);
mx_filter(1)=mx(1);
my_filter(1)=my(1);
mz_filter(1)=mz(1);
for k=2:num
    p_filter(k)=p_filter(k-1)+0.1*(p(k)-p_filter(k-1));
    q_filter(k)=q_filter(k-1)+0.1*(q(k)-q_filter(k-1));
    r_filter(k)=r_filter(k-1)+0.1*(r(k)-r_filter(k-1));
    ax_filter(k)=ax_filter(k-1)+0.1*(ax(k)-ax_filter(k-1));
    ay_filter(k)=ay_filter(k-1)+0.1*(ay(k)-ay_filter(k-1));
    az_filter(k)=az_filter(k-1)+0.1*(az(k)-az_filter(k-1));
    mx_filter(k)=mx_filter(k-1)+0.05*(mx(k)-mx_filter(k-1));
    my_filter(k)=my_filter(k-1)+0.05*(my(k)-my_filter(k-1));
    mz_filter(k)=mz_filter(k-1)+0.05*(mz(k)-mz_filter(k-1));
end
% figure;plot(p,'-r');hold on;plot(p_filter,'-b');

%% 对x、y、z线性插值
k=0;static_x = ned_x(2);
for i=3:num
    if static_x == ned_x(i)
        k = k+1;
    else
        dx = (ned_x(i)-ned_x(i-k-1))/(k+1);
        for j=i-k:i-1
            ned_x(j) = ned_x(j-1)+dx;
        end
        k=0;static_x = ned_x(i);
    end
end
k=0;static_y = ned_y(2);
for i=3:num
    if static_y == ned_y(i)
        k = k+1;
    else
        dy = (ned_y(i)-ned_y(i-k-1))/(k+1);
        for j=i-k:i-1
            ned_y(j) = ned_y(j-1)+dy;
        end
        k=0;static_y = ned_y(i);
    end
end
k=0;static_z = ned_z(2);
for i=3:num
    if static_z == ned_z(i)
        k = k+1;
    else
        dz = (ned_z(i)-ned_z(i-k-1))/(k+1);
        for j=i-k:i-1
            ned_z(j) = ned_z(j-1)+dz;
        end
        k=0;static_z = ned_z(i);
    end
end   

%% 对u、v、w线性插值
k=0;static_u = ned_u(2);
for i=3:num
    if static_u == ned_u(i)
        k = k+1;
    else
        du = (ned_u(i)-ned_u(i-k-1))/(k+1);
        for j=i-k:i-1
            ned_u(j) = ned_u(j-1)+du;
        end
        k=0;static_u = ned_u(i);
    end
end 
k=0;static_v = ned_v(2);
for i=3:num
    if static_v == ned_v(i)
        k = k+1;
    else
        dv = (ned_v(i)-ned_v(i-k-1))/(k+1);
        for j=i-k:i-1
            ned_v(j) = ned_v(j-1)+dv;
        end
        k=0;static_v = ned_v(i);
    end
end 
k=0;static_w = ned_w(2);
for i=3:num
    if static_w == ned_w(i)
        k = k+1;
    else
        dw = (ned_w(i)-ned_w(i-k-1))/(k+1);
        for j=i-k:i-1
            ned_w(j) = ned_w(j-1)+dw;
        end
        k=0;static_w = ned_w(i);
    end
end 

%% 四元数归一化 
q4_imu=eu2q4(roll_imu,pitch_imu,yaw1_imu);
q0_imu=q4_imu(:,1);
q1_imu=q4_imu(:,2);
q2_imu=q4_imu(:,3);
q3_imu=q4_imu(:,4);
q_1=sqrt(q0_imu.^2+q1_imu.^2+q2_imu.^2+q3_imu.^2);
q0_imu=q0_imu./q_1;
q1_imu=q1_imu./q_1;
q2_imu=q2_imu./q_1;
q3_imu=q3_imu./q_1;

vx = zeros(num,1);
vy = zeros(num,1);
vz = zeros(num,1);
for i=1:num
    nCb = q4_2_nCb(q0_imu(i),q1_imu(i),q2_imu(i),q3_imu(i));
    V = nCb*[ned_u(i); ned_v(i); ned_w(i)];
    vx(i) = V(1);
    vy(i) = V(2);
    vz(i) = V(3);
end
% figure;plot(ned_u,'-r');hold on;plot(vx,'-b');

% g_filter = 9.8*ones(num,1);
g_filter = sqrt(ax_filter.^2+ay_filter.^2+az_filter.^2);
R_x=var(ned_xt);
R_y=var(ned_yt);
R_z=var(ned_zt);
R_vx=var(ned_ut);
R_vy=var(ned_vt);
R_vz=var(ned_wt);

% R_vx=var(vx);
% R_vy=var(vy);
% R_vz=var(vz);
% X1 = [ned_xt';ned_yt';ned_zt';vx';vy';vz'];%6*N状态矩阵
% Z1 = [ned_xt';ned_yt';ned_zt';vx';vy';vz'];
X1 = [ned_xt';ned_yt';ned_zt';ned_ut';ned_vt';ned_wt'];%6*N状态矩阵
Z1 = [ned_xt';ned_yt';ned_zt';ned_ut';ned_vt';ned_wt'];
%状态误差阵Q  观测误差阵R
Q=diag([R_x R_y R_z R_vx R_vy R_vz]);
R=diag([R_x R_y R_z R_vx R_vy R_vz]);

KQ=diag([0.1 0.1 0.1 1 1 1]);  
KR=diag([1 1 1 1 1 1]);
Q=Q*KQ;
R=R*KR;
Pk1=diag([0.1 0.1 0.1 0.1 0.1 0.1]);
xhat1=zeros(6,num);%状态估计
xhat1(:,1)=X1(:,1);

for k=2:num
    %状态一步预测
    xkk_1hat1=xhat1(:,k-1)+T*fx(xhat1(:,k-1),q0_imu(k-1),q1_imu(k-1),q2_imu(k-1),q3_imu(k-1),ax_filter(k-1),ay_filter(k-1),az_filter(k-1),p_filter(k-1),q_filter(k-1),r_filter(k-1),g_filter(k-1));
    %一步预测均方误差
    fxdot_kk_1=fx_dot(q0_imu(k-1),q1_imu(k-1),q2_imu(k-1),q3_imu(k-1),p_filter(k-1),q_filter(k-1),r_filter(k-1));%状态转移矩阵（雅克比矩阵）    
    Pkk_1=fxdot_kk_1*Pk1*fxdot_kk_1'+Q;
    %滤波增益
    Hk1=eye(6,6);  
    if(det(Hk1*Pkk_1*Hk1'+R)~=0)                  %判断逆是否存在，不存在的话直接用量测值作为估计值
         Kk1=Pkk_1*Hk1'/((Hk1*Pkk_1*Hk1'+R));   %滤波增益
         %状态估计
         xhat1(:,k)=xkk_1hat1+Kk1*(Z1(:,k)- xkk_1hat1);   %更新
         %估计均方误差
         Pk1=(eye(6,6)-Kk1*Hk1)*Pkk_1;        
    else
        xhat1(:,k)=X1(:,k);
        display(k);
    end
end

%% EKF结束
ned_x_EKF = xhat1(1,:);
ned_y_EKF = xhat1(2,:);
ned_z_EKF = xhat1(3,:);

figure;plot(ned_xt,'-r');hold on;plot(ned_x_EKF,'-g');legend('GPS值','EKF后');title('ned_x滤波效果图');
figure;plot(ned_yt,'-r');hold on;plot(ned_y_EKF,'-g');hold on;plot(ned_y,'-b');
figure;plot(ned_zt,'-r');hold on;plot(ned_z_EKF,'-g');hold on;plot(ned_z,'-b');