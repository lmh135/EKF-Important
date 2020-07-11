clear all;close all;clc;

data = load('armdata.txt');

sz = size(data);
num = sz(1,1);

T = 1/200;%%采样周期

p = data(:,1)./180.*3.1415926;
q = data(:,2)./180.*3.1415926;
r = data(:,3)./180.*3.1415926;
ax = data(:,4);
ay = data(:,5);
az = data(:,6);
mx = data(:,7);
my = data(:,8);
mz = data(:,9);
% roll_imu = data(:,10)./180.*3.1415926;
% pitch_imu = data(:,11)./180.*3.1415926;
% yaw_imu = data(:,12)./180.*3.1415926;
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
%%
%%%去除跳变点
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

%%
%%%用GPS输出航向修正IMU输出YAW
last_heading = 0;
yaw0 = 0;
yaw1_imu = zeros(num,1);
for i=1:num
    if(abs(last_heading-gps_heading(i))>0.0001)
        yaw0 = gps_heading(i)-yaw_imu(i);
        yaw1_imu(i) = gps_heading(i);
    else
        yaw1_imu(i) = yaw_imu(i)+yaw0;
    end
    last_heading = gps_heading(i);
end
%%
%%%%%%%对x、y、z线性插值
k=0;
static_x = ned_x(2);
for i=3:num
    if static_x == ned_x(i)
        k = k+1;
        static_x = ned_x(i);
    else
        for j=i-k:i-1
            dx = (ned_x(i)-ned_x(i-k-1))/(k+1);
            ned_x(j) = ned_x(j-1)+dx;
        end
        k=0;
        static_x = ned_x(i);
    end
end
k=0;
static_y = ned_y(2);
for i=3:num
    if static_y == ned_y(i)
        k = k+1;
        static_y = ned_y(i);
    else
        for j=i-k:i-1
            dy = (ned_y(i)-ned_y(i-k-1))/(k+1);
            ned_y(j) = ned_y(j-1)+dy;
        end
        k=0;
        static_y = ned_y(i);
    end
end
k=0;
static_z = ned_z(2);
for i=3:num
    if static_z == ned_z(i)
        k = k+1;
        static_z = ned_z(i);
    else
        for j=i-k:i-1
            dz = (ned_z(i)-ned_z(i-k-1))/(k+1);
            ned_z(j) = ned_z(j-1)+dz;
        end
        k=0;
        static_z = ned_z(i);
    end
end   
%%
%%%%%%%对u、v、w线性插值
k=0;
static_u = ned_u(2);
for i=3:num
    if static_u == ned_u(i)
        k = k+1;
        static_u = ned_u(i);
    else
        for j=i-k:i-1
            du = (ned_u(i)-ned_u(i-k-1))/(k+1);
            ned_u(j) = ned_u(j-1)+du;
        end
        k=0;
        static_u = ned_u(i);
    end
end 
k=0;
static_v = ned_v(2);
for i=3:num
    if static_v == ned_v(i)
        k = k+1;
        static_v = ned_v(i);
    else
        for j=i-k:i-1
            dv = (ned_v(i)-ned_v(i-k-1))/(k+1);
            ned_v(j) = ned_v(j-1)+dv;
        end
        k=0;
        static_v = ned_v(i);
    end
end 
k=0;
static_w = ned_w(2);
for i=3:num
    if static_w == ned_w(i)
        k = k+1;
        static_w = ned_w(i);
    else
        for j=i-k:i-1
            dw = (ned_w(i)-ned_w(i-k-1))/(k+1);
            ned_w(j) = ned_w(j-1)+dw;
        end
        k=0;
        static_w = ned_w(i);
    end
end 
%%
% 
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
%%

g_filter = 9.8*ones(num,1);

R_x=var(ned_x);
R_y=var(ned_y);
R_z=var(ned_z);
R_vx=var(vx);
R_vy=var(vy);
R_vz=var(vz);

R_q0=var(q0_imu);
R_q1=var(q1_imu);
R_q2=var(q2_imu);
R_q3=var(q3_imu);

R_roll=var(roll_imu);
R_pitch=var(pitch_imu);
R_yaw=var(yaw1_imu);

X1 = [ned_x';ned_y';ned_z';vx';vy';vz';q0_imu';q1_imu';q2_imu';q3_imu';g_filter';];%11*N状态矩阵
Z1 = [ned_x';ned_y';ned_z';vx';vy';vz';roll_imu';pitch_imu';yaw1_imu'];
Pk1=diag([0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 ]);

Q=diag([R_x R_y R_z R_vx R_vy R_vz R_q0 R_q1 R_q2  R_q3 0.1 ]);
R=diag([R_x R_y R_z R_vx R_vy R_vz R_roll R_pitch R_yaw ]);

KR=diag([1 1 1 1 1 1 1 1 1]);
KQ=diag([1 1 1 1 1 1 0.1 0.1 0.1 0.1 1 ]);                   

Q=Q*KQ;
R=R*KR;
%%
% alfa = 0.001;
klta = 2;

xhat1=zeros(11,num);%状态估计
xhat1(:,1)=X1(:,2);

sigma = zeros(11,23);
gama_kk_1 = zeros(11,23);

gama_kk_1_z = zeros(9,23);

for k=2:num
    %%%
    sigma(:,1) = xhat1(:,k-1);
    sqrt_Px = sqrt((11+klta)*Pk1);
    for i=2:12
        sigma(:,i) = sigma(:,1) + sqrt_Px(:,i-1);
    end
    for i=13:23
        sigma(:,i) = sigma(:,1) - sqrt_Px(:,i-12);
    end
    W0 = klta/(klta+11);
    Wi = 1/(2*(klta+11));
    %%%
    for i=1:23
        gama_kk_1(:,i) =  xhat1(:,k-1) + T*fx(sigma(:,i),ax(k-1),ay(k-1),az(k-1),p(k-1),q(k-1),r(k-1));
    end
    %%%
    xkk_1hat1 = W0*gama_kk_1(:,1);
    for i=2:23
        xkk_1hat1 = xkk_1hat1 + Wi*gama_kk_1(:,i);
    end
    %%%
    Pkk_1 = W0*((gama_kk_1(:,1)-xkk_1hat1)*(gama_kk_1(:,1)-xkk_1hat1)');
    for i=2:23
        Pkk_1 = Pkk_1 + Wi*((gama_kk_1(:,i)-xkk_1hat1)*(gama_kk_1(:,i)-xkk_1hat1)');
    end    
    Pkk_1 = Pkk_1 + Q;
    %%%
    sigma(:,1) = xkk_1hat1;
    sqrt_Px = sqrt((11+klta)*Pkk_1);
    for i=2:12
        sigma(:,i) = sigma(:,1) + sqrt_Px(:,i-1);
    end
    for i=13:23
        sigma(:,i) = sigma(:,1) - sqrt_Px(:,i-12);
    end
    %%%
    for i=1:23
        gama_kk_1_z(:,i) = hx(sigma(:,i));
    end
    %%%
    zkk_1hat1 = W0*gama_kk_1_z(:,1);
    for i=2:23
        zkk_1hat1 = zkk_1hat1 + Wi*gama_kk_1_z(:,i);
    end
    %%%
    Pzk = W0*((gama_kk_1_z(:,1)-zkk_1hat1)*(gama_kk_1_z(:,1)-zkk_1hat1)');
    for i=2:23
        Pzk = Pzk + Wi*((gama_kk_1_z(:,i)-zkk_1hat1)*(gama_kk_1_z(:,i)-zkk_1hat1)');
    end    
    Pzk = Pzk + R;
    %%%
    Pxkzk = W0*((sigma(:,1)-xkk_1hat1)*(gama_kk_1_z(:,1)-zkk_1hat1)');
    for i=2:23
        Pxkzk = Pxkzk + Wi*((sigma(:,i)-xkk_1hat1)*(gama_kk_1_z(:,i)-zkk_1hat1)');
    end    
    Pxkzk = Pxkzk;    
    %%%
    Kk1 = Pxkzk*inv(Pzk);
    %%%
    xhat1(:,k) = xkk_1hat1 + Kk1*(Z1(:,k)-zkk_1hat1);
    Pk1 = Pkk_1 - Kk1*Pzk*Kk1';
end
    
    
%%%UKF结束%%%%%%%%%%%
q0_UKF=xhat1(7,:)';
q1_UKF=xhat1(8,:)';
q2_UKF=xhat1(9,:)';
q3_UKF=xhat1(10,:)';

q_1=sqrt(q0_UKF.^2+q1_UKF.^2+q2_UKF.^2+q3_UKF.^2);
q0_UKF=q0_UKF./q_1;
q1_UKF=q1_UKF./q_1;
q2_UKF=q2_UKF./q_1;
q3_UKF=q3_UKF./q_1;

eu = q42eu(q0_UKF,q1_UKF,q2_UKF,q3_UKF);

roll_UKF = eu(:,1)*57.3;
pitch_UKF = eu(:,2)*57.3;
yaw_UKF = eu(:,3)*57.3;


ned_x_UKF = xhat1(1,:)';
ned_y_UKF = xhat1(2,:)';
ned_z_UKF = xhat1(3,:)';
vx_UKF = xhat1(4,:)';
vy_UKF = xhat1(5,:)';
vz_UKF = xhat1(6,:)';

kk=1:num;
figure;
plot(kk,ned_x,'-r');
hold on;
plot(kk,ned_x_UKF,'-g');

figure;
plot(kk,ned_y,'-r');
hold on;
plot(kk,ned_y_UKF,'-g');

figure;
plot(kk,ned_z,'-r');
hold on;
plot(kk,ned_z_UKF,'-g');

figure;
plot(kk,vx,'-r');
hold on;
plot(kk,vx_UKF,'-g');

figure;
plot(kk,vy,'-r');
hold on;
plot(kk,vy_UKF,'-g');

figure;
plot(kk,vz,'-r');
hold on;
plot(kk,vz_UKF,'-g');





figure;
plot(kk,roll_imu*57.3,'-r');
hold on;
plot(kk,roll_UKF,'-g');

figure;
plot(kk,pitch_imu*57.3,'-r');
hold on;
plot(kk,pitch_UKF,'-g');

figure;
plot(kk,yaw1_imu*57.3,'-r');
hold on;
plot(kk,yaw_UKF,'-g');
hold on;
plot(kk,yaw_imu*57.3+65.5,'-b');
    
    
    
figure;
plot(kk,q0_imu,'-r');
hold on;
plot(kk,q0_UKF,'-g');

figure;
plot(kk,q1_imu,'-r');
hold on;
plot(kk,q1_UKF,'-g');

figure;
plot(kk,q2_imu,'-r');
hold on;
plot(kk,q2_UKF,'-g');

figure;
plot(kk,q3_imu,'-r');
hold on;
plot(kk,q3_UKF,'-g');





















