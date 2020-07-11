clear all;close all;clc;

data = load('armdata.txt');

sz = size(data);
num = sz(1,1);


for i=1:num
    gx(i) = data(i,1);gy(i) = data(i,2);gz(i) = data(i,3);
    ax(i) = data(i,4);ay(i) = data(i,5);az(i) = data(i,6);
    mx(i) = data(i,7);my(i) = data(i,8);mz(i) = data(i,9);
    roll(i) = data(i,10);pitch(i) = data(i,11);yaw(i) = data(i,12);
    ned_x(i) = data(i,13);ned_y(i) = data(i,14);ned_z(i) = data(i,15);
    ned_u(i) = data(i,16);ned_v(i) = data(i,17);ned_w(i) = data(i,18);
    gps_heading(i) = data(i,19);
    gganum(i) = data(i,22);
    vtgnum(i) = data(i,23);
    headingnum(i) = data(i,24);
end


for i=2:num
    if abs(gx(i)-gx(i-1))>3
        gx(i) = gx(i-1);
    end
    if abs(gy(i)-gy(i-1))>5
        gy(i) = gy(i-1);
    end
    if abs(gz(i)-gz(i-1))>5
        gx(i) = gx(i-1);
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
    if abs(roll(i)-roll(i-1))>5
        roll(i) = roll(i-1);
    end
    if abs(pitch(i)-pitch(i-1))>5
        pitch(i) = pitch(i-1);
    end
    if abs(yaw(i)-yaw(i-1))>5
        yaw(i) = yaw(i-1);
    end
end

last_heading = 0;
yaw0 = 0;

for i=1:num
    if(abs(last_heading-gps_heading(i))>0.0001)
        yaw0 = gps_heading(i)-yaw(i);
        yaw1(i) = gps_heading(i);
    else
        yaw1(i) = yaw(i)+yaw0;
    end
    last_heading = gps_heading(i);
end










gx1(1) = gx(1);gy1(1) = gy(1);gz1(1) = gz(1);
ax1(1) = ax(1);ay1(1) = ay(1);az1(1) = gz(1);
mx1(1) = mx(1);my1(1) = my(1);mz1(1) = mz(1);

K1 = 0.1;
K2 = 0.1;
K3 = 0.1;

for i=2:num
    gx1(i) = gx1(i-1)+K1*(gx(i)-gx1(i-1));gy1(i) = gy1(i-1)+K1*(gy(i)-gy1(i-1));gz1(i) = gz1(i-1)+K1*(gz(i)-gz1(i-1));
    ax1(i) = ax1(i-1)+K2*(ax(i)-ax1(i-1));ay1(i) = ay1(i-1)+K2*(ay(i)-ay1(i-1));az1(i) = az1(i-1)+K2*(az(i)-az1(i-1));
    mx1(i) = mx1(i-1)+K3*(mx(i)-mx1(i-1));my1(i) = my1(i-1)+K3*(my(i)-my1(i-1));mz1(i) = mz1(i-1)+K3*(mz(i)-mz1(i-1));
end

for i=1:num
    rol(i) = atan(ay1(i)/sqrt(power(ax1(i),2)+power(az1(i),2)));
    pit(i) = atan(-ax1(i)/sqrt(power(ay1(i),2)+power(az1(i),2)));
    Hx = mx1(i)*cos(pit(i))+my1(i)*sin(pit(i))*sin(rol(i))+mz1(i)*sin(pit(i))*cos(rol(i));
    Hy = my1(i)*cos(rol(i))-mz1(i)*sin(rol(i));
    yw(i) = atan2(Hx,Hy);
    
    rol(i) = rol(i)*57.3;
    pit(i) = pit(i)*57.3;
    yw(i) = yw(i)*57.3;
    if rol(i)>180
        rol(i) = rol(i)-180;
    else if rol(i)<-180
            rol(i) = rol(i)+180;
        end
    end
    if pit(i)>180
        pit(i) = pit(i)-180;
    else if pit(i)<-180
            pit(i) = pit(i)+180;
        end
    end
    if yw(i)>180
        yw(i) = yw(i)-180;
    else if yw(i)<-180
            yw(i) = yw(i)+180;
        end
    end
end


k = 1:num;

% figure;
% plot(k,roll,'b'); 
% hold on;
% plot(k,-rol,'g'); 
% 
% figure;
% plot(k,pitch,'b'); 
% hold on;
% plot(k,-pit,'g'); 
% 
% figure;
% plot(k,yaw1,'b'); 
% hold on;
% plot(k,yw,'g'); 

% figure;
% plot(k,pitch,'b'); 
% hold on;
% plot(k,pit,'g'); 
% 
% figure;
% plot(k,yaw1,'b'); 
% hold on;
% plot(k,yw,'g'); 



plot3(ned_x,ned_y,ned_z);


% for i=1:num
%     g(i) = sqrt(ax(i)*ax(i)+ay(i)*ay(i)+az(i)*az(i));
% end

% figure;
% plot(k,g);
% figure;
% plot(k,ay);
% figure;
% plot(k,az);


