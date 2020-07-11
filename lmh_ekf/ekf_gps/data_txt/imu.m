close all;
clear all;
clc;
load a.txt; %读文件，为一个六列矩阵，前三列为陀螺输出角速度，后三列为加计输出
format long  %有效数字十六位
%初始化参数
% vx(1)=0.000048637;
% vy(1)=0.000206947;
% vz(1)=0.007106781;          %初始速度
vx(1)=0;
vy(1)=0;
vz(1)=0;
J(1)=116.344695283*2*pi/360;
L(1)=39.975172*2*pi/360;                     %初始经纬度，经纬度化成弧度
% B(1)=-91.637207*2*pi/360;         %初始航向角   tian
% C(1)=0.120992605*2*pi/360;         %初始俯仰角   dong
% D(1)=0.010445947*2*pi/360;         %初始横滚角   bei %初始姿态角
B(1)=0;
C(1)=0;
D(1)=0;
re=6378245;
Wie=7.27221e-5;
e=1/298.3;  %椭圆度
Ti=1;    %采样时间
j=14;

g=9.78049;

wib_c = a(:,1:3)';   %将a矩阵里的一至三列放入wib_c
f_c = a(:,4:6)';      %将a矩阵里的一至三列放入f_c
wib_x = a(:,1);
wib_y = a(:,2);
wib_z = a(:,3);
fx = a(:,4);
fy = a(:,5);
fz = a(:,6);

for i=1:j;
    Rx=re/(1-e*sin(L(1))^2);        %卯酉圈曲率半径
    Ry=re/(1+2*e-3*e*sin(L(1))^2);  %子午圈主曲率半径
  
    q(1) = cos(B(1)/2)*cos(C(1)/2)*cos(D(1)/2)-sin(B(1)/2)*sin(C(1)/2)*sin(D(1)/2);
    q(2) = cos(B(1)/2)*sin(C(1)/2)*cos(D(1)/2)-sin(B(1)/2)*cos(C(1)/2)*sin(D(1)/2);
    q(3) = cos(B(1)/2)*cos(C(1)/2)*sin(D(1)/2)+sin(B(1)/2)*sin(C(1)/2)*cos(D(1)/2);
    q(4) = cos(B(1)/2)*sin(C(1)/2)*sin(D(1)/2)+sin(B(1)/2)*cos(C(1)/2)*cos(D(1)/2);
    
    T11 = (q(1))^2 + (q(2))^2 - (q(3))^2 - (q(4))^2;
    T12 = 2*q(3)*q(2) + 2*q(1)*q(4); 
    T13 = 2*q(4)*q(2) - 2*q(1)*q(3);
    T21 = 2*q(3)*q(2) - 2*q(1)*q(4);
    T22 = (q(1))^2 - (q(2))^2 + (q(3))^2 - (q(4))^2;
    T23 = 2*q(3)*q(4) - 2*q(1)*q(2);
    T31 = 2*q(4)*q(2) + 2*q(1)*q(3);
    T32 = 2*q(3)*q(4) - 2*q(1)*q(2);
    T33 = (q(1))^2 - (q(2))^2 - (q(3))^2 + (q(4))^2;       
    

    Cbn=[T11 T12 T13;T21 T22 T23;T31 T32 T33];%粗对准后确定的姿态矩阵
    Cnb=Cbn';
    
    gnx=0;
    gny=0;
    gnz=g;
    gn=[gnx;gny;gnz];%3*1
     gb=gn'*Cbn;   %重力加速度在机体系的表示 1*3*3*3=1*3
    Wien_x=0;
    Wien_y=Wie*cos(L(1));
    Wien_z=Wie*sin(L(1));
    Wien=[Wien_x;Wien_y;Wien_z];%3*1
    Wieb=Wien'*Cbn; %地球自转角速度在机体系的表示
    Wibb=[wib_x(i,1) wib_y(i,1) wib_z(i,1)]';%陀螺输出的各个轴表示
    Fb=[fx(i,1) fy(i,1) fz(i,1)]';  %加速度计输出的各个轴表示
%     Wibb=Wien'*Cbn;
%      [Fbx,INSc(i,1) Fby,INSc(i,1) Fbz,INSc(i,1)]=-1*gn'*Cbn;
  %姿态角的计算
    if abs(Cnb(2,2))>1e-10
        if Cnb(2,2)>0
           B(i+1)=atan(Cnb(2,1)/Cnb(2,2));              
        elseif Cnb(2,1)>0
           B(i+1)=atan(Cnb(2,1)/Cnb(2,2))+pi;
        else 
            B(i+1)=atan(Cnb(2,1)/Cnb(2,2))-pi;
        end
    elseif Cnb(2,1)>0
       B(i+1)=pi/2;
    else 
       B(i+1)=-pi/2;
    end                     %求航向角
   
    C(i+1)=asin(Cnb(2,3));   %求俯仰角
    if abs(Cnb(3,3))>1e-10
        if Cnb(3,3)>0
           D(i+1)=atan(-Cnb(1,3)/Cnb(3,3));
        elseif Cnb(1,3)>0
            D(i+1)=atan(-Cnb(1,3)/Cnb(3,3))-pi;
        else
           D(i+1)=atan(-Cnb(1,3)/Cnb(3,3))+pi;
        end
   elseif Cnb(1,3)>0
       D(i+1)=-pi/2;
   else 
       D(i+1)=pi/2;
   end                   %求横滚角
   
    vx(i+1)=(fx(i,1)+2*Wie*sin(L(1))*vy(i)+vx(i)*vy(i)*tan(L(1))/Rx-2*Wie*cos(L(1))*vz(i)-vx(i)*vz(i)/Rx)*Ti+vx(i);%东向速度
    vy(i+1)=(fy(i,1)-2*Wie*sin(L(1))*vx(i)-vx(i)*vx(i)*tan(L(1))/Rx-vy(i)*vz(i)/Rx)*Ti+vy(i);%北向速度
    vz(i+1)=(fz(i,1)+(2*Wie*cos(L(1))+vx(i)/Rx)*vx(i)+vy(i)*vy(i)/Ry-g)*Ti+vz(i);%天向速度
    L(i+1)=vy(i)*Ti/Ry+L(i);                    %纬度
    J(i+1)=vx(i)*Ti/(Rx*cos(L(i)))+J(i);        %经度
     
    

end


 t=0:1:j;
  figure(1)
  plot(J,L);xlabel('经度'),ylabel('纬度');   %输出图形
  figure(2)
  plot(L,'g');xlabel('时间'),ylabel('纬度');
  figure(3)
  plot(vx,'b');xlabel('时间'),ylabel('东向速度');
  figure(4)
  plot(vy,'b');xlabel('时间'),ylabel('北向速度');
  figure(5)
  plot(vz,'b');xlabel('t'),ylabel('天向速度');
  figure(6)
  plot(B,'b');xlabel('时间'),ylabel('航向角');
  figure(7)
  plot(C,'g');xlabel('时间'),ylabel('俯仰角');
  figure(8)
  plot(D,'r');xlabel('时间'),ylabel('横滚角');