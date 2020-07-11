clear all;close all;clc;
data = load('lmh.txt');
sz = size(data);
num = sz(1,1);

T = 1/20;%%采样周期

p = data(:,1);
q = data(:,2);
r = data(:,3);
ax = data(:,4);
ay = data(:,5);
az = data(:,6);
xh = data(:,7);
yh = data(:,8);
zh = data(:,9);
roll_true = data(:,10);
pitch_true = data(:,11);
yaw_true = data(:,12);

ax1=lowp(ax,2,8);
ay1=lowp(ay,2,8);
az1=lowp(az,2,8);
p1=lowp(p,2,8);
q1=lowp(q,2,8);
r1=lowp(r,2,8);

figure;
plot(ned_z,'-r');
hold on;
plot(ned_z_EKF,'-g');

