clear all;close all;clc;
data = load('lmh.txt');
sz = size(data);
num = sz(1,1);

T = 1/20;%%采样周期

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
% ned_x = data(:,13);
% ned_y = data(:,14);
% ned_z = data(:,15);
% ned_u = data(:,16);
% ned_v = data(:,17);
% ned_w = data(:,18);
% gps_heading = data(:,19)./180.*3.1415926;

figure;
plot(roll_imu,'-b');
figure;
plot(pitch_imu,'-b');
figure;
plot(yaw_imu,'-b');


