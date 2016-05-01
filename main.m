%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%程序初始化操作%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;
close all;

load model ZM;
iBeacon_IMU_KF_location = KF(10);
GPS_IMU_KF_location = KF(100);
IMM_KF_location = IMM_KF();

iBeacon_IMU_PF_location = PF(10);
GPS_IMU_PF_location = PF(100);
IMM_PF_location = IMM_PF();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%全局变量定义%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
outdoor_sensor_data=361;
indoor_sensor_data=0;
sensor_data=outdoor_sensor_data+indoor_sensor_data;

[groundtruthx,groundtruthy,heading,velocity]=Ground_Truth();
groundtruth = [groundtruthx,groundtruthy]';
Measure_line=ZM(1:2,2:361)-groundtruth(:,2:361);
Measure_error=sqrt(Measure_line(1,:).^2+Measure_line(2,:).^2);

iBeacon_IMU_PF_location_line=iBeacon_IMU_PF_location(:,2:361)-groundtruth(:,2:361);
iBeacon_IMU_PF_location_error=sqrt(iBeacon_IMU_PF_location_line(1,:).^2+iBeacon_IMU_PF_location_line(2,:).^2);
GPS_IMU_PF_location_line=GPS_IMU_PF_location(:,2:361)-groundtruth(:,2:361);
GPS_IMU_PF_location_error=sqrt(GPS_IMU_PF_location_line(1,:).^2+GPS_IMU_PF_location_line(2,:).^2);
IMM_PF_location_line=IMM_PF_location(:,2:361)-groundtruth(:,2:361);
IMM_PF_location_error=sqrt(IMM_PF_location_line(1,:).^2+IMM_PF_location_line(2,:).^2);

x_IMM_PF_location = zeros(1,11);
c_IMM_PF_location = zeros(1,11);
[b_IMM_PF_location, x_IMM_PF_location(1,2:11)]=hist(IMM_PF_location_error,10);
num=numel(IMM_PF_location_error);
%figure;plot(x_Adopted_location(1,2:11),b_Adopted_location/num);   %概率密度
c_IMM_PF_location(1,2:11)=cumsum(b_IMM_PF_location/num);        %累积分布

x_iBeacon_IMU_PF_location = zeros(1,11);
c_iBeacon_IMU_PF_location = zeros(1,11);
[b_iBeacon_IMU_PF_location, x_iBeacon_IMU_PF_location(1,2:11)]=hist(iBeacon_IMU_PF_location_error,10);
num=numel(iBeacon_IMU_PF_location_error);
%figure;plot(x_Adopted_location(1,2:11),b_Adopted_location/num);   %概率密度
c_iBeacon_IMU_PF_location(1,2:11)=cumsum(b_iBeacon_IMU_PF_location/num);        %累积分布

x_GPS_IMU_PF_location = zeros(1,11);
c_GPS_IMU_PF_location = zeros(1,11);
[b_GPS_IMU_PF_location, x_GPS_IMU_PF_location(1,2:11)]=hist(GPS_IMU_PF_location_error,10);
num=numel(GPS_IMU_PF_location_error);
%figure;plot(x_Adopted_location(1,2:11),b_Adopted_location/num);   %概率密度
c_GPS_IMU_PF_location(1,2:11)=cumsum(b_GPS_IMU_PF_location/num);        %累积分布
% average_error = sum(Adopted_location_error)/num-1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%KF%%%%%%%%
iBeacon_IMU_KF_location_line=iBeacon_IMU_KF_location(:,2:361)-groundtruth(:,2:361);
iBeacon_IMU_KF_location_error=sqrt(iBeacon_IMU_KF_location_line(1,:).^2+iBeacon_IMU_KF_location_line(2,:).^2);
GPS_IMU_KF_location_line=GPS_IMU_KF_location(:,2:361)-groundtruth(:,2:361);
GPS_IMU_KF_location_error=sqrt(GPS_IMU_KF_location_line(1,:).^2+GPS_IMU_KF_location_line(2,:).^2);
IMM_KF_location_line=IMM_KF_location(:,2:361)-groundtruth(:,2:361);
IMM_KF_location_error=sqrt(IMM_KF_location_line(1,:).^2+IMM_KF_location_line(2,:).^2);

x_IMM_KF_location = zeros(1,11);
c_IMM_KF_location = zeros(1,11);
[b_IMM_KF_location, x_IMM_KF_location(1,2:11)]=hist(IMM_KF_location_error,10);
num=numel(IMM_KF_location_error);
%figure;plot(x_Adopted_location(1,2:11),b_Adopted_location/num);   %概率密度
c_IMM_KF_location(1,2:11)=cumsum(b_IMM_KF_location/num);        %累积分布

x_iBeacon_IMU_KF_location = zeros(1,11);
c_iBeacon_IMU_KF_location = zeros(1,11);
[b_iBeacon_IMU_KF_location, x_iBeacon_IMU_KF_location(1,2:11)]=hist(iBeacon_IMU_KF_location_error,10);
num=numel(iBeacon_IMU_KF_location_error);
%figure;plot(x_Adopted_location(1,2:11),b_Adopted_location/num);   %概率密度
c_iBeacon_IMU_KF_location(1,2:11)=cumsum(b_iBeacon_IMU_KF_location/num);        %累积分布

x_GPS_IMU_KF_location = zeros(1,11);
c_GPS_IMU_KF_location = zeros(1,11);
[b_GPS_IMU_KF_location, x_GPS_IMU_KF_location(1,2:11)]=hist(GPS_IMU_KF_location_error,10);
num=numel(GPS_IMU_KF_location_error);
%figure;plot(x_Adopted_location(1,2:11),b_Adopted_location/num);   %概率密度
c_GPS_IMU_KF_location(1,2:11)=cumsum(b_GPS_IMU_KF_location/num);        %累积分布
% average_error = sum(Adopted_location_error)/num-1;

cordinatex=round(groundtruthx(1));
cordinatey=round(groundtruthx(2));
figure;
plot(IMM_PF_location_error,'b');hold on;
plot(Measure_error,'r:');hold off;
legend('AMP-IMM-PF','观测值',1);
xlabel('定位误差/m', 'FontSize', 10); ylabel('累积概率分布', 'FontSize', 10);
figure;
plot(x_IMM_PF_location,c_IMM_PF_location,'r');hold on;
plot(x_iBeacon_IMU_PF_location,c_iBeacon_IMU_PF_location,'b');hold on;
plot(x_GPS_IMU_PF_location,c_GPS_IMU_PF_location,'g');hold on;
plot(x_IMM_KF_location,c_IMM_KF_location,'r--');hold on;
plot(x_iBeacon_IMU_KF_location,c_iBeacon_IMU_KF_location,'b--');hold on;
plot(x_GPS_IMU_KF_location,c_GPS_IMU_KF_location,'g--');hold off;
axis([0 20 0 1]),grid on;
legend('IMM-PF','PF-1定位','PF-10定位','IMM-KF','KF-1定位','KF-10定位',1);
xlabel('定位误差/m', 'FontSize', 10); ylabel('累积概率分布', 'FontSize', 10);


