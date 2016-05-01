%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%程序初始化操作%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function PCenter=PF(R)
% clc;
% clear;
% close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%全局变量定义%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load model ZM;
outdoor_sensor_data=260;
indoor_sensor_data=101;
sensor_data=outdoor_sensor_data+indoor_sensor_data;
ZALL=zeros(4,sensor_data);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%读取传感器数据%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:sensor_data
        gx(n)=ZM(1,n);
        gy(n)=ZM(2,n);
        dx(n)=ZM(3,n);
        dy(n)=ZM(4,n);
        ZALL(:,n)=[gx(n),gy(n),dx(n),dy(n)];
end

% fgps=fopen('sensor_data_041518.txt','r');%%%打开文本
% for n=1:sensor_data
%     gpsline=fgetl(fgps);%%%读取文本指针对应的行
%     if ~ischar(gpsline) break;%%%判断是否结束
%     end;
%     %%%%读取室内数据
%    data=sscanf(gpsline,'[Info] 2016-04-15 %*s (ViewController.m:%*d)-[ViewController outputAccelertion:]:lat:%f;lon:%f;heading:%f;distance:%f;beacon_lat:%f;beacon_lon:%f');
%    if(isempty(data))
%        break;
%    end
%     if n>127 && n<355
%        result=lonLat2Mercator(data(6,1),data(5,1));
%     else
%        result=lonLat2Mercator(data(2,1),data(1,1));
%     end
%         gx(n)=result.X;%GPS经过坐标变换后的东向坐标，换算成米数
%         gy(n)=result.Y;%GPS经过坐标变换后的北向坐标，换算成米数
%         Phi(n)=(data(3,1)+90)*pi/180;%航向角>>>>>>> origin/master
%         dd(n)=data(4,1);%某一周期的位移
%         ZALL(:,n)=[gx(n),gy(n),Phi(n),dd(n)];
% end
% fclose(fgps);%%%%%关闭文件指针

% 参数设置
N = 100;   %粒子总数
Q = 10;      %过程噪声
% R = 10;      %测量噪声
X = zeros(2, sensor_data);    %存储系统状态
Z = zeros(2, sensor_data);    %存储系统的观测状态
P = zeros(2, N);    %建立粒子群
PCenter = zeros(2, sensor_data);  %所有粒子的中心位置
w = zeros(N, 1);         %每个粒子的权重
err = zeros(1,sensor_data);     %误差
X(:, 1) = [ZALL(1,1); ZALL(2,1)];     %初始系统状态
Z(:, 1) = [ZALL(1,1); ZALL(2,1)] ;    %初始系统的观测状态
cordinatex=round(ZALL(1,1));
cordinatey=round(ZALL(2,1));
%初始化粒子群
for i = 1 : N
    P(:, i) = [randi([cordinatex,cordinatex],1);randi([cordinatey,cordinatey],1)];
    dist = norm(P(:, i)-Z(:, 1));     %与测量位置相差的距离
    w(i) = (1 / sqrt(R) / sqrt(2 * pi)) * exp(-(dist)^2 / 2 / R);   %求权重
end
PCenter(:, 1) = sum(P, 2) / N;      %所有粒子的几何中心位置
err(1) = norm(X(:, 1) - PCenter(:, 1));     %粒子几何中心与系统真实状态的误差

%开始运动
for k = 2 : sensor_data
       
    %模拟一个弧线运动的状态
    X(:, k) = X(:, k-1) + [ZALL(3,k); ZALL(4,k)];     %状态方程
    Z(:, k) = ZALL(1:2, k) ;     %观测方程 
   
    %粒子滤波
    %预测
    for i = 1 : N
        P(:, i) = P(:, i) +  [ZALL(3,k); ZALL(4,k)]+ wgn(2, 1, 10*log10(Q));
        dist = norm(P(:, i)-Z(:, k));     %与测量位置相差的距离
        w(i) = (1 / sqrt(R) / sqrt(2 * pi)) * exp(-(dist)^2 / 2 / R);   %求权重
    end
%归一化权重
    wsum = sum(w);
    for i = 1 : N
        w(i) = w(i) / wsum;
    end
   
    %重采样（更新）
    for i = 1 : N
        wmax = 2 * max(w) * rand;  %另一种重采样规则
        index = randi(N, 1);
        while(wmax > w(index))
            wmax = wmax - w(index);
            index = index + 1;
            if index > N
                index = 1;
            end          
        end
        P(:, i) = P(:, index);     %得到新粒子
    end
   
    PCenter(:, k) = sum(P, 2) / N;      %所有粒子的中心位置
   
    %计算误差
    err(k) = norm(X(:, k) - PCenter(:, k));     %粒子几何中心与系统真实状态的误差
end

figure;
set(gca,'FontSize',12);
[groundtruthx,groundtruthy]=Ground_Truth();
plot(groundtruthx,groundtruthy,'r');hold on;
plot(PCenter(1,:), PCenter(2,:), 'g');hold off;
axis([cordinatex-100 cordinatex+200 cordinatey-200 cordinatey+100]),grid on;
legend('真实轨迹', '目标滤波航迹');
title('PF');
xlabel('x', 'FontSize', 20); ylabel('y', 'FontSize', 20);
axis equal;
