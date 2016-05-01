% 交互式多模型
function PCenter=IMM_KF()
% clc;
% clear;
% close all;
load model ZM
ww=100;%过程噪声方差
% R=10;
vv1=10*log10(10);%观测噪声方差
vv2=10*log10(100);%观测噪声方差

T=1;%采样时间
outdoor_sensor_data=260;
indoor_sensor_data=101;
sensor_data=outdoor_sensor_data+indoor_sensor_data;
ZALL=zeros(4,sensor_data);

for n=1:sensor_data
        gx(n)=ZM(1,n);
        gy(n)=ZM(2,n);
        dx(n)=ZM(3,n);
        dy(n)=ZM(4,n);
        Ve(n)=ZM(3,n);%里程计输入的东向速度，暂时用某一周期的东向位移代替
        Vn(n)=ZM(4,n);%里程计输出的北向速度，暂时用某一周期的北向位移代替
        ZALL(:,n)=[gx(n),gy(n),dx(n),dy(n)];
end

% fgps=fopen('sensor_data_041518.txt','r');%%%打开文本
% for n=1:sensor_data
%     gpsline=fgetl(fgps);%%%读取文本指针对应的行
%     if ~ischar(gpsline) break;%%%判断是否结束
%     end;
%    %%%%读取室内数据
%    time=sscanf(gpsline,'[Info] 2016-04-15%s(ViewController.m:%d)-[ViewController outputAccelertion:]:lat:%f;lon:%f;heading:%f;distance:%f;beacon_lat:%f;beacon_lon:%f');
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
%         Phi(n)=(data(3,1)+90)*pi/180;%航向角
%         dd(n)=data(4,1);%某一周期的位移
%         dx(n)=dd(n)*sin(Phi(n))*4;%某一周期的东向位移
%         dy(n)=dd(n)*cos(Phi(n))*4;%某一周期的北向位移
%         Ve(n)=dd(n)*sin(Phi(n));%里程计输入的东向速度，暂时用某一周期的东向位移代替
%         Vn(n)=dd(n)*cos(Phi(n));%里程计输出的北向速度，暂时用某一周期的北向位移代替
%         ZALL(:,n)=[gx(n),gy(n),dx(n),dy(n)];
% end
% fclose(fgps);%%%%%关闭文件指针

r=2;%模型的个数   
%模型1 
F1=[
    1,0,T,0,0;
    0,1,0,T,0;
    0,0,1,0,0,;
    0,0,0,1,0;
    0,0,0,0,1;
   ];
Q1=diag([0,0,ww,ww,ww]);
R1=diag([vv1,vv1,vv1,vv1]);
P1=diag([0,0,ww,ww,ww]); % 滤波输出误差均方差矩阵
x1=[ZALL(1,1),ZALL(2,1),0,0,0]'; %初始条件进行估计
%模型2
F2=[
    1,0,T,0,0;
    0,1,0,T,0;
    0,0,1,0,0,;
    0,0,0,1,0;
    0,0,0,0,1;
   ];
Q2=diag([0,0,ww,ww,ww]);
R2=diag([vv2,vv2,vv2,vv2]);
P2=diag([0,0,ww,ww,ww]); % 滤波输出误差均方差矩阵
x2=[ZALL(1,1),ZALL(2,1),0,0,0]'; %初始条件进行估计

F=[F1 F2];
Q=[Q1 Q2];
R=[R1 R2];
P=[P1 P2];
x=[x1 x2];
xx1=zeros(5,1);%模型1估计的中间值
xx2=zeros(5,1);%模型2估计的中间值
xx=[xx1 xx2];
xf=zeros(5,361);

pm=[0.9 0.1
    0.1 0.9];%马尔科夫转移矩阵
u1=0.5;
u2=0.5;
u=[u1 u2];%混合概率
um=[0 0
    0 0];%混合概率矩阵
zz=zeros(4,2);%两模型的测量值分布与对应的滤波值的距离
%交互式多模型算法
for k=1:361
    %每次计算之前有些变量清零
    H=[1,0,0,0,0;
        0,1,0,0,0;
        0,0,1,0,-Vn(k);
        0,0,0,1,Ve(k);
        ];
    c=[0 0];%混合概率中的系数
    xx=[xx1 xx2];
    pp=zeros(5,10);
    cc=0;%模型概率更新中的c
    %混合概率
    for j=1:r
        for i=1:r
            c(j)=c(j)+pm(i,j)*u(i);
        end
        for i=1:r
            um(i,j)=1/c(j)*pm(i,j)*u(i);
        end
    end
    %混合估计
    for j=1:r
        for i=1:r
            xx(:,j)=xx(:,j)+x(:,i)*um(i,j);
        end
    end
    for j=1:r
        for i=1:r
            pp(:,(j-1)*5+1:(j-1)*5+5)=pp(:,(j-1)*5+1:(j-1)*5+5)+(P(:,(i-1)*5+1:(i-1)*5+5)+(x(:,i)-xx(:,j))*(x(:,i)-xx(:,j))')*um(i,j);
        end
    end
    %模型条件滤波
    %状态预测
    for i=1:r
        x(:,i)=F(:,(i-1)*5+1:(i-1)*5+5)*xx(:,i);
        P(:,(i-1)*5+1:(i-1)*5+5)=F(:,(i-1)*5+1:(i-1)*5+5)*pp(:,(i-1)*5+1:(i-1)*5+5)*F(:,(i-1)*5+1:(i-1)*5+5)'+Q(:,(i-1)*5+1:(i-1)*5+5);
    end
    %量测预测残差及其协方差阵计算
    for i=1:r
        zz(:,i)=ZALL(:,k)-H*x(:,i);
        S(:,(i-1)*4+1:(i-1)*4+4)=H*P(:,(i-1)*5+1:(i-1)*5+5)*H'+R(:,(i-1)*4+1:(i-1)*4+4);
        L(i)=abs(2*3.1415926*det(S(:,(i-1)*4+1:(i-1)*4+4)))^(-0.5)*exp(-0.5*zz(:,i)'/S(:,(i-1)*4+1:(i-1)*4+4)*zz(:,i));%似然函数
    end
    %滤波更新
    for i=1:r
        K(:,(i-1)*4+1:(i-1)*4+4)=P(:,(i-1)*5+1:(i-1)*5+5)*H'/S(:,(i-1)*4+1:(i-1)*4+4);
        x(:,i)=x(:,i)+K(:,(i-1)*4+1:(i-1)*4+4)*zz(:,i);
        P(:,(i-1)*5+1:(i-1)*5+5)=P(:,(i-1)*5+1:(i-1)*5+5)-K(:,(i-1)*4+1:(i-1)*4+4)*S(:,(i-1)*4+1:(i-1)*4+4)*K(:,(i-1)*4+1:(i-1)*4+4)';
    end
    %模型概率更新
    for j=1:r
        cc=cc+L(j)*c(j);
    end
    for i=1:r
        u(i)=1/cc*L(i)*c(i);
    end
    %估计融合
    for i=1:r
        xf(:,k)=xf(:,k)+x(:,i)*u(:,i);
        p_model(i,k)=u(i);
    end
end    

PCenter=xf(1:2,:);
cordinatex=ZALL(1,5);
cordinatey=ZALL(2,5);
%显示滤波轨迹
figure
set(gca,'FontSize',12);
[groundtruthx,groundtruthy]=Ground_Truth();
plot(groundtruthx,groundtruthy,'r');hold on;
% plot(ZALL(1,:),ZALL(2,:),'o');hold on;
plot(PCenter(1,:),PCenter(2,:),'g');hold off;
axis([cordinatex-100 cordinatex+200 cordinatey-200 cordinatey+100]),grid on;
xlabel('x', 'FontSize', 20); ylabel('y', 'FontSize', 20);
title('IMM-KF');
legend('真实轨迹','目标滤波航迹');
axis equal;

figure
set(gca,'FontSize',12);
plot(p_model(1,:),'g');hold on;
plot(p_model(2,:),'r');hold on;
xlabel('time/s', 'FontSize', 20); ylabel('model probability', 'FontSize', 20);
title('IMM-KF 模型概率');
legend('PF1','PF10');

