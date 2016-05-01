% ����ʽ��ģ��
function PCenter=IMM_PF()
% clc;
% clear;
% close all;
load model ZM
outdoor_sensor_data=260;
indoor_sensor_data=101;
sensor_data=outdoor_sensor_data+indoor_sensor_data;
ZALL=zeros(4,sensor_data);

for n=1:sensor_data
        gx(n)=ZM(1,n);
        gy(n)=ZM(2,n);
        dx(n)=ZM(3,n);
        dy(n)=ZM(4,n);
        ZALL(:,n)=[gx(n),dx(n),gy(n),dy(n)];
        Z(:,n)=[gx(n),gy(n)];
end

% fgps=fopen('sensor_data_041518.txt','r');%%%���ı�
% for n=1:sensor_data
%     gpsline=fgetl(fgps);%%%��ȡ�ı�ָ���Ӧ����
%     if ~ischar(gpsline) break;%%%�ж��Ƿ����
%     end;
%    %%%%��ȡ��������
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
% 
%         gx(n)=result.X;%GPS��������任��Ķ������꣬���������
%         gy(n)=result.Y;%GPS��������任��ı������꣬���������
%         Phi(n)=(data(3,1)+90)*pi/180;%�����
%         dd(n)=data(4,1);%ĳһ���ڵ�λ��
%         dx(n)=dd(n)*sin(Phi(n))*4;%ĳһ���ڵĶ���λ��
%         dy(n)=dd(n)*cos(Phi(n))*4;%ĳһ���ڵı���λ��
%         Ve(n)=dd(n)*sin(Phi(n));%��̼�����Ķ����ٶȣ���ʱ��ĳһ���ڵĶ���λ�ƴ���
%         Vn(n)=dd(n)*cos(Phi(n));%��̼�����ı����ٶȣ���ʱ��ĳһ���ڵı���λ�ƴ���
%         ZALL(:,n)=[gx(n),dx(n),gy(n),dy(n)];
%         Z(:,n)=[gx(n),gy(n)];
% end
% fclose(fgps);%%%%%�ر��ļ�ָ��

T = 1;
N = 100;               %������Ŀ
pai=[0.90,0.1;0.1,0.90]; %����һ��ת�Ƹ��ʾ���
u_cv=0.5;              %�����˶�ģ���ڳ�ʼʱ����ȷ�ĸ���
u_ca=0.5;              %�ȼ����˶�ģ���ڳ�ʼʱ����ȷ����
R1 = diag([10,10]);
R2 = diag([100,100]);
P1 =diag([5,0,5,0]);
Q1 =diag([5,0,5,0]); 
A1 = [1,T,0,0;
      0,1,0,0;
      0,0,1,T;
      0,0,0,1];
  
P2 =diag([5,0,5,0]);  
Q2 =diag([5,0,5,0]);
A2 = [1,T,0,0;
      0,1,0,0;
      0,0,1,T;
      0,0,0,1];
 
x1(:,1) = ZALL(:,1);
x2(:,1) = ZALL(:,1);
xe(:,1) = ZALL(:,1);

%%PARTICLE FILTER 
%%%-----------------------------

for t = 1:sensor_data-1
    
    %1 ���뽻��
    c_1 = pai(1,1)*u_cv+pai(2,1)*u_ca;
    c_2 = pai(1,2)*u_cv+pai(2,2)*u_ca;
    u11 = pai(1,1)*u_cv/c_1;
    u12 = pai(1,2)*u_cv/c_2;
    u21 = pai(2,1)*u_ca/c_1;
    u22 = pai(2,2)*u_ca/c_2;
    
    x11(:,t) = x1(:,t)*u11+x2(:,t)*u21;
    P11 = u11*(P1+(x1(:,t)-x11(:,t))*(x1(:,t)-x11(:,t))')+u21*(P2+(x2(:,t)-x11(:,t))*(x2(:,t)-x11(:,t))');
    
    x22(:,t) = x1(:,t)*u12+x2(:,t)*u22;
    P22 = u12*(P1+(x1(:,t)-x22(:,t))*(x1(:,t)-x22(:,t))')+u22*(P2+(x2(:,t)-x22(:,t))*(x2(:,t)-x22(:,t))');
    
    %2 ��������
    for i = 1:N
        x1part(:,i) = x11(:,t)+P1*randn(4,1);
        x2part(:,i) = x22(:,t)+P2*randn(4,1);
    end
    
    %3 PF�˲�
    
    %ģ��1���˲�
    for i = 1:N
        x1partemp(:,i) = A1*x1part(:,i)+Q1*randn(4,1);
        y1part(:,i) = [x1partemp(1,i);x1partemp(3,i)];
        error1(:,i) = y1part(:,i) - Z(:,t+1);
    end
    
    for i = 1:N
        er1(:,i) = [error1(1,i);error1(2,i)];
        w1(i) = exp(-er1(:,i)'*er1(:,i)/2);
    end
    
    % Ȩֵ��һ��
    w1sum = sum(w1);
    for i = 1 : N
        w1(i) = w1(i) / w1sum;
    end
               
    c(1) = w1(1);
    for i = 2 : N
        c(i) = c(i-1)+w1(i);
    end
    i = 1;
    u(1) = rand/N;
    for j = 1:N
        u(j) = u(1)+(j-1)/N;
        while u(j)>c(i)
            i = i+1;
        end
        x1part(:,j) = x1partemp(:,i);
        w1(j) = 1/N;
    end
    
   x1(:,t+1) = [mean(x1part(1,:));mean(x1part(2,:));mean(x1part(3,:));mean(x1part(4,:))];
   y1(:,t+1) = [mean(y1part(1,:));mean(y1part(2,:))];
   P1 = Q1;
   for i = 1:N
       e1(:,i) = x1part(:,i) - x1(:,t+1); 
       P1 = P1 + e1(:,i)*e1(:,i)'*w1(i);
   end
   
    S1 =R1;
    for i = 1:N
       S1 = R1+w1(i)*er1(:,i)*er1(:,i)';
    end
   
    
    %ģ��2���˲�
    for i = 1:N
        x2partemp(:,i) = A2*x2part(:,i)+Q2*randn(4,1);
        y2part(:,i) = [x2partemp(1,i);x2partemp(3,i)];
        error2(:,i) = y2part(:,i) - Z(:,t);
    end
    
    for i = 1:N
        er2(:,i) = [error2(1,i);error2(2,i)];
        w2(i) = exp(-er2(:,i)'*er2(:,i)/2);
    end
    
    % Ȩֵ��һ��
    w2sum = sum(w2);
    for i = 1 : N
        w2(i) = w2(i) / w2sum;
    end
     
    c(1) = w2(1);
    for i = 2 : N
        c(i) = c(i-1)+w2(i);
    end
    i = 1;
    u(1) = rand/N;
    for j = 1:N
        u(j) = u(1)+(j-1)/N;
        while u(j)>c(i)
            i = i+1;
        end
        x2part(:,j) = x2partemp(:,i);
        w2(j) = 1/N;
    end
    
   x2(:,t+1) = [mean(x2part(1,:));mean(x2part(2,:));mean(x2part(3,:));mean(x2part(4,:))];
   y2(:,t+1) = [mean(y2part(1,:));mean(y2part(2,:))];
    
   P2 = Q2;
   for i = 1:N
       e2(:,i) = x2part(:,i) - x2(:,t+1);
       P2 = P2 + e2(:,i)*e2(:,i)'*w2(i);
   end
   
   S2 =R2;
    for i = 1:N
       S2 = R2+w2(i)*er2(:,i)*er2(:,i)';
    end                                  
      
   
    %4����ģ�͸���
        
    Er1 = Z(:,t+1) -y1(:,t+1);
    Er2 = Z(:,t+1) -y2(:,t+1);
    yyy=Er1'*inv(S1)*Er1;
    xxx=exp(-yyy/2);
    like1=xxx/(sqrt(2*pi*det(S1)));
    like2=exp(-Er2'*inv(S2)*Er2/2)/(sqrt(2*pi*det(S2)));
    c_1=pai(1,1)*u_cv+pai(2,1)*u_ca;
    c_2=pai(1,2)*u_cv+pai(2,2)*u_ca;
    c=like1*c_1+like2*c_2;
    u_cv=like1*c_1/c; 
    u_ca=like2*c_2/c; 
    
    %5 �������
     xe(:,t+1)=x1(:,t+1)*u_cv+x2(:,t+1)*u_ca;
     p_model(1,t)=u_cv;
     p_model(2,t)=u_ca;
end

PCenter=[xe(1,:);xe(3,:)];
cordinatex=ZALL(1,1);
cordinatey=ZALL(3,1);
%��ʾ�˲��켣
figure
set(gca,'FontSize',12);
[groundtruthx,groundtruthy]=Ground_Truth();
plot(groundtruthx,groundtruthy,'r');hold on;
% plot(ZALL(1,:),ZALL(2,:),'o');hold on;
plot(PCenter(1,:),PCenter(2,:),'g');hold off;
axis([cordinatex-100 cordinatex+200 cordinatey-200 cordinatey+100]),grid on;
xlabel('x', 'FontSize', 20); ylabel('y', 'FontSize', 20);
title('IMM-PF');
legend('��ʵ�켣','Ŀ���˲�����');
axis equal;

figure
set(gca,'FontSize',12);
plot(p_model(1,:),'g');hold on;
plot(p_model(2,:),'r');hold on;
xlabel('time/s', 'FontSize', 20); ylabel('model probability', 'FontSize', 20);
title('IMM-PF ģ�͸���');
legend('PF1','PF10');

