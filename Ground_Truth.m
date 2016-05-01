function [X,Y,Heading,Velocity] = Groud_Truth()

X=zeros(361,1);
Y=zeros(361,1);
Heading=zeros(361,1);
Velocity=zeros(361,1);

result0=lonLat2Mercator(121.415633,31.029636);
%第一段
result1=lonLat2Mercator(121.415274,31.029524);
for t=1:39
	X(t)=result0.X+(result1.X-result0.X)/39*t;
	Y(t)=result0.Y+(result1.Y-result0.Y)/39*t;
    Heading(t)=252;
    Velocity(t)=1;
end

%第二段
result2=lonLat2Mercator(121.415260,31.029545);
for t=1:3
    X(t+39)=X(39)+(result2.X-result1.X)/3*t;
	Y(t+39)=Y(39)+(result2.Y-result1.Y)/3*t;
    Heading(t)=252+90;
    Velocity(t)=1;
end

%第三段
result3=lonLat2Mercator(121.415180,31.029517);
for t=1:9
    X(t+42)=X(42)+(result3.X-result2.X)/9*t;
	Y(t+42)=Y(42)+(result3.Y-result2.Y)/9*t;
    Heading(t)=252;
    Velocity(t)=1;
end

%第四段
result4=lonLat2Mercator(121.415074,31.029778);
for t=1:30
    X(t+51)=X(51)+(result4.X-result3.X)/30*t;
	Y(t+51)=Y(51)+(result4.Y-result3.Y)/30*t;
    Heading(t)=252+90;
    Velocity(t)=1;
end

%第五段
result5=lonLat2Mercator(121.414834,31.029709);
for t=1:20
    X(t+81)=X(81)+(result5.X-result4.X)/20*t;
	Y(t+81)=Y(81)+(result5.Y-result4.Y)/20*t;
    Heading(t)=252;
    Velocity(t)=1;
end

%第六段
result6=lonLat2Mercator(121.414735,31.029686);
for t=1:11
    X(t+101)=X(101)+(result6.X-result5.X)/11*t;
	Y(t+101)=Y(101)+(result6.Y-result5.Y)/11*t;
    Heading(t)=252;
    Velocity(t)=1;
end

%%%室内的101米%%%%%%%%%%%%%
%第七段
result7=lonLat2Mercator(121.415083,31.028926);
for t=1:92
    X(t+112)=X(112)+(result7.X-result6.X)/92*t;
	Y(t+112)=Y(112)+(result7.Y-result6.Y)/92*t;
    Heading(t)=252-90;
    Velocity(t)=1;
end

%第八段
result8=lonLat2Mercator(121.415974,31.029208);
for t=1:91
    X(t+204)=X(204)+(result8.X-result7.X)/91*t;
	Y(t+204)=Y(204)+(result8.Y-result7.Y)/91*t;
    Heading(t)=252-90-90;
    Velocity(t)=1;
end

%第九段
result9=lonLat2Mercator(121.415765,31.029677);
for t=1:55
    X(t+295)=X(295)+(result9.X-result8.X)/55*t;
	Y(t+295)=Y(295)+(result9.Y-result8.Y)/55*t;
    Heading(t)=252-90;
    Velocity(t)=1;
end

%第十段
result10=lonLat2Mercator(121.415633,31.029636);
for t=1:11
    X(t+350)=X(350)+(result10.X-result9.X)/11*t;
	Y(t+350)=Y(350)+(result10.Y-result9.Y)/11*t;
    Heading(t)=252;
    Velocity(t)=1;
end

 %显示真实轨迹
% cordinatex=round(X(1));
% cordinatey=round(Y(1));
% plot(X,Y,'r'),grid on;
% axis([cordinatex-200 cordinatex+200 cordinatey-200 cordinatey+200]),grid on;
% 
% legend('目标真实航迹');
% axis equal;


