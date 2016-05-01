T=1;%采样时间
outdoor_sensor_data=260;
indoor_sensor_data=101;
sensor_data=outdoor_sensor_data+indoor_sensor_data;
ZALL=zeros(4,sensor_data);
[ground_truthx,ground_truthy,heading,velocity]=Ground_Truth(); 

for n=1:sensor_data
        if n<102
            gx(n)=ground_truthx(n)+ wgn(1, 1,  10*log10(10));%室内观测值，噪声方差为1
            gy(n)=ground_truthy(n)+ wgn(1, 1,  10*log10(10));%室内观测值，噪声方差为1
        else
            gx(n)=ground_truthx(n)+ wgn(1, 1,  10*log10(100));%室外观测值，噪声方差为10
            gy(n)=ground_truthy(n)+ wgn(1, 1,  10*log10(100));%室外观测值，噪声方差为10
        end
        phi(n)=(heading(n))*pi/180;%航向角
        dd(n)=velocity(n);%速度
        dx(n)=dd(n)*sin(phi(n));%某一周期的东向位移
        dy(n)=dd(n)*cos(phi(n));%某一周期的北向位移
        Ve(n)=dd(n)*sin(phi(n));%里程计输入的东向速度，暂时用某一周期的东向位移代替
        Vn(n)=dd(n)*cos(phi(n));%里程计输出的北向速度，暂时用某一周期的北向位移代替
        ZM(:,n)=[gx(n),gy(n),dx(n),dy(n)];
end

save model ZM