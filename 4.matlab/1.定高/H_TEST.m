%% 气压计和加速度计Z轴高度和速度融合

%% 数据加载
get = xlsread('get.csv');
x=1;
y=2;
z=3;
acc=get(:,1:3); %加速度
gry=get(:,4:6); %陀螺仪
mag=get(:,7:9); %罗盘
att=get(:,10:12);%姿态角
dt=0.02;
%% 函数初始化
Q=[ 0.01    0       0        0;
    0         0.01   0        0 ;
    0         0       0.01   0;
    0         0       0        0.01];
R=[1 0;
   0 0.1];
I=eye(4);

%  F=[1 dt 0.5*dt*dt 0;
%     0 1 dt 0.5*dt*dt;
%     0 0 1  dt;
%     0 0 0  1];
 F=[1 dt 0 0;
    0 1 dt 0;
    0 0 1  0;
    0 0 0  1];


 H=[1 0 0 0;
   0 0 1 1];

  %   Z=[mag(i,y)h 
  %     mag(i,x)]a
%%
h(1)=0; v(1)=0;a(1)=0;
for i=1:length(mag(:,z))
    Z=[mag(i,y)
       mag(i,x)];
[ h(i) v(i) a(i) ] = nav_h_ekf(Q, H, R ,F, I ,Z );
end

step =1:length(h)
plot(step ,h(step),'g');
hold on
plot(step,v(step),'r');
hold on 
plot(step ,a(step),'b');
legend('高度','速度','加速度');
grid on 
gtext('KF融合气压计和加速度计')