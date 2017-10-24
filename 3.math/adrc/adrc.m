clc ,clear;
close all;
t=0:0.1:8*pi;  
%y1=sin(t)+cos(t) ;%%
m=randn(1,length(t));
y1=sin(t).*m;

r=5;% 过度过程加速度
h=0.1;

%r=15;% 过度过程加速度
%h=0.01;
%h=0.025;% 滤波因子
  x1(1)=0;
  x2(1)=0;
  y2(1)=0;
  
for k=1:length(y1)-1
    v(k)=y1(k);
    fh=fhan(x1(k)-v(k),x2(k),r,h);
    x1(k+1)=x1(k)+h*x2(k);
    x2(k+1)=x2(k)+h*fh;
    y2(k)=y1(k+1)-y1(k);
end

l=1:length(x2);
h=1:length(y1);
%plot(sin(t));
hold on ;
plot(h,y1,'g');
hold on
plot(l,x1,'b');
legend('y1','x1')
title('TD');
grid on;

figure(2) 
g=1:length(y2);
plot(l,x2,'r');
hold on ;
plot(g,y2,'k');
legend('td 微分信号','原始微分信号');
hold on ;
title('微分信号');
grid on
