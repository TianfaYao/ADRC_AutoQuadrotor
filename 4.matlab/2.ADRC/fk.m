


clc,clear
Fs = 100; %采样频率
T = 1/Fs;
h = 0.008; % h,r为参数
r=1200;
x = -2*pi:0.1:2*pi; % 生成一个算例：sin函数
u = sin(2*x);%%.*wgn(1,length(x),10);
u1=0; % 赋初值
u2=0;
for i=1:length(u) % 主体程序
x1 = u1;
x2 = u2;
delta = r*h;
delta0 = delta*h;
y = x1-u+h*x2;
abs_y = abs(y);
a0 = sqrt(delta*delta+8*r*abs_y);
if (abs_y <= delta0)
a = x2+y/h; 
else 
if (y >= 0) 
a = x2 + 0.5*(a0-delta); 
else 
a = x2 - 0.5*(a0-delta); 
end
end
if (abs(a) <= delta)
fst = -r*a/delta; 
else
if (a >= 0)
fst = -r; 
else 
fst = r;
end
end
u1 = x1+T*x2; % 计算跟踪信号
u2 = x2+T*fst; % 计算微分信号

u1_result(i)=u1; % 保存数值
u2_result(i)=u2; 
end

figure
plot(x,u,x,u1_result,x,u2_result)
legend('原始信号','跟踪信号','微分信号')
%


