function [ekf_zh ekf_zv ekf_za]=EKF_Attitude(Z)
%% 数据融合滤波器
%状态矩阵
dt=0.02  ;
persistent X ;%静态变量
X=[0 0 0 0]';
%状态转移矩阵
persistent H;%静态变量
H=[1 0 0 0;
   0 0 1 1];
%
persistent P;
P=[1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0  10];
persistent Q;
Q=[ 0.01    0       0        0;
    0         0.01   0        0 ;
    0         0       0.01   0;
    0         0       0        0.01];
persistent R;
R=[1 0;
   0 0.1];
persistent I;
I=eye(4);

 F=[1 dt 0 0;
    0 1 dt 0;
    0 0 1  0;
    0 0 0  1];
    %状态值
       X=F*X;
       P=F*P*F'+Q;
       %观测值
       z=[Z(1);
          Z(2)];
       y=z-H*X;
       S=H*P*H' +R;
       K=P*H'*pinv(S);
       X=X+K*y;
       P=(I-K*H)*P;
ekf_zh=X(1);
ekf_zv=X(2);
ekf_za=X(3);
end