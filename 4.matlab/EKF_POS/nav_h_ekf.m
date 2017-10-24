function [ h v a a_a] = nav_h_ekf(Q, H, R ,F, I ,Z )
%作者 ytf
%日期 2017年4月27日
%状态转移协方差矩阵
%H 预测矩阵
%R 观测噪声方差
%F 状态转移矩阵
%Z 测量矩阵
persistent X ;% 状态变量
          if(isempty(X))
           X=single([0 0 0 0]');
          end
 persistent P ;% 状态协方差矩阵
            if(isempty(P))
            P=single([10 0 0 0;
                     0 10 0 0;
                     0 0 10 0;
                     0 0 0  100]);
            end
       P=F*P*F'+Q;
       K=P*H'/(H*P*H' +R); %增益
       X=X+K*(Z-H*X);
       P=(single(I)-K*H)*P;
       h=X(1);
       v=X(2);
       a=X(3);
       a_a=X(4);
       