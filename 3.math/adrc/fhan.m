function fh=fhan(x1 ,x2 ,r ,h)
%% °ô°ô·¨  bang_bang
%x1(k+1)=x1(k)+h*x2;
%x2(k+1)=x2(k)-r*u
%%
% d=r*h^2;
% a0==h*x2;
% y=x1+a0;
% a1=sqrt(d*(d+8*abs(y)));
% a2=a0+sign(y)(a1-d)/2;
% Sy=(sign(y+d)-sign(y-d))/2;fsg
% a=(a0+y-a2)*Sy+a2;
% Sa=(sign(a+d)-sign(a-d))/2;
% fhan=-r(a/d-sign(a))*Sa-r*sign(a);
%
%               ÀëÉ¢×´Ì¬
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% fh=fhan(x1(k)-v(k),x2(k),r,h);
% x1(k+1)=x1(k)+h*x2(k);
% x2(k+1)=x2(k)+h*fh;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
d=r*h^2;
a0=h*x2;
y=x1+a0;
a1=sqrt(d*(d+8*abs(y)));
a2=a0+sign(y)*(a1-d)/2;

Sy=(sign(y+d)-sign(y-d))/2;

a=(a0+y-a2)*Sy+a2;
Sa=(sign(a+d)-sign(a-d))/2;
fh=-r*(a/d-sign(a))*Sa-r*sign(a);
end