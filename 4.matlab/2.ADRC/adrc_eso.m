clc,clear
close all;
t=0:0.01:4;

y(1)=0
for i=1:length(t)
    if i>=200
    y(i)=1;
    else 
     y(i)=0;
    end
end
plot(y)
hold on ;

%%
r0=100;
h0=0.01
%%
c1=1;
r1=5; 
h1=0.1;
%%
uw(1)=0;
vw1=0;
vw2=0;
s1(1)=0;
s2(1)=0;

for i=1:length(y)
    fh=fhan(vw1-y(i),vw2,r0,h0);
    vw1=vw1+h0*vw2;
    vw2=vw2+h0*fh;
    s1(i)=vw1;
    uw(i)=eso(c1,r1,h1,vw1 ,vw2 ,y(i));
end

L=1:length(uw);
plot(L,s1,'r')
legend('原始信号','TD信号');

%figure(2) 
plot(L,uw,'k');
legend('ESO信号')
