function   u=eso(c1 ,r,h , v1, v2, x)
    persistent  z1 z2 z3 uo ;
    if isempty(z1)
        z1=0;
        z2=0;
        z3=0; 
        uo=0;
    end
	
    B1=1/h;
	B2=1/(3*h^2);
    B3=1/(64*h^3);




    e=z1-x;
	fe=fal(e,0.5,h);
	fe1=fal(e,0.25,h);
	
	z1=z1+h*(z2-B1*e);
	z2=z2+h*(z3-B2*fe+uo);
	z3=z3+h*(-B3*fe1);
	
	e1=v1-z1;
	e2=v2-z2;
	u0=fhan(e1,c1*e2,r,h);
	u=(u0-z3);
    uo=u;
end
    
%% fal ÌØÐÔÇúÏß
e=-100:0.01:100;
l=length(e);
fa(1)=0;
    for i=1:l
     fa(i)=fal(e(i),0.5,0.1);
    end
    plot(fa);
    hold on
grid on 







