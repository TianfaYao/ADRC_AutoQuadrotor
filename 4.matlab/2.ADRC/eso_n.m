function   u=eso(c1 ,r,h ,b0, v1, v2, x)
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
% 	
	u0=fhan(e1,c1*e2,r,h);
	u=(u0-z3)/b0;
    uo=u;
    end