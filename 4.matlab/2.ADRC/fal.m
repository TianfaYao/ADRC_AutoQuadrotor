function f=fal(e,a,d)
if abs(e)<=d
    f=e/(d^(1-a));
else
   f=(abs(e))^a*sign(e);
end
