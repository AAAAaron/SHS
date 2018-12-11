function k=LeastSquare(x,y)
n=length(x);
t1=0;
t2=0;
t3=0;
t4=0;

for i=1:n
    t1=t1+x(i)^2;
    t2=t2+x(i);
    t3=t3+x(i)*y(i);
    t4=t4+y(i);
end
k = (t3*n - t2*t4) / (t1*n - t2*t2);

    
