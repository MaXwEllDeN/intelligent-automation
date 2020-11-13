clear
close all
clc
%
h=1e-3;
x=(-3*pi:h:3*pi)';
%
a=2.0;
b=0.1;
c=1.0;
%
%f=@(x) a*log(x)+b*log(1-x)+c;f1p=@(x) a./x + b./(x - 1);f2p=@(x) -a./x.^2 - b./(x - 1).^2;
f=@(x) a*exp(-b*x).*sin(c*x);
f1p=@(x) a*exp(-b*x).*(c*cos(c*x)-b*sin(c*x));
f2p=@(x) a*(b^2).*exp(-b*x).*sin(c*x)-a*(c^2).*exp(-b*x).*sin(c*x)-2*a*b*c.*exp(-b*x).*cos(c*x);
%
y=f(x);
%
figure(1)
plot(x,y),hold
xlabel('x')
grid
%
x0=-1.0;
tol=1e-6;
N=50;
sol=false;
gamma=0.5;
vals=[];
%
for i=1:N
    dxN=-gamma*f(x0)/f1p(x0);% Newton's (Newton-Raphson) method
    dxH=-2*f(x0)*f1p(x0)/(2*f1p(x0)^2-f1p(x0)*f2p(x0));% Halley's method
    dx=dxH;
    x1=x0+dx;
    vals=[vals;x0 dx x1];
    if(abs(x1-x0)<=tol*abs(x1))
        sol=true;
        break;
    end
    x0=x1;
end
%
if(sol)
   title(['x_{*}=' num2str(x1) ' e f(x_{*})=' num2str(f(x1))])
   plot(x1,f(x1),'ro'), legend('f(x)','x_{*}')
else
   title('No solution found! ')
end