function gotogoalPID
%
clear all
close all
%
%fmincon
oldopts=optimset(@fmincon);
optionsFMINCON=optimset(oldopts,'display','iter','TolCon',1e-9);
%ode45
optionsODE=odeset('RelTol',1e-6);
%
kp=0.1; ki=0.1; kd=0.01; pd=50;
xg=2; yg=2; v0=0.25;
fg=atan2(yg,xg);
%
tfin=20;
xini=[eps;eps;-fg;eps;eps];
[t0,x0]=ode45(@(t,x)smf(t,x,kp,ki,kd,pd),[0 tfin],xini,optionsODE);
%
fun=@fobj;
A=[-1,0,0;0,-1,0;0,0,-1];b=[0;0;0];
Aeq=[];beq=[];
lb=[0,0,0];ub=[1,1,1];
nonlcon=@mycon;
nvars=3;
%
PID=fmincon(@fobj,[kp ki kd],A,b,Aeq,beq,lb,ub,nonlcon,optionsFMINCON);

Kp=PID(1); Ki=PID(2); Kd=PID(3);
%
[t1,x1]=ode45(@(t,x)smf(t,x,Kp,Ki,Kd,pd),[0 tfin],xini,optionsODE);
%
figure(1)
plot(x0(:,1),x0(:,2),'b-','LineWidth',2.0)
xlabel('x, (m)'), ylabel('y, (m)')
hold on
for i=1:round(length(x0(:,1))/30):length(x0(:,1)),
    desenherobo(x0(i,1),x0(i,2),x0(i,3),0.03,'b-');
end
plot(x1(:,1),x1(:,2),'r-','LineWidth',2.0)
for i=1:round(length(x1(:,1))/30):length(x1(:,1)),
    desenherobo(x1(i,1),x1(i,2),x1(i,3),0.03,'r-');
end
grid
hold off
%
str = sprintf('Kp = %d, Ki = %d e Kd = %d', kp,ki,kd);
disp(str);
str = sprintf('Kp = %d, Ki = %d e Kd = %d', Kp,Ki,Kd);
disp(str);
%
function [vobj] = fobj(k)
%
P=k(1);
I=k(2);
D=k(3);
%
[t,x]=ode45(@(t,x)smf(t,x,P,I,D,pd),[0 tfin],xini,optionsODE);
%
xp=x(:,1);
yp=x(:,2);
fp=x(:,3);
%
%e=sqrt((xg-xp).^2+(yg-yp).^2);
%
fd=atan2(yg-yp,xg-xp);
e=fd-fp;
e=atan2(sin(e),cos(e));
%
%% Performance criteria
ISE=trapz(e.^2);
%IAE=trapz(t,abs(e));
%ISE=trapz(t,e.^2);
%ITAE=trapz(t,t.*abs(e));
%ITSE=trapz(t,t.*(e.^2));
%
vobj=ISE;
end
%
function [c,ceq] = mycon(x)
gp=x(1);gi=x(2);gd=x(3);
%
c(1) = -(gi+gp*pd)/(gp+gd*pd);
c(2) = -(gi*pd)/(gp+gd*pd);
c(3) = -(gi*(pd+1)+gp*pd)/(gp+gd*pd);
%
ceq = [];
end
%
function xdot = smf(t,x,kp,ki,kd,pd)
%
xp=x(1,1);
yp=x(2,1);
fp=x(3,1);
z1=x(4,1);
z2=x(5,1);
%% Goal
fd=atan2(yg-yp,xg-xp);
e=fd-fp;
e=atan2(sin(e),cos(e));
%
delta=norm([xg-xp yg-yp]);
if delta < 0.01,
    nu=0;
else
    nu=v0;
end
%
xdot=[nu*cos(fp);nu*sin(fp);ki*pd*z1+(ki-kd*pd^2)*z2+(kp+kd*pd)*e;z2;-pd*z2+e];
end
%
function []=desenherobo(x,y,q,s,c)
%
p=[ 1              1/7     
   -3/7            1       
   -5/7            6/7     
   -5/7            5/7     
   -3/7            2/7     
   -3/7            0       
   -3/7           -2/7     
   -5/7           -5/7     
   -5/7           -6/7     
   -3/7           -1       
    1             -1/7     
    1              1/7 ];
%
p=s*p;
p=[p,ones(length(p),1)];
r=[cos(q),sin(q);-sin(q),cos(q);x,y];
p=p*r;
%
X=p(:,1); 
Y=p(:,2); 
plot(X,Y,c,'LineWidth',1.0)
end
%
end