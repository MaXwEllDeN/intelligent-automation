clear all
syms Kd
syms Kp
syms c
syms s
syms v0
syms phi0
%%
%PD
A = [0 0 -v0*sin(phi0) 0;
    0 0 v0*cos(phi0) 0;
    0 0 0 -Kd*c^2;
    0 0 0 -c];
B = [0;0;Kp+Kd*c;1];
C = [0 0 0 -Kd*c^2];
D = Kp+Kd*c;
K = (s*eye(4)-A)
GP = simplify(C*(K\B)+D)
MF = simplify(GP/(1+GP))
[~,p] = numden(simplify(MF))
solve(p == 0,s)
%%
fun = matlabFunction(subs(GP,[Kp,Kd,c],[1,5,1]));
fun = str2func(regexprep(func2str(fun),...
                        '\.([/^\\*])','$1'));
gp = fun(tf('s'))
r = rlocus(gp,1);
n = nyquist(gp);
margin(gp)
%%
%P
syms x
syms y
syms y_g
syms x_g
syms phi

e=atan((y_g-y)/(x_g-x))-phi
A = [0 0 -v0*sin(phi0);
    0 0 v0*cos(phi0);
    0 0 0]
B = [0;0;1];
C = [0 0 0];
D = Kp;
K = (s*eye(3)-A)
GP = simplify(C*(K\B)+D)
MF = simplify(GP/(1+GP))
[~,p] = numden(simplify(MF))
solve(p == 0,s)
%%
gp = tf([.5],[1+.5])
r = rlocus(gp,1)
nyquist(gp)
margin(gp)