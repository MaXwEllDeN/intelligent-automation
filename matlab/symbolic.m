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
% C = [0 0 0 -Kd*c^2];
% D = Kp+Kd*c;
C = [0 0 1 0];
D = 0;
K = (s*eye(4)-A);
MA = simplify(C*(K\B)+D)
[~,p] = numden(simplify(MA));
MF = simplify(MA/(1+MA))
[~,p] = numden(simplify(MF));
solve(p == 0,s)
%%
% fun = matlabFunction(subs(GP,[Kp,Kd,c],[1,5,1]));
% fun = str2func(regexprep(func2str(fun),...
%                         '\.([/^\\*])','$1'));
% gp = fun(tf('s'))
% r = rlocus(gp,1);
% n = nyquist(gp);
% margin(gp)
%%
%P
A = [0 0 -v0*sin(phi0);
    0 0 v0*cos(phi0);
    0 0 0]
B = [0;0;Kp];
% C = [0 0 0];
% D = Kp;
C = [0 0 1];
D = 0;
K = (s*eye(3)-A);
MA = simplify(C*(K\B)+D)
[~,p] = numden(simplify(MA));
MF = simplify(MA/(1+MA))
[~,p] = numden(simplify(MF));
solve(p == 0,s)
%%
% gp = tf([.5],[1+.5])
% r = rlocus(gp,1)
% nyquist(gp)
% margin(gp)