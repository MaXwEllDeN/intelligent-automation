syms Kd
syms Kp
syms c
syms s
syms v0
syms phi0
%%
% syms x
% syms y
% syms y_g
% syms x_g
% syms phi
% syms r
% 
% %phi_g = atan((y_g-y)/(x_g-x));
% %e = phi_g-phi;
% syms e;
% z = [x;y;phi;r];
% 
% A = [v0*cos(phi);
%     v0*sin(phi);
%     (Kp+c*Kd)*e-Kd*c^2*r;
%     -c*r+e]
% C = [(Kp+c*Kd)*e-Kd*c^2*r];
% syms linA [4 4]
% syms linC [1 4]
% syms linB [4 1]
% for i=1:4
%     for j=1:4
%         linA(i,j)=diff(A(i),z(j));
%     end    
%     linB(i)=diff(A(i),e);
%     linC(i)=diff(C,z(i));
% end
% 
% linD=diff(C,e);
% 
% linA
% linB
% linC
% linD
%%
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
%%
solve(p == 0,s)
%%
fun = matlabFunction(subs(GP,[Kp,Kd,c],[.5,1,1]));
fun = str2func(regexprep(func2str(fun),...
                        '\.([/^\\*])','$1'));
gp = fun(tf('s'))
r = rlocus(gp,1)