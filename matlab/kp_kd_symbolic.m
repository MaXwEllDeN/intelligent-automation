% Determinando valores de Kp e Kd

%Escolhendo K_p = 0.2828:
syms K_p real;
syms K_d real;
syms s;

K_p = 0.2828;
K_d = -.37218;
%K_d >= -0.37218: estável.

v_0 = 2;
x_0 = 0;
y_0 = 0;
phi_0 = pi/4;

y_g = 2;
x_g = 2;

c = 1;

dxdot_dx = 0;
dxdot_dy = 0;
dxdot_dz = 0;
dxdot_dphi = - v_0 * sin(phi_0);

dydot_dx = 0;
dydot_dy = 0;
dydot_dz = 0;
dydot_dphi = v_0 * cos(phi_0);

dzdot_dx = (y_g - y_0) / ((y_g-y_0)^2 + (x_g - x_0)^2);
dzdot_dy = -(x_g - x_0) / ((y_g-y_0)^2 + (x_g - x_0)^2);
dzdot_dz = -c;
dzdot_dphi = -1;

dphidot_dx = (K_p + c*K_d)*(y_g - y_0) / ((y_g-y_0)^2 + (x_g - x_0)^2);
dphidot_dy = -(K_p + c*K_d)*(x_g - x_0) / ((y_g-y_0)^2 + (x_g - x_0)^2);
dphidot_dz = -c^2*K_d;
dphidot_dphi = -(K_p + c*K_d);

A = [dxdot_dx, dxdot_dy, dxdot_dz, dxdot_dphi;
     dydot_dx, dydot_dy, dydot_dz, dydot_dphi;
     dzdot_dx, dzdot_dy, dzdot_dz, dzdot_dphi;
     dphidot_dx, dphidot_dy, dphidot_dz, dphidot_dphi
    ];
 

I = eye(length(A));

den = det(s*I - A);
den_roots = vpa(solve(den, s, "MaxDegree", length(A)));
    
isStable(den_roots)

function ret = isStable(den_roots)
    ret = ~any(real(den_roots) > 0);
end
%root1 = feval(symengine, 'solve', strcat(char(den_roots(2)), '< 0'), 's', 'Real');
%R1: Sempre é negativa
%root2 = feval(symengine, 'solve', strcat(char(den_roots(3)), '< 0'), 's', 'Real');
%R2: Sempre é negativa
%root3 = feval(symengine, 'solve', strcat(char(den_roots(4)), '< 0'), 's', 'Real');
%R3: Sempre é negativa
