% Como resolver demora ao range de valores de Kd para manter a
% estabilidade?????
% Determinando valores de Kp e Kd

% Interest range for K_d
range = [-0.4, 0];
search_step = 1e-3;
%

%
%Escolhendo K_p = 0.2828:
syms K_p real;
syms K_d real;
syms s;

K_p = 0.2828;

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
 
stable_from = realmax;
stable_to = -realmax;
tic


for K_d = range(1):search_step:range(2)
    if isStable(subs(A, K_d))
        stable_to = K_d;
        if (K_d < stable_from)
            stable_from = K_d;
        end
    elseif (stable_to ~= -realmax & stable_to < K_d)
            break;
    end
    toc    
end

fprintf("O sistema é estável para K_d contido no intervalo (%.4f, %.4f]\n", stable_from, stable_to)
function ret = isStable(A)
    syms s;
    I = eye(length(A));

    den = det(s*I - A);
    den_roots = vpa(solve(den, s, "MaxDegree", length(A)));
    ret = ~any(real(den_roots) > 0);
end