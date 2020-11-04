% Determinando valores de Kp

syms K_p;
syms s;


A = [0, 0, -sqrt(2)/10;
     0, 0, sqrt(2)/10;
     K_p/4, -K_p/4, -K_p]
 
I = eye(3);

den = det(s*I - A);
den_roots = solve(den, s);
root1 = feval(symengine, 'solve', strcat(char(den_roots(2)), '< 0'), 's', 'Real');
%R1: K_p < 0 | K_p >= 0.2828    --> Z_
root2 = feval(symengine, 'solve', strcat(char(den_roots(3)), '< 0'), 's', 'Real');
%R2: K_p >= 0.2828              --> R_

%R1 intersection R2:
%K_p >= 0.2828                  --> R_
