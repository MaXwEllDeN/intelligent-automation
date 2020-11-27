function xdot = dynamicModelPID(t, x, kp, ki, kd, params)
    xg = params(1); %2
    yg = params(2); %2
    v0 = params(3); %0.25
    pd = params(4); %50
    
    xp = x(1,1);
    yp = x(2,1);
    phi = x(3,1);
    z1 = x(4,1);
    z2 = x(5,1);

    phi_d = atan2(yg-yp, xg-xp);
    e = phi_d - phi;
    e = atan2(sin(e),cos(e));

    delta = norm([xg-xp, yg-yp]);

    nu = v0;

    if delta < 0.01
        nu = 0;
    end

    xdot=[nu*cos(phi);
          nu*sin(phi);
          ki*pd*z1+(ki-kd*pd^2)*z2+(kp+kd*pd)*e;
          z2;
          -pd*z2+e];
end
