function [dx] = dx_ab_lqr(t, x, u, p)
if any(isnan(x))
    disp(x)
    error('NaN detected in state BEFORE computing D');
end
q1 = x(1);
q2 = x(2);
q1dot = x(3);
q2dot = x(4);

d11 = p.m1 * p.lc1^2 + p.m2 * (p.l1^2 + p.lc2^2 + 2*p.l1*p.lc2.*cos(q2))...
    + p.I1 + p.I2;
d22 = p.m2 * p.lc2^2 + p.I2;
d12 = p.m2 * (p.lc2^2 + p.l1.*p.lc2.*cos(q2)) + p.I2;
d21 = d12;
h1 = -p.m2 * p.l1*p.lc2.*sin(q2).*q2dot.^2 - (2 * p.m2 * p.l1 * p.lc2 .* sin(q2)...
    .* q2dot .* q1dot);
h2 = p.m2 * p.l1 * p.lc2 .* sin(q2) .* q1dot.^2;
phi1 = ((p.m1 * p.lc1 + p.m2 * p.l1) * p.g .* cos(q1)) + (p.m2 * p.lc2 * p.g...
    .* cos(q1 + q2));
phi2 = p.m2 * p.lc2 * p.g .* cos(q1 + q2);

D = [d11 d12;
    d21 d22];

qddot = D \ ([u ; 0] - [h1 + phi1; h2 + phi2]);

dx = [q1dot;
     q2dot;
     qddot(1);
     qddot(2)];

end