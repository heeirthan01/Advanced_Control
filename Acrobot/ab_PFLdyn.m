function [dx] = ab_PFLdyn(t, x,qref,u,p,k)
z1 = x(1);
z2 = x(2);
n1 = x(3);
n2 = x(4);
q1_d = qref(1);
q1d_d = qref(2); 


%Gains
kp = k(1);
kd = k(2);

d11 = p.m1 * p.lc1^2 + p.m2 * (p.l1^2 + p.lc2^2 + 2*p.l1*p.lc2.*cos(n1))...
    + p.I1 + p.I2;
d12 = p.m2 * (p.lc2^2 + p.l1.*p.lc2.*cos(n1)) + p.I2;

h1 = -p.m2 * p.l1*p.lc2.*sin(n1).*n2.^2 - (2 * p.m2 * p.l1 * p.lc2 .* sin(n1)...
    .* n2 .* (z2 + q1d_d));

phi1 = ((p.m1 * p.lc1 + p.m2 * p.l1) * p.g .* cos(z1 + q1_d)) + (p.m2 * p.lc2 * p.g...
    .* cos((z1+q1_d) + n1));

v1 = u;
n2dot = -(1/d12)*(h1 + phi1) - (d11/d12)*v1;
dx = [z2;-kp*z1 - kd*z2;n2;n2dot];

end