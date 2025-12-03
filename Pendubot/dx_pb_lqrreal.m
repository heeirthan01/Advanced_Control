function [dx] = dx_pbreal(t, x,u,p)
q1 = x(1);
q2 = x(2);
q1dot = x(3);
q2dot = x(4);
d11 = p.theta1 + p.theta2 + (2 .* p.theta3 .* cos(q2));
d22 = p.theta2 * ones(size(q2));
d12 = p.theta2 + (p.theta3 .* cos(q2));
d21 = d12;
h1 = - p.theta3 .* sin(q2).*q2dot.^2 - (2 .* p.theta3 .* sin(q2) .* q1dot.*q2dot);
h2 = p.theta3 .* sin(q2) .* q1dot.^2;
phi1 = (p.theta4 .* cos(q1) .* p.g) + (p.theta5 .* p.g .* cos(q1 + q2)); 
phi2 = p.theta5 .* p.g .* cos(q1 + q2);

D = [d11 d12;
    d21 d22];

qddot = D \ ([u ; 0] - [h1 + phi1; h2 + phi2]);

dx = [q1dot;
     q2dot;
     qddot(1);
     qddot(2)];

end