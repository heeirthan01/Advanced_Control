function [dx] = dx_ab(t, x,u,p)
q1 = x(1, :);
q2 = x(2, :);
q1dot = x(3,:);
q2dot = x(4,:);
d11 = p.m1 * p.lc1^2 + p.m2 * (p.l1^2 + p.lc2^2 + 2*p.l1*p.lc2.*cos(q2))...
    + p.I1 + p.I2;
d22 = p.m2 * p.lc2^2 + p.I2 * ones(size(q2));
d12 = p.m2 * (p.lc2^2 + p.l1.*p.lc2.*cos(q2)) + p.I2;
d21 = d12;
h1 = -p.m2 * p.l1*p.lc2.*sin(q2).*q2dot.^2 - (2 * p.m2 * p.l1 * p.lc2 .* sin(q2)...
    .* q2dot .* q1dot);
h2 = p.m2 * p.l1 * p.lc2 .* sin(q2) .* q1dot.^2;
phi1 = ((p.m1 * p.lc1 + p.m2 * p.l1) * p.g .* cos(q1)) + (p.m2 * p.lc2 * p.g...
    .* cos(q1 + q2));
phi2 = p.m2 * p.lc2 * p.g .* cos(q1 + q2);

N = size(u,2);
qddot = zeros(2,N);

for k = 1:N
    Dk = [d11(k), d12(k);
        d21(k), d22(k)];

    rhs = [u(k)-(h1(k) + phi1(k)); %just switch this for acrobot
            - (h2(k) + phi2(k))];
    qddot(:,k) = Dk \ rhs;

% D = [d11 d12;
%     d21 d22];

qdot = [q1dot;q2dot];
% qddot = D \ ( [-(h1 + phi1); u - h2 + phi2]);
% dx = [qdot;qddot];
dx = [qdot; qddot];

end