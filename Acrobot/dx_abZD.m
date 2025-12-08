function dx = dx_abZD(t, x, p)
q2 = x(1);
q2d = x(2);
t1 = p.m2 * p.l1 * p.lc2 * sin(q2) * q2d^2;
t2 = p.m2 * p.lc2 * sin(q2) * p.g;
t3 = p.m2 * p.lc2^2 + p.m2 * p.l1 * p.lc2 * cos(q2) + p.I2;
q2ddot = (t1 + t2) / t3;
dx = [q2d;q2ddot];
end