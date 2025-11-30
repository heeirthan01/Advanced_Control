function x_next = EKF_stateFNpb(x, u)
%As in Spong 95
p.m1 = 1;
p.m2 = 1;
p.l1 = 1;
p.l2 = 1;
p.lc1 = 0.5;
p.lc2 = 0.5;
p.I1 = 0.083;
p.I2 = 0.33;
p.g = 9.8;
dt = 0.01;
x_next = rk4_step(@(xx, uu) dx_pb_lqr(0,xx, uu, p), x, u,dt);
x_next(1) = wrapToPi(x_next(1));
x_next(2) = wrapToPi(x_next(2));
end