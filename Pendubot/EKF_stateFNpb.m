function x_next = EKF_stateFNpb(x, u)
%From REal pendulum in lab
p.theta1 = 0.0308; 
p.theta2 = 0.0106;
p.theta3 = 0.0095;
p.theta4 = 0.2087;
p.theta5 = 0.063;
p.g = 9.8;
dt = 0.01;
x_next = rk4_step(@(xx, uu) dx_pb_lqrreal(0,xx, uu, p), x, u,dt);
x_next(1) = wrapToPi(x_next(1));
x_next(2) = wrapToPi(x_next(2));
end