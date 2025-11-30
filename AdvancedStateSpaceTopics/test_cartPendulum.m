clear all

if 0 % test "dummy animation", if you like
    tout = [0;1;2.5;5];
    xout = [0 80*pi/180; .7 96*pi/180; 0 90*pi/180; .2 -70*pi/180];
    animate_cartPendulum(tout,xout,.05,1)
end

X0 = [0;pi/2;0;0]; % equilibrium state
T = 0.02; % you might reset this
u = 10;
[tout,xout] = ode45(@(t,X) dX_cartPendulum(t,X,u),[0 T],X0);

dt = .005;
tout = tout;
animate_cartPendulum(tout,xout,dt,2)

A = zeros(4,4)
del = 1e-5;
for n=1:4
    xp = X0; xp(n) = xp(n) + del
    dxp = dX_cartPendulum(0,xp,0)
    xn = X0; xn(n) = xn(n) - del
    dxn = dX_cartPendulum(0,xn,0)
    A(:,n) = (dxp-dxn)./(2*del)
end
B = (dX_cartPendulum(0,X0,del) - dX_cartPendulum(0,X0,-del))./(2*del)


Co = ctrb(A,B)
c = cond(Co)
r = rank(Co)
Q = diag([100 1000 1 1])
R = 1
K = lqr(A,B,Q,R) % This uses the CONTINUOUS-TIME matrices.
% (It's better to use the discrete-time system's matrices!)



Xinit = [0;pi/2+2*pi/180;0;0]; % initial condition
if 0
    T = .4;
    [tout,xout] = ode45(@(t,X) dX_cartPendulum_CL(t,X,K,X0),[0 T],Xinit);
end

T = 0.02; % constant u value for this time amount of time
Tnow = 0;
Tfinal = 10;
Xinit = [0; pi/2+5*pi/180; 0; 0];
Xnow = Xinit;
uout = [];
ttot = [0];
xtot = Xinit';
utot = -K*(Xnow-X0);
while Tnow<Tfinal
    u = -K*(Xnow-X0);
    u = max(min(u,10),-10);
    if u>0 ,u=10; else, u=-1; end
    % if u>0 ,u=10; else, u=-10; end
    [tout,xout] = ode45(@(t,X) dX_cartPendulum(t,Xnow,u),Tnow+[0 T],Xnow);
    Tnow = Tnow+T;
    Xnow = xout(end,:)';
    ttot = [ttot; tout(2:end)];
    xtot = [xtot; xout(2:end,:)];
    utot = [utot; 0*tout(2:end)+u];

end

figure(1); clf
subplot(211)
plot(ttot,xtot); ylabel('States')
title('from: test\_cartPendulum.m')
subplot(212)
plot(ttot,utot); ylabel('Input')
xlabel('Time (s)')

fig = 2;
animate_cartPendulum(ttot,xtot,.01,fig)
