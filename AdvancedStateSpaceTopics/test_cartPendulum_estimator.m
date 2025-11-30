clear 

load AB_matrices A B % load the 4x4 A and 4x1 B matrices

% Kalman filtering example

% Assume true angle, thp, is the sum of an initial (unknown)
% offset, th0, plus the measured encoder angle, thm:
% thp = th0 + thm
A = [A, 0*A(:,2); 0 0 0 0 0];
T = 0.005; % constant u value for this time
Ad = expm(A*T)   % DISCRETE-TIME dynamics, for KALMAN FILTER...
B = [B; 0];
C = [1 0 0 0 0; 0 1 0 0 -1]; % measure cart position, and pendulum angle,
% but assume pendulum angle MAY have some offset to it

ssct = ss(A,B,C,0)
ssdt = c2d(ssct,T,'zoh')
Bd = ssdt.b

Co = ctrb(A,B)
r = rank(Co) % NOT CONTROLLABLE, b/c 
c = cond(Co)


Q = diag([100 1000 1 1]) % 5th state is INTEGRAL of xc (cart position)
R = 1
K = lqr(A(1:4,1:4),B(1:4),Q,R)
K = [K, 0]

% Note: we CAN do "pole placement" directly, to find gains for Lp:
p_est = [-15+5*j, -15-5*j,-30, -40, -50];
Ob = obsv(Ad,C)
Lp = place(Ad', C', exp(T*p_est))'

open_loop_poles_dyn = eig(Ad-Bd*K)
open_loop_poles_est = eig(Ad-Lp*C)



th0 = pi/2 + 10*pi/180; % CONSTANT offset to actual angle: thp = th0 + thm
thm = 0 + pi/2; % always measures 0, at t=0 (let's say)
thoff = th0-thm
% Note: b/c we EXPECT it will start at pi/2, this is simply a constant
% added to the measurement. The actually encoder reading itself will
% begin at 0, so we always add pi/2 to any measurement, for the upright
% case
X0 = [0; th0; 0; 0; thoff]; % upright equilibrium, with 5th (integral) state



%th_offset = 0.5*pi/180; % actual initial pendulum angle

% Consider a "push" of some amount, applied at the cart
u_sig = 1;
% We will test with u of plus and minus this value, to estimate Q
X0 = [0; pi/2; 0; 0; 0];
up = u_sig;
[tp,xp] = ode45(@(t,X) dX_cartPendulum_offset(t,X,up),[0 T],X0);
un = -u_sig; % negative sign, same push
[tn,xn] = ode45(@(t,X) dX_cartPendulum_offset(t,X,un),[0 T],X0);
Q = cov([xp(end,:)',xn(end,:)']') % 5x5 covariance matrix
Q(5,5) = 1e-6; % small amt

% measurement may have noise (e.g., discretization)
% ...but even with no noise, we need to estimate velocities, and
% we may have a constant offset in the actual pendulum angle, too.

R = (0.001*pi/180)^2 % 1x1, single angle measurement's variance

% Finally, what's our uncertainty on the initial state?
% Perhaps, everything is "almost perfectly" known, except for that
% constant offset value?
del = 1e-3; % some "small amount" of uncertainty, not well known.
P0 = diag([del del del del .1]) % this is VARIANCE


% First, initialize actual and estimated states
Tnow = 0;
Tfinal = 10;
Xinit = [0; pi/2-thoff; 0; 0; thoff]; % 5th value is integral (starts at zero)
Xhat0 = [0; pi/2; 0; 0; 0]; % initial ESTIMATE of the state

Xnow = Xinit;
ttot = [0];
xtot = Xinit'; % sadly, ode45 has one state per ROW; this matches that
xhat = Xhat0; % 5xN matrix, with each COLUMN being a state
utot = -K*(Xnow-X0);
Pdata(1).Pp = P0; % (Pp=P+) initial covariance on estimate. (no Pm: P-)

%keyboard

Xhat = Xhat0; % initial ESTIMATE of states
xhatm = Xhat;
xhatp = Xhat;

% Over time, via discrete steps, we will:
% - Calculate control BASED ON THE ESTIMATE
% - Simulate TRUE dynamics
% - Update our ESTIMATE

for k=2:800
    u = -K*(Xhat-X0);
    %u = max(min(u,10),-10);
    %if u>0 ,u=10; else, u=-1; end
    % First, simulate and store TRUE state information:
    [tout,xout] = ode45(@(t,X) dX_cartPendulum_offset(t,X,u),[0 T],Xnow);
    Xnow = xout(end,:)';
    xtot(k,:) = Xnow';

    % Next, perform the 5 steps for the Kalman filter:
    % Step 1: a priori estimate: simulate true dynamics, for one time step, 
    % based on xhat at time step "k-1":
    [tout2,xout2] = ode45(@(t,X) dX_cartPendulum_offset(t,X,u),[0 T],xhatp(:,k-1));
    xhatm(:,k) = xout2(end,:)';

    % Step 2: update a priori covariance:
    % NOTE: use the DISCRETE-TIME matric, Ad, for variance propagation...
    Pdata(k).Pm = Ad*Pdata(k-1).Pp*Ad' + Q; % propagate variance (one time step)
    Pm = Pdata(k).Pm;

    % Step 3: Calculate the Kalman gain
    Pdata(k).Lp = Pm*C' * (C*Pm*C' + R)^-1;
    Kal = Pdata(k).Lp;

    % Step 4: a posteriori estimate, using Kalman gain
    y = C*Xnow; % measurement, from true state
    xhatp(:,k) = xhatm(:,k) + Kal*(y - C*xhatm(:,k));
    %xhatp(:,k) = xhatm(:,k) + Lp*(y - C*xhatm(:,k));

    % Step 5: a posteriori covariance update
    %Pdata(k).Pp = ((eye(5)-Lp*C) * Pm * (eye(5)-Lp*C)' + Lp*R*Lp');
    Pdata(k).Pp = ((eye(5)-Kal*C) * Pm * (eye(5)-Kal*C)' + Kal*R*Kal');

    Xhat = xhatp(:,k); % most recent estimate, for next time step...
    


end
ttot = [0:k-1]*T;


figure(5); clf
plot(xhatm')
%keyboard

% figure(3); clf
% subplot(211)
% plot(ttot,xtot)
% subplot(212)
% plot(ttot,utot)

animate_cartPendulum(ttot,xtot,.01,4) % only pays attention to first 2 columns of xtot