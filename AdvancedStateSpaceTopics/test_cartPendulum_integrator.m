% This code "augments" the state space by including a 5th state that
% is the integral of cart position. (In this example, the control
% input, u, is artificially limited to have the values of only either
% +10 or -1.
%
% Katie Byl, ECE 238, UCSB

clear 

% You might wish to save A and B matrices. Otherwise, they original
% A and B matrices can be estimated numerically (after the "else")
if 0
    load AB_matrices A B % load the 4x4 A and 4x1 B matrices
else
    X0 = [0;pi/2;0;0]; % equilibrium state, for linearization
    del = 1e-5;
    A = zeros(4,4)
    for n=1:4
        xp = X0; xp(n) = xp(n) + del
        dxp = dX_cartPendulum(0,xp,0)
        xn = X0; xn(n) = xn(n) - del
        dxn = dX_cartPendulum(0,xn,0)
        A(:,n) = (dxp-dxn)./(2*del)
    end
    B = (dX_cartPendulum(0,X0,del) - dX_cartPendulum(0,X0,-del))./(2*del)
end


A = [A, zeros(4,1); 1 0 0 0 0];
B = [B; 0];

Co = ctrb(A,B)
r = rank(Co)
c = cond(Co)

% Normally, we would have not have zeros below, but xc, dxc/dt and
% the integral of xc are all interrelated here.
Q = diag([100 1000 0 0 30]) % 5th state is INTEGRAL of xc (cart position)
R = 1
Kc = lqr(A,B,Q,R)
eig(A-B*Kc) % if we were updating u continuously, use this K

T = 0.02; % constant u value for this time
ssct = ss(A,B,[],[]);
ssdt = c2d(ssct,T,'zoh')
% below, Kd is the more appropriate way to find control gains
Kd = lqr(ssdt,Q,R) % more appropriate to find DT control matrix
log(eig(ssdt.a-ssdt.b*Kd))/T 


X0 = [0; pi/2; 0; 0; 0]; % upright equilibrium, with 5th (integral) state
Tnow = 0;
Tfinal = 10;
Xinit = [0; pi/2+5*pi/180; 0; 0; 0]; % 5th value is integral (starts at zero)
Xnow = Xinit;
uout = [];
ttot = [0];
xtot = Xinit';

% The loop below holds u(t)=constant for a time step of T, and then
% simulates the continuous-time dynamics for T seconds. This is of
% course more accurate than using the DT linearized model (with Ad and Bd).

while Tnow<Tfinal
    %u = -Kc*(Xnow-X0); % using CT model, to set K
    u = -Kd*(Xnow-X0); % using DT model, for the given T
    u = max(min(u,10),-10);

    % Below is quite an odd -1 vs +10 choice! 
    % you can comment out the line below, to compare
    if u>0 ,u=10; else, u=-1; end

    % simulate CT dynamics for one time step:
    [tout,xout] = ode45(@(t,X) dX_cartPendulumInt(t,Xnow,u),Tnow+[0 T],Xnow);
    Xnow = xout(end,:)';
    ttot = [ttot; tout(2:end)];
    xtot = [xtot; xout(2:end,:)];
    if Tnow==0
        utot = 0*tout+u;
    else
        utot = [utot; 0*tout(2:end)+u];
    end
    Tnow = Tnow+T;

end

figure(3); clf
subplot(211)
plot(ttot,xtot)
subplot(212)
plot(ttot,utot)

animate_cartPendulum(ttot,xtot,.01,4) % only pays attention to first 2 columns of xtot