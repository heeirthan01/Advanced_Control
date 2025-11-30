% cartPendulum_eom.m
%
% Classic Cart and Pendulum: Equations of Motion.
%
% Last updated Nov. 20, 2025
%
% Katie Byl. ECE/ME 238, UCSB.

clear all; format compact  % compact produces single-spaced output

% Define symbolic variables in matlab:
syms xc thp mc mp Jp L g u b

% xc: x position of cart
% thp: theta of pendulum
% Above, and their derivatives, are the 4 states

% 1a. GC's (generalized coordinates), and their derivatives:
GC = [{xc},{thp}]; % Using ABSOLUTE angles here
dxc = fulldiff(xc,GC); % time derivative. GC are variables (over time)here
d2xc = fulldiff(dxc,GC); % time derivative. GC are variables (over time)
dthp = fulldiff(thp,GC);
d2thp = fulldiff(dthp,GC);

% 1b. Geometry of the masses/inertias, given GC's are freely changing...
xp = xc + L*cos(thp); % thp=0 when pendulum lies sideways (on x axis)
yp = L*sin(thp);

% 1c. Define any required velocity terms (for masses):
dxp = fulldiff(xp,GC);
dyp = fulldiff(yp,GC);

% 2. Kinetic Energy:
T = (1/2)*(mc*dxc^2 + mp*dxp^2 + mp*dyp^2 + Jp*dthp^2);

% 3. Potential Energy:
V = mp*g*yp;  % only the pendulum changes height, wrt gravity

% 4. Lagrangian:
L = T-V

% 5. EOMs:
% q1 = phiw, q2 = thetab (order matters)
eq1 = fulldiff(diff(L,dxc),GC) - diff(L,xc);
eq1 = simplify(eq1)
eq2 = fulldiff(diff(L,dthp),GC) - diff(L,thp);
eq2 = simplify(eq2)

% 6. Xi: non-conservative terms
Xi1 = u; % force input to cart
Xi2 = -b*dthp; % damping on pendulum

% 7. M * d2q = Tau: find M and Tau; then use d2q = Tau \ M later.
M(1,1) = diff(eq1-Xi1,d2xc);
M(1,2) = diff(eq1-Xi1,d2thp);  % note: eq1 - Xi1 = 0
M(2,1) = diff(eq2-Xi2,d2xc);
M(2,2) = diff(eq2-Xi2,d2thp)  % note: eq2 - Xi2 = 0
% This yields the following 2x2 matrix of symbolic expressions:
% M =
% [       mc + mp, -L*mp*sin(thp)]
% [-L*mp*sin(thp),    mp*L^2 + Jp]
%
% note: M should be SYMMETRIC

Tau(1,1) = -(eq1-Xi1) + M(1,1)*d2xc + M(1,2)*d2thp;
Tau(2,1) = -(eq2-Xi2) + M(2,1)*d2xc + M(2,2)*d2thp;
Tau = simplify(Tau)

% This yields a 2x1 vectore Tau of symbolic expressions:
% Tau =
%   L*mp*cos(thp)*dthp^2 + u
% - b*dthp - L*g*mp*cos(thp)
% Again, use the expressions in M and Tau, and then solve for the
% two accelerations via: 
%                            d2q = M \ Tau 

% NO! DON'T DO WHAT IS BELOW!!!! (It's "slow" to use the lengthy
% symbolic code, as opposed to the shorter matrix approach in "7.")
%
%% Note: It is usually NOT appropriate to solve directly for each
%  d2qi, because you can solve the equations in MATLAB more rapidly
%  by leaving them in matrix form:  M*d2q = Tau, d2q = Tau / M,
%  where M is a dxd "mass matrix" and Tau is a dx1 vector which
%  will often include many nonlinear terms, for a robot system.
%
%  However, if you wish to isolate each acceleration, d2qi, you can
%  do so via solving the full SYSTEM of equations. Note that 
%  eq1 - Xi1 = 0, and also eq2-Xi2 = 0, so that our equations are:
eqns = [eq1-Xi1, eq2-Xi2];
S = solve(eqns, [d2xc, d2thp])

% This should result in struct S with the following two fields:
% S = 
%   struct with fields: [it will show a truncated output...]
%
% Full output is shown below:
%
% S = 
%   struct with fields:
% 
% S.d2xc
% ans =
% (cos(thp)*L^3*dthp^2*mp^2 - g*cos(thp)*sin(thp)*L^2*mp^2 + u*L^2*mp + Jp*cos(thp)*L*dthp^2*mp - b*sin(thp)*L*dthp*mp + Jp*u)/(- L^2*mp^2*sin(thp)^2 + L^2*mp^2 + mc*L^2*mp + Jp*mp + Jp*mc)
% S.d2thp
% ans =
% -(b*dthp*mc + b*dthp*mp - L*mp*u*sin(thp) + L*g*mp^2*cos(thp) - L^2*dthp^2*mp^2*cos(thp)*sin(thp) + L*g*mc*mp*cos(thp))/(- L^2*mp^2*sin(thp)^2 + L^2*mp^2 + mc*L^2*mp + Jp*mp + Jp*mc)

