% Define symbolic variables in matlab:
% syms xc thp mc mp Jp L g u b

% xc and thp are the DOFs; u is the input
mc = 1.0;
mp = 0.1;
L = 0.5; % to midpoint of pendulum
g = 9.81;
b = 0; % damping in pendulum (optional)
Jp = (1/12)*mp*L^2; % thin rod assumption

F_aigym = 10; % magnitude of force (either left or right)


