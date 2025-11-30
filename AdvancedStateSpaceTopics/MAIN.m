%% MAIN.m
%
% This code simulates various scenarios for control and estimation of
% a cart-pendulum system.
%
% Katie Byl, ECE 238, UCSB

clear all   % clear all variables in the workspace
clc

fprintf('All you should need to do is to in the Workspace is to hit enter\n')
fprintf('from time to time. In addition, please read through the examples\n')
fprintf('and look at the various code.\n\n')

fprintf('There are a total of 4 "parts" in this tutorial.\n\n')

fprintf('Once this tutorial is complete, you should experiment by modifying\n')
fprintf('various m-file.\n\n\n')

%% 1. We can derive equations of motion, using the Lagrangian approach.
%     This is the starting point for simulating a mechanical dynamic 
%     system. The m-file "fulldiff.m" is required, to implement the
%     chain rule here.

fprintf('Part 1: Finding equations of motion.\n\n')
fprintf('Open this file, to see details: cartPendulum_eom.m\n')

input('Hit enter to open the file and run this code...')
edit cartPendulum_eom
for n=1:20, fprintf('-- '), end; fprintf(['\n'])

cartPendulum_eom

for n=1:20, fprintf('-- '), end; fprintf(['\n'])

fprintf('\nThe cart-pendulum has d=2 DOFs, q1=xc (cart position), and\n')
fprintf('q2=thp (pendulum angle in radians).\n\n')

fprintf('\nThe key outputs here are: matrix M and vector Tau.\n')
fprintf('You could copy and paste the results into another file,\n')
fprintf('or you could write code to print out the results in a\n')
fprintf('nice format directly in a file, or to the workspace, e.g.:\n\n')

fprintf('M = [%s, %s\n',M(1,1),M(1,2))
fprintf('   %s, %s];\n\n',M(2,1),M(2,2))

fprintf('Tau = [%s\n',Tau(1))
fprintf('   %s];\n\n',Tau(2))

input('Hit enter to continue...')
for n=1:20, fprintf('-- '), end; fprintf(['\n'])
fprintf('Then, the accelerations of DOFs q1 and q2 can be found as:\n\n')
fprintf('d2q = M \\ Tau; \n\n')


input('Hit enter to continue...')
for n=1:20, fprintf('-- '), end; fprintf(['\n'])
fprintf('Alternatively, you can copy and paste the direct solution, which\n')
fprintf('is stored in the variable S. However, this is a slower way to\n')
fprintf('find the solution than using the backslash, in general.\n\n')

fprintf('For example, here are the expressions stored in S:\n\n')

fprintf('d2xc = %s\n\n',S.d2xc)
fprintf('d2thp = %s\n\n',S.d2thp)

fprintf('The fact that both equations have the same expression in the denominator\n')
fprintf('is not a coincidence; it relates to the matrix inverse (from M \\ Tau).\n\n')


for n=1:20, fprintf('-- '), end; fprintf(['\n'])
for n=1:20, fprintf('-- '), end; fprintf(['\n'])

%% 2. Run a baseline LQR controller, but constrain u to be either +10 or -1,
%     artificially, to see what happens.

fprintf('\nPart 2: LQR with u either equal to +10 or -1.\n\n')
fprintf('Open this file, to see details: test_cartPendulum.m\n')
input('Hit enter to open the file and run this code...')
edit test_cartPendulum
for n=1:20, fprintf('-- '), end; fprintf(['\n'])

test_cartPendulum

fprintf('You should note that the pendulum does NOT arrive at xc=0.\n')
fprintf('Instead, it has a steady-state offset in position.\n\n')
fprintf('(You can experiment by modifying this code later...\n')
fprintf('for example, by changing matrices Q and R.)\n\n')

fprintf('One way to try to fix this is to include a 5th state, the integral\n')
fprintf('of xc, so that state feedback (u=-K*x) includes an integral term.\n\n')

input('Hit enter to open the file and run this code...')
for n=1:20, fprintf('-- '), end; fprintf(['\n'])
for n=1:20, fprintf('-- '), end; fprintf(['\n'])

%% 3. Add a 5th state, the integral of xc, and run again with new gains.

fprintf('\nPart 3: Implement integral control.\n')
fprintf('To do this in the framework of u=-K*x we need a state that is this\n')
fprintf('integral quantity. So, we add a 5th state that is that integral of\n')
fprintf('xc (cart position). This changes the CT A and B matrices, as discussed\n')
fprintf('in class, and you can track down the changes (please do, to be sure you\n')
fprintf('understand them) in the following m-file:\n')
fprintf('    test_cartPendulum_integrator')

input('Hit enter to open the file and run this code...')
for n=1:20, fprintf('-- '), end; fprintf(['\n'])

edit test_cartPendulum_integrator
test_cartPendulum_integrator

fprintf('\nAs before, keep in mind you can modify and experiment with this code, later.\n')

input('Hit enter to open the file and run this code...')
for n=1:20, fprintf('-- '), end; fprintf(['\n'])
for n=1:20, fprintf('-- '), end; fprintf(['\n'])

%% 4. Implement a Kalman filter.


fprintf('\nPart 4: Implement a Kalman filter.\n\n')
fprintf('There are two new elements to explore in this final part:\n')
fprintf('   (a) Implementing a discrete-time (DT) Kalman filter\n')
fprintf('   (b) Estimating an unknown initial offset in pendulum angle\n\n')

fprintf('The rest is left to explore. Here is the high-level script:\n')
fprintf('    test_cartPendulum_estimator\n')
edit test_cartPendulum_estimator

%% End of tutorial

input('Hit enter to open the file and run this code...')
for n=1:20, fprintf('-- '), end; fprintf(['\n'])
for n=1:20, fprintf('-- '), end; fprintf(['\n'])
test_cartPendulum_estimator

fprintf('\n\nThat is it, for the formal walk-through! There is a lot to explore\n')
fprintf('within the individual scripts. Please email me, if you have questions:\n')
fprintf('   katiebyl@ucsb.edu\n\n')
