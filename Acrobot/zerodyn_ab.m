clear;
close all;

params_pend;
tF = 20;
step = 0.1; 
xpos = -pi:step:pi;
figure;
hold on;
for i =1:length(xpos)
    X0 = [xpos(i);0];
    tsPan = [0, tF];
    dt = 0.001;
    S = odeset('RelTol',1e-8,'AbsTol',1e-8);  % low resolution
    [tout,xout] = ode45(@(t,x) dx_abZD(t,x,p),tsPan,X0,S);
    
    q2traj = xout(:,1);
    q2dtraj = xout(:,2);
    plot(q2traj, q2dtraj)
  
end
%plot(tout,q2traj)
plot(pi,0,'ks','MarkerSize',10,'MarkerFaceColor','k');
hold on;
plot(0,0,'ks','MarkerSize',10,'MarkerFaceColor','k');
hold off;
title('Phase Portrait Zero Dynamics')
xlabel('q2')
ylabel('q2_dot')
% %% Animate Acrobot
% L1 = p.l1;
% L2 = p.l2;  
% fps = 100;       
% skip = round(length(tout)/(fps*10));  
% 
% figure; hold on;
% axis equal;
% axis([-L1-L2, L1+L2, -L1-L2, L1+L2]);
% grid on;
% title('Acrobot Swing-Up');
% xlabel('X');
% ylabel('Y');
% 
% for k = 1:skip:length(tout)
%     q1 = pi/2;
%     q2 = xout(k,1);
% 
%     %model x,y tips
%     x1 = L1*cos(q1);
%     y1 = L1*sin(q1);
%     x2 = x1+L2*cos(q1+q2);
%     y2 = y1+L2*sin(q1+q2);
% 
%     cla;
%     plot([0 x1 x2],[0 y1 y2],'-o','LineWidth',2,'MarkerFaceColor','k');
%     plot(0,0,'ks','MarkerSize',10,'MarkerFaceColor','k'); 
%     title(sprintf('Time = %.2f s', tout(k)));
%     drawnow;
%     pause(dt);
% end