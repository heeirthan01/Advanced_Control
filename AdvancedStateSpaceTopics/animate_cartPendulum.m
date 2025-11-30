function animate_cartPendulum(tout,xout,dt,fig)

if ~exist('dt','var')
    dt = .05; % time between frames
end
if ~exist('fig','var')
    fig = 1;  % figure to draw the cart and pendulum
end

tu = [tout(1):dt:tout(end), tout(end)];
xc = interp1(tout,xout(:,1),tu);
thp = interp1(tout,xout(:,2),tu);

xb = [-1 1 1 -1 -1]*.2;
dd = .01;
yb = [dd dd 1 1 dd]*.2;
dy = max(yb);
Lp = 1.0; % total length (not just to the COM)
ccart = [.3 .7 1];
cpend = [.2 .2 .2];
dp = .03; % offset to pendulum pivot
rp = .03; % radius of support for pivot;
av = pi*[0:.02:1];
xu = [rp, rp*cos(av), -rp];
yu = [0, dp+rp*sin(av), 0] + dy;

figure(fig); clf
p1 = patch(xc(1)+xb,yb,'b','FaceColor',ccart,'LineWidth',2); hold on
axis image;
p3 = plot(xc(1)+xu,yu,'k-','LineWidth',2);
p2 = plot(xc(1)+[0 Lp]*cos(thp(1)),...
    dp+dy+[0 Lp]*sin(thp(1)),'k-','LineWidth',4,...
    'Color',cpend);
plot([-1 1]*10,[0 0],'k-','LineWidth',2);
drawnow
title(['t = ',num2str(tu(1),'%.2f'), ' (s)'])
set(gca,'FontSize',14)
axis(.8*[-2 2 -1 2]);

plot3([0 0],[-2 4],-1+[0 0],'k--')

pause(.9*dt);
for n=2:length(tu)
    set(p1,'XData',xc(n)+xb);
    set(p3,'XData',xc(n)+xu);
    set(p2,'XData',xc(n)+[0 Lp]*cos(thp(n)),...
        'YData',dp+dy+[0 Lp]*sin(thp(n)));
    title(['t = ',num2str(tu(n),'%.2f'), ' (s)'])
    drawnow
    pause(.9*dt);
end