
% Animation of slx file

data = out.xcurrout.Data;
time = out.xcurrout.Time;

%% Animate Pendubot
L1 = 1;
L2 = 1;  
fps = 200;       
skip = round(length(time)/(fps*5));  

figure; hold on;
axis equal;
axis([-L1-L2, L1+L2, -L1-L2, L1+L2]);
grid on;
title('Pendubot Swing-Up');
xlabel('X');
ylabel('Y');

for k = 1:skip:length(time)
    q1 = data(1,1,k);
    q2 = data(2,1,k);
    
    %model x,y tips
    x1 = L1*cos(q1);
    y1 = L1*sin(q1);
    x2 = x1+L2*cos(q1+q2);
    y2 = y1+L2*sin(q1+q2);

    cla;
    plot([0 x1 x2],[0 y1 y2],'-o','LineWidth',2,'MarkerFaceColor','k');
    plot(0,0,'ks','MarkerSize',10,'MarkerFaceColor','k'); 
    title(sprintf('Time = %.2f s', time(k)));
    drawnow;
end