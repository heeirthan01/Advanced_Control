function [next_state] = dx_lqr(x, u, p, dt)

xpos = x(1);
v = x(2);

gt = p.gt;
et = p.et;
pos_next = xpos + dt * v;
v_next = v + ((et * u) - (gt * cos(3*xpos))) * dt;
next_state = [pos_next; v_next];
end