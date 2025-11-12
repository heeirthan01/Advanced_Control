function [closestpos, closestvel] = closeststate(x, p_ref, v_ref)
p_true = x(1);
closestpos = inf; %closest position
index = 0;
for i=1:length(p_ref)
    if (norm(p_ref(i) - p_true) < closestpos)
        closestpos = p_ref(i);
        index = i;
    end
end
closestvel = v_ref(index);
end