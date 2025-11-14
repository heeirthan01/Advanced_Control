function [Cineq, Ceq] = bndCstPendubot(~,~,~,xF,qF)
    Ceq = xF-qF;            
    Cineq = [];  
end