function [x_pred,p_pred] = prediction (x,p,A,B,delta,Q)

dt = 0.01;

F = expm(A*dt);
G = F*(eye(2) - expm(-A*dt)).*inv(A)*B;
x_pred = F*x  +G*delta ;
p_pred = F*p .*F' + Q;

% x_pred = A*x  +B*delta ;
% p_pred = A*p *A' + Q;

end