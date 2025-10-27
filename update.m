function [x_est,p_est] = update(p_pred,H,R,x_pred, y_measurement,D,delta)

k = p_pred * H' *inv(H*p_pred*H' +R);
x_est = x_pred + k * (y_measurement' -( H*x_pred + D*delta));
p_est = (eye(size(p_pred)) - k * H) * p_pred;

end