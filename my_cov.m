function [R,Q]=my_cov(y_measurement,betha,yawrate,cf,cr,m,lf,lr,delta,vx,N,iz)

psidot = yawrate;

for i=1:length(psidot)-1
    ay_model(i) = ((cf+cr)/m)*betha(i) + (((cf*lf)-(cr*lr))/(m*vx(i))) *psidot(i) - (cf/m)*delta(i);
end

y_model = [ay_model' psidot(1:end-1)];
noise_meas = y_measurement(1:end-1,:) - y_model;
R = cov(noise_meas);


x_real = [betha psidot]';
dt = 0.01;
for i =1:N-1
    x_1 = ((cr+cf)/(m*vx(i)))*betha(i) +((((cf*lf)-(cr*lr))/(m*vx(i)^2))-1)*psidot(i)-((cf)/(m*vx(i)))*delta(i);
    x_2 = (((cf*lf)-(cr*lr))/(iz))*betha(i) + (((cf*lf^2)+(cr*lr^2))/(iz*vx(i)))*psidot(i) - ((cf*lf)/(iz))*delta(i);
    x = [x_1 x_2]';

    noise_process(:,i) = x_real(:,i+1) - (x_real(:,i) + dt*x);
end
 Q = cov(noise_process');
end