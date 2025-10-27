% Kalma Filter for Linear System Using Linear Tire Model
% Prediction Step : x_hat(k) = A * x_hat(k-1) + B * u(k);
%                   P_hat(k)     = A * P(k-1) * A' + Q;
% Correction Step : K(k) = P_hat(k) * H' * inv(H * P_hat(k) * H' + R);
%                   x(k) = x_hat(k) + K(k) * (z(k) - H* x_hat(k))
%                   P(k) = (I - K(k) * H ) *P_hat(k)
% -------------------------------------------------------------------------
% Linear System according to CM frames
% Dynamic Equations
% bethadot = (1/(m*vx))*(fyf*cos(delta) + fyr) - seidot;
% seiddot  = (1/iz)*(lf* fyf*cos(delta) - lr * fyr);
% yfdot    = vx*sin(phic) + betha*vx*cos(phic) + seidot*lcf*cos(phic);
% Tire Linear Model
% fyf = c_f * a_f;  fyr = c_r * a_r  ;
% -------------------------------------------------------------------------
% ßdot = ((cr+cf)/(m*vx)) ß + (((cf*lf-cr*lr)/(m*vx^2)) -1) psidot
% -(cf/(m*vx)) delta
% psidot = ((cf*lf-cr*lr)/Iz) ß + ((cf*lf^2 + cr*lr^2)/(Iz*vx))*psidot -
% ((cf*lf)/Iz)*delta
% -------------------------------------------------------------------------
clc; clear ; 
rng('default')
% close all;
% Loading the Data which obtained from IPG Carmaker 

filepath = 'C:\Users\nkhds\Documents\MATLAB\KF Adapting\maneuvers\slalom_0.25_20kph.xlsx';

aa = readmatrix(filepath);

% % % for some signals: 
% aa = aa(300:end,:);

delta = (aa(:,18) +  aa(:,19))/2;                            % Steering angle rad
vx = aa(:,29);                                               % inertia sensor
vy = aa(:,30);                                               % inertia sensor
ax = aa(:,27);                                               % inertia sensor
ay = aa(:,28);                                               % inertia sensor
Time = aa(:,32);
yawrate=aa(:,22);
betha = aa(:,31);                                            % Sensor.SAngle

% Parameters
iz = 2245.322;
lf =    1.024;
lr =    1.602;
meu=      0.9;
l  =  lf + lr;
hg =    0.561;
g  =     9.81;
m  =     1574.02;

cf = -1.8897e+05;
cr = -1.4990e+05;


% Measurements from IPG Carmaker
y_measurement(:,1) = ay;                     % ay
y_measurement(:,2) = yawrate;                % yawrate

% adding noise to measurements:
noise_variance_ay = 0.0003;               
noise_variance_yawrate = 0.00003;                  

noise_measurement_ay = wgn(length(y_measurement), 1, noise_variance_ay, 'linear');
noise_measurement_yawrate = wgn(length(y_measurement), 1, noise_variance_yawrate, 'linear');
noise_measurement_ay = noise_measurement_ay - mean(noise_measurement_ay);
noise_measurement_yawrate = noise_measurement_yawrate - mean(noise_measurement_yawrate);  % noise mean value is zero
noise_measurement =[noise_measurement_ay noise_measurement_yawrate];
y_measurement = y_measurement + noise_measurement;
figure
subplot 211
plot(Time,y_measurement(:,1),'b','linewidth',1),hold on
plot(Time,ay,'-r','linewidth',0.5)
grid minor
title 'Acceleration'
legend 'Noisy signal' 'no noise'
subplot 212
plot(Time,y_measurement(:,2),'b','linewidth',0.5),hold on
plot(Time,yawrate,'-r','linewidth',1)
grid minor
title 'Yaw Rate'
legend 'Noisy signal' 'no noise'

% Manuever Time, Sampling Rate, Number Of steps
dt = Time(101)-Time(100);
T = numel(Time)*dt;
N = numel(Time);

% Initial Values
x_init = [betha(1) yawrate(1)]';
P_init = diag([0.5  0.9]);
n = length(x_init);
[R,Q]= my_cov(y_measurement,betha,yawrate,cf,cr,m,lf,lr,delta,vx,N,iz);
var_estimation(:,1) = [P_init(1,1) ; P_init(2,2)];
std_estimation(:,1) = sqrt(var_estimation(:,1));
x_estimation(:,1) = x_init;
for i = 2:N-1

    % A , B , C , D
    a11 = (cr+cf)/(m*vx(i));
    a12 = (((cf*lf)-(cr*lr))/(m*vx(i)^2))-1;
    a21 = ((cf*lf)-(cr*lr))/(iz);
    a22 = ((cf*lf^2)+(cr*lr^2))/(iz*vx(i));
    A = [a11 a12
        a21 a22];

    b11 = (-cf)/(m*vx(i));
    b21 = (-cf*lf)/(iz);
    B = [b11
        b21];

    c11 = (cf+cr)/m;
    c12 = ((cf*lf)-(cr*lr))/(m*vx(i));
    c21 = 0;
    c22 = 1;
    C = [c11 c12
        c21 c22];

    d11 = -cf/m;
    d21 = 0;
    D = [d11
        d21];

%     sys = ss(A,B,C,D);
%     d_sys = c2d(sys,dt);
%     A = d_sys.A;
%     B = d_sys.B;

    % Prediction Step
    [x_pred,p_pred] = prediction (x_init,P_init,A,B,delta(i),Q);
    x_prediction(:,i) = x_pred;
    % Correction Step
    [x_est,p_est] = update(p_pred,C,R,x_pred, y_measurement(i,:),D,delta(i));

    x_estimation (:,i) = x_est;
    x_init = x_est;
    P_init = p_est;
    var_estimation(:,i) = [P_init(1,1) ; P_init(2,2)];
    std_estimation(:,i) = sqrt(var_estimation(:,i));

end

% Plot
figure
subplot 211
plot( Time(1:end-1),x_estimation(1,:),'r--','linewidth',1.5);hold on
plot(Time(1:end-1),betha(1:end-1),'b','linewidth',1.5);
legend ('Estimation', 'IPG Data', 'Estimation Uncertainty')
grid minor
title('Sideslip Angle (rad)')

subplot 212
plot( Time(1:end-1),x_estimation(2,:),'r--','linewidth',1.5);hold on
plot(Time(1:end-1),yawrate(1:end-1),'b','linewidth',1.5);
legend ('Estimation', 'IPG Data', 'Estimation Uncertainty')
title 'Yawrate (rad/s)'
grid minor
