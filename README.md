# KalmanFilter
MATLAB Codes, Slalom Maneuver, CarMaker 
# Slalom Kalman Filter (MATLAB) – β (Sideslip) Estimation

MATLAB scripts to estimate sideslip angle (β) in a slalom using a Kalman Filter.

## How to run
In MATLAB:
matlab
addpath(genpath(pwd));   % add current folder
KF                      % run the script
## What this code does
Estimate sideslip angle **β** during a slalom using a **linear bicycle model** and a **Kalman Filter (KF)**.  
Inputs: steering angle δ, yaw rate r, lateral acceleration a_y, and vehicle speed Vx.

## Signals and parameters
- **States:** \( x = [\beta,\ r]^\top \)  
- **Input:** \( u = \delta \)  
- **Measurements:** \( y = [r,\ a_y]^\top \)  
- **Params:** \( m, I_z, a, b, C_f, C_r, V_x \)  
Units: radians, m/s, N, kg, m, kg·m².

## Bicycle model (small-angle, linear tire)
Continuous-time:
$$
\dot{\beta} = -\frac{C_f+C_r}{mV_x}\,\beta
+ \left(\frac{-aC_f + bC_r}{mV_x}-1\right) r
+ \frac{C_f}{mV_x}\,\delta
$$
$$
\dot{r} =
-\frac{aC_f - bC_r}{I_z V_x}\,\beta
-\frac{a^2C_f + b^2C_r}{I_z V_x}\, r
+ \frac{aC_f}{I_z}\,\delta
$$

Measurement model (two-sensor case):
$$
y = \begin{bmatrix} r \\ a_y \end{bmatrix}
= H \begin{bmatrix} \beta \\ r \end{bmatrix} + D\,\delta,
\qquad
H=
\begin{bmatrix}
0 & 1\\
-\frac{C_f+C_r}{m} & \frac{-aC_f + bC_r}{m} + V_x
\end{bmatrix},
\quad
D=\begin{bmatrix}0\\ \frac{C_f}{m}\end{bmatrix}.
$$

Discrete model used in code:
$$
x_{k+1}=A x_k + B u_k + w_k, \qquad
y_k = H x_k + D u_k + v_k,
$$
with \( w_k\sim \mathcal N(0,Q) \) and \( v_k\sim \mathcal N(0,R) \).

## Kalman Filter (discrete)
**Predict**
$$
\hat x_{k|k-1}=A\hat x_{k-1|k-1}+B u_k,\qquad
P_{k|k-1}=A P_{k-1|k-1} A^\top + Q
$$
**Update**
$$
K_k=P_{k|k-1}H^\top(H P_{k|k-1}H^\top+R)^{-1}
$$
