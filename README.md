# KalmanFilter
MATLAB Codes, Slalom Maneuver, CarMaker 
# Slalom Kalman Filter (MATLAB) – β (Sideslip) Estimation

MATLAB scripts to estimate sideslip angle (β) in a slalom using a Kalman Filter.

## How to run
In MATLAB:
```matlab
addpath(genpath(pwd));   % add current folder
KF                      % run the script


## What this code does
Estimate sideslip angle **β** during a slalom using a **linear bicycle model** and a **Kalman Filter (KF)**.  
Inputs are steering angle δ, yaw rate r, lateral acceleration a_y, and vehicle speed Vx (constant or slowly varying).

## Signals and parameters
- **States:** \(x = [\beta,\ r]^\top\)
- **Input:** \(u = \delta\) (steering at wheels or road wheels)
- **Measurements:** \(y = [r,\ a_y]^\top\)  *(you can use only r if a_y is noisy)*
- **Params:** mass \(m\), yaw inertia \(I_z\), front/rear distances \(a,b\), cornering stiffnesses \(C_f,C_r\), speed \(V_x\)

Units: radians, m/s, N, kg, m, kg·m².

## Bicycle model (small-angle, linear tire)
Continuous-time:
\[
\begin{aligned}
\dot{\beta} &= -\frac{C_f+C_r}{mV_x}\,\beta
              + \left(\frac{-aC_f + bC_r}{mV_x}-1\right) r
              + \frac{C_f}{mV_x}\,\delta \\
\dot{r} &= -\frac{aC_f - bC_r}{I_z V_x}\,\beta
          -\frac{a^2C_f + b^2C_r}{I_z V_x}\, r
          + \frac{aC_f}{I_z}\,\delta
\end{aligned}
\]

Measurement model (two-sensor case):
\[
\begin{aligned}
y = \begin{bmatrix} r \\ a_y \end{bmatrix}
= \underbrace{\begin{bmatrix} 0 & 1 \\[2pt]
-\frac{C_f+C_r}{m} & \frac{-aC_f + bC_r}{m} + V_x \end{bmatrix}}_{H}\!
\begin{bmatrix} \beta \\ r \end{bmatrix}
+ \underbrace{\begin{bmatrix} 0 \\ \frac{C_f}{m} \end{bmatrix}}_{D}\, \delta
\end{aligned}
\]
*(The \(a_y\) row comes from \(m(a_y) = C_f(\delta-\beta-\tfrac{a}{V_x}r)+C_r(-\beta+\tfrac{b}{V_x}r)\) and \(a_y \approx V_x(\dot{\beta}+r)\).)*

Discrete-time form (used by the code):
\[
x_{k+1} = A x_k + B u_k + w_k,\quad
y_k = H x_k + D u_k + v_k
\]
with \(w_k \sim \mathcal{N}(0,Q)\), \(v_k \sim \mathcal{N}(0,R)\).  
\(A,B\) are from zero-order hold discretization of the continuous model at the sample time \(T_s\).

## Kalman Filter steps (discrete)
**Predict**
\[
\hat{x}_{k|k-1}=A\hat{x}_{k-1|k-1}+B u_k,\quad
P_{k|k-1}=A P_{k-1|k-1} A^\top + Q
\]

**Update**
\[
K_k = P_{k|k-1} H^\top (H P_{k|k-1} H^\top + R)^{-1}
\]
\[
\hat{x}_{k|k}=\hat{x}_{k|k-1}+K_k\big(y_k-(H\hat{x}_{k|k-1}+D u_k)\big)
\]
\[
P_{k|k}=(I-K_k H)P_{k|k-1}
\]

## Tuning tips
- Start with \(Q = \mathrm{diag}(q_\beta,\ q_r)\) and \(R = \mathrm{diag}(r_r,\ r_{a_y})\).
- Increase \(q_\beta\) if β lags the measured behavior too much; increase \(r_{a_y}\) if lateral-accel noise shakes the estimate.
- If you only use yaw rate: set \(H=[0\ 1]\), drop the \(a_y\) row, and use a scalar \(R\).

## Files in this repo
- `KF.m` – entry script calling prediction/update and plotting β
- `prediction.m` – state/time update
- `update.m` – measurement update
- `my_cov.m` – Q/R setup or helper (edit as needed)
- `slalom_0.25_20kph.xlsx` – example measured data (tiny)

## How to map your data
Edit `KF.m` to point to your file and columns:
```matlab
T = readtable('slalom_0.25_20kph.xlsx');   % or your filename
t        = T{:,1};           % time [s]
delta    = T{:,2};           % steering δ [rad]
r_meas   = T{:,3};           % yaw rate r [rad/s]
ay_meas  = T{:,4};           % lateral accel a_y [m/s^2]
Vx       = 20/3.6;           % example constant speed [m/s]
