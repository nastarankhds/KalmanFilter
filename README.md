# KalmanFilter
MATLAB Codes, Slalom Maneuver, CarMaker 
# Slalom Kalman Filter (MATLAB) – β (Sideslip) Estimation

MATLAB scripts to estimate sideslip angle (β) in a slalom maneuver using a Kalman Filter.

## How to run
In MATLAB:
matlab
addpath(genpath(pwd));   % add current folder
KF                      % run the script
## What this code does
Estimate sideslip angle **β** during a slalom maneuver using a **linear bicycle model** and a **Kalman Filter (KF)**.  
Input to the system: steering angle δ,
measurements of Kalman Filter: yaw rate r, lateral acceleration a_y

## Signals and parameters
- **States:** $x = [\beta,\ r]^\top$
- **Input:** $u = \delta$
- **Measurements:** $y = [r,\ a_y]^\top$
- **Params:** $m, I_z, a, b, C_f, C_r, V_x$  
Units: radians, m/s, N, kg, m, kg·m².

## Bicycle model (small-angle, linear tire)
**Continuous-time**

$$
\dot{\beta} = \frac{C_f+C_r}{mV_x}\beta + \left(\frac{aC_f - bC_r}{mV_x^2}-1\right) r - \frac{C_f}{mV_x}\delta
$$

$$
\dot{r} = \frac{aC_f - bC_r}{I_z}\beta +\frac{a^2C_f + b^2C_r}{I_z V_x} r - \frac{aC_f}{I_z}\delta
$$

Measurement model (two-sensor case):

$$
\begin{bmatrix} a_y ,\\ r \end{bmatrix}
= H \begin{bmatrix} \beta \\ r \end{bmatrix} + D\,\delta
$$

with

$$
H=
\begin{bmatrix}
0 & 1\\
-\frac{C_f+C_r}{m} & \frac{-aC_f + bC_r}{m} + V_x
\end{bmatrix},
\qquad
D=\begin{bmatrix}0\\ \frac{C_f}{m}\end{bmatrix}.
$$

Discrete model used in code:

$$
x_{k+1}=A x_k + B u_k + w_k,
\qquad
y_k = H x_k + D u_k + v_k,
$$

with $w_k\sim \mathcal N(0,Q)$ and $v_k\sim \mathcal N(0,R)$.

## Kalman Filter (discrete)

**Predict**

$$
\hat x_{k|k-1}=A\hat x_{k-1|k-1}+B u_k,
\qquad
P_{k|k-1}=A P_{k-1|k-1} A^\top + Q
$$

**Update**

$$
K_k=P_{k|k-1}H^\top(H P_{k|k-1}H^\top+R)^{-1}
$$

$$
\hat x_{k|k}=\hat x_{k|k-1}+K_k\!\left(y_k-(H\hat x_{k|k-1}+D u_k)\right),
\qquad
P_{k|k}=(I-K_k H)P_{k|k-1}.
$$

## Tuning tips
- Start with $Q=\mathrm{diag}(q_\beta,q_r)$ and $R=\mathrm{diag}(r_r,r_{a_y})$.
- Increase $q_\beta$ if β reacts too slowly; increase $r_{a_y}$ if $a_y$ is noisy.
- Yaw-rate-only case: use $H=[0\ 1]$ and scalar $R$.
