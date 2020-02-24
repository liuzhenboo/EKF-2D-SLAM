% KF Kalman Filter
%
% I. System
%
%   x+ = F_x *x + F_u * u + F_n * n
%   y  = H * x + v
%
%   x : state vector           - P : cova. matrix
%   u : control vector
%   n : perturbation vector    - Q : cov. matrix
%   y : measurement vector
%   v : measuremet noise       - R : cov. matrix
%
%   F_x : transition matrix
%   F_u : control amtrix
%   F_n : pert. matrix
%   H   : measurement matrix
%
%   
% II. Initialization
%
%   Define F_x, F_u, F_n, and H.
%
%   Precise x, P, Q and R.
%
%
% III. Temporal loop
%
%   IIIa. Prediction of mean(x) and P at the arrival of u
%
%       x+ = F_x * x + F_u * u        ... ( + F_n * 0 )
%       P+ = F_x * P * F_x' + F_n * Q * F_n'
%
%   IIIb. correction of mean(x) and P at the arrival of y
%
%       e  = H * x          - expectation
%       E  = H * P * H'
%
%       z  = y - e          - innovation
%       Z  = R + E
%
%       K  = P * H' * Z^-1  - Kalman gain
%
%       x+ = x + K * z
%       P+ = P - K * H * P  //  P - K * Z * K' // and Joseph form
%
%   IV. Plot results
%
%   V. How to set up KF examples
%
%       1. Simulate system, get x, u and y trajectories
%
%       2. Estimate x with the KF. Get x and P trajectories.
%
%       3. Plot results.


































% KF  Kalman Filter, for linear-Gaussian systems.
%
%   I. System :
%
%       x+  = F_x * x + F_u * u + F_n * n
%       y   = H * x   + v
%
%       x   : state vector               - P : cov. matrix
%       u   : control vector
%       n   : perturbation vector        - Q : cov. matrix
%       y   : measurement vector      
%       v   : measurement noise vector   - R : cov. matrix
%
%       F_x : transition matrix
%       F_u : control matrix
%       F_n : perturbation matrix
%       H   : measurement matrix
%
%   II. Initialization: before start, do:
%
%       Define F_x, F_u, F_n and H
%
%       Precise x, P, Q, and R
%   
%
%   III. Prediction: at each new control input u, do:
%
%       x+ = F_x * x + F_u * u                 - mean update
%       P+ = F_x * P * F_x' + F_n * Q * F_n'   - covariances update
%
%   IV. Correction: at each measurement y, do:
%
%       e  = H * x            - expectation
%       E  = H * P * H'
%
%       z  = y - e            - innovation
%       Z  = R + E
%
%       K  = P * H' * Z^-1    - Kalman gain
%
%       x+ = x + K * z        - mean update
%       P+ = P - K * H * P    - covariances update
%
%   
%   V. To test a KF, do:
%
%       Build a simulator to generate trajectories for x, u, n, y and v.
%
%       Filter u and y to obtain the estimated x and P.
%
%       Plot the results. Compare x_simu with x and P.
%
