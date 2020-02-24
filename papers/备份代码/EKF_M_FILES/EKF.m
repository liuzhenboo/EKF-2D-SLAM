% KF Kalman Filter
%
% I. System
%
%   x+ = f ( x, u, n )
%   y  = h ( x ) + v
%
%   x : state vector           - P : cova. matrix
%   u : control vector
%   n : perturbation vector    - Q : cov. matrix
%   y : measurement vector
%   v : measuremet noise       - R : cov. matrix
%
%   f() : transition function
%   h() : measurement function
%
%
%   
% II. Initialization
%
%   Define f(), and h().
%
%   Precise x, P, Q and R.
%
%
% III. Temporal loop
%
%   IIIa. Prediction of mean(x) and P at the arrival of u
%
%       Jacobian computation
%       F_x : jac. of x+ wrt. state
%       F_u : jac. of x+ wrt. control
%       F_n : jac. of x+ wrt. perturbation
%
%       x+ = f( x, u, 0 )
%       P+ = F_x * P * F_x' + F_n * Q * F_n'
%
%   IIIb. correction of mean(x) and P at the arrival of y
%
%       Jacobian computation
%       H   : jac. of y wrt. x
%
%       e  = h( x )          - expectation
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





























