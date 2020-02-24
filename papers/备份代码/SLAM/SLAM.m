% SLAM 2D Simultaneous Localization and Mapping
%
%   Map:
%       x: state vector, with
%           x(r) : robot pose: x(r) = [px, py, a]'  ;  r = [1 2 3]
%           x(l(i,:)): landmark # i: x(l(i,:)) = [xi yi]'
%
%           ex:  x = [x(r) x(l1) x(l2) x(l3) x(l5)]
%
%       P: covariances matrix
%
%       mapspace: [1 2 3 4 5 6 ... N]
%
%       querying map space: s = find(mapspace,size)
%       reserve map space:   mapspace(s) = 0;
%       free map space:      mapspace(s) = s;
%
%   EKF:
%
%       Prediction : when the robot moves
%       Correction : when robot makes a measurement of already mapped lmks
%       Initialization: at the discovery of new lmks
%       (reparametrization): to change lmk nature
%       (deletion): to remove bad lmks
%       
%   Functions:
%       robot motion f()   - x+ = f(x, u ,n)  |   x(r)+ = f(x(r), u, n)
%                                                   F_x = [F_r 0 ; 0 I]
%       lmk oservation h() - y = h(x) + v     |   y = h(x(r), x(l(i,:)))
%                                                   H = [H_r 0 H_i 0]
%       back-projection g() - x+ = g(x, y)    |   x(l(i,:)) = g(x(r), y)
%                                                   G = [I 0 0; 0 I 0; G_r 0 0]
%
%       Frame transforms:   
%           [l_r, LR_r, LR_l] = toFrame(x(r), x(l(i,:))) = toFrame(R, L) : at lmk correction
%
%           [l, L_r, L_lr] = fromFrame(R, L_R) : at lmk initialization
%
%



































% SLAM 2D Simultaneous Localization And Mapping
%
%   I. EKF operations
%
%       Prediction:        f() for robot motion
%       Correction:        h() for observations of already mapped landmarks
%       Initialization:    g() to add new landmarks to the map
%       Reparametrization: j() to change landmark encoding
%       Deletion:          ... to remove bad landmarks
%
%   II. Objects:
%
%       Map: 
%           - a moving robot with a sensor 
%           - a set of landmarks
%           - a global coordinate frame
%
%       Robot:
%           - a moving frame:   r  = [x y a]
%           - a motion equation r+ = f(r, u, n)    - for EKF prediction
%           - it has a sensor
%
%       Sensor:
%           - situated in robot frame
%           - a measurement function: y = h(r, l)  - for EKF corrections
%           - an inverse measurement: l = g(r, y)  - for EKF initialization
%
%       Landmarks:
%           - a position
%
%       EKF:
%           - a mean x:
%               . x(1:3) = r  : robot frame
%               . x(4:5) = l1 : first landmark
%               . x(6:7) = l2 : second landmark, etc
%           - a covariances matrix P:
%               . P(1:3,1:3) : cov. of robot frame
%               . P(4:5,4:5) : cov. of landmark 1
%               . P(1:3,6:7) : cross-variance of robot with landmark 2
%
%   III. Functions
%
%       Robot:
%           Odometry [r, R_r, R_u, R_n] = f(r, u, n)
%
%       Landmarks:
%           Frame transformation [l_r, LR_r, LR_l] =   toFrame(r, l)
%           Frame transformation [l  , L_r , L_lr] = fromFrame(r, l_r)
%           Reserve space in map:   [li, mapspace] = reserve(mapspace)
%
%       Sensor:
%           Projection      [y, Y_r, Y_l] = h(r, l)
%           Back-projection [l, L_r, L_y] = g(r, y)
%
%   IV. Sparse nature of function Jacobians:
%
%       x+ = f(x, u, n) is in fact r+ = f(r, u, n) = f(x([1 2 3]), u, n)
%       F_x is then [F_r 0 ; 0 I]
%
%       y = h(x) is in fact y = h(r, l2) = h(x([1 2 3 6 7])
%       H is then [H_r 0 H_l2 0 0]
%
%       x+ = g(x, y) is in fact l3 = g(r, y)      
%       G_x is then [I 0 0 ; 0 I 0 ; G_r 0 I]
%
%
