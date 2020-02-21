function [X,Y] = cov2elli(x,P,ns,NP)

% Ellipsoidal representation of multivariate Gaussian variables (2D). Different
% sigma-value ellipses can be defined for the same covariances matrix. The most useful
% ones are 2-sigma and 3-sigma
% 用椭圆表达二维高斯状态变量分布
% x为2维状态的均值；p为协方差；ns为sigma-value；NP为画椭圆时离散点的个数；
%Ellipse points from mean and covariances matrix.
%   [X,Y] = COV2ELLI(X0,P,NS,NP) returns X and Y coordinates of the NP
%   points of the the NS-sigma bound ellipse of the Gaussian defined by
%   mean X0 and covariances matrix P.
%
%   The ellipse can be plotted in a 2D graphic by just creating a line
%   with line(X,Y).
%
persistent circle

if isempty(circle)
    alpha = 2*pi/NP*(0:NP);
    circle = [cos(alpha);sin(alpha)];
end

% 两种方法：
% 一：用SVD对P进行分解得到椭圆半轴d(1,1),d(2,2)和旋转矩阵R
% SVD method, R*d*d*R' = P
 [R,D]=svd(P);
 d = sqrt(D);
% % circle -> aligned ellipse -> rotated ellipse -> ns-ellipse
 ellip = ns*R*d*circle;

% 二：Choleski方法对P分解直接得到R*d
% Choleski method, C*C' = P
%C = chol(P)';
%ellip = ns*C*circle;

% output ready for plotting (X and Y line vectors)
X = x(1)+ellip(1,:);
Y = x(2)+ellip(2,:);
