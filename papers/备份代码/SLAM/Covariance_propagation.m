% Covariance propagation
%
%   1. Probability density
%
%       p(_x) = probability density of x, evaluated at the value _x
%
%       p(_x) = P(_x < x =< _x+dx) / dx   <-- definition of p(x)
%
%       We simply write : p(x)
%
%   2. Gaussian variable
%
%       x ~ N(x_, X) : x is Gaussian with mean x_ and covariances matrix X
%   
%       p(x) = 1 / sqrt( (2 * pi)^n * det(P) ) * exp (-0.5 * (x - x_)' * X^-1 * (x - x_) )
%
%   3. Expectation operator
%
%       E(f(x)) = integral(-inf)(+inf) (f(x) * p(x)) * dx
%
%       Linear operator: 
%           E( a * f(x) + b * g(x) ) = a * E(f(x)) + b * E(g(x))
%
%   4. Mean and variance of a scalar random variable
%
%       x_  = E(x)
%
%       v_x = E( (x - x_)^2 )
%           = integral(-inf)(+inf) p(x) * (x - x_)^2 * dx
%
%       if p(x) is uniform between 0 and 1:
%       
%           x_  = 0.5
%           v_x = integral(0)(1) (x - x_)^2
%
%   5. Standard deviation of a scalar random variable
%
%       s_x = sqrt(v_x)
%
%   6. cross-variance between 2 random scalar variables x and y
%
%       c_xy = E((x-x_) * (y-y_))
%
%   7. Random vector v = [x y]' - what is the covariances matrix V?
%
%       variance of x  : v_x
%       variance of y  : v_y
%       cross variance : c_xy
%
%       co-variances matrix: V = [v_x c_xy ; c_xy v_y]
%
%   8. Covariances matrix: generalization of 6. and 7.:
%
%       x  = [x1 x2 x3 ... xn]
%       x_ = [x1_ x2_ x3_ ... xn_]
%
%       c_ij = E((xi - xi_)(xj - xj_))
%       v_i  = E((xi - xi_)^2)          = c_ii
%
%       All these products may be performed in matrix form with:
%
%       X = E( (x-x_) * (x-x_)' )
%
%   9. Covariance propagation, y = f(x)  --> y ~ N(y_, Y)
%
%       Taylor up to the linear term:
%
%       y   ~ f(x_) + df/dx|x_ * (x - x_) 
%           = f(x_) + F_x * (x - x_)
%
%       Mean and covariances matrix of y = f(x):
%
%       y_  = f(x_)
%
%       Y   = E((y - y_) * (y - y_)')
%           = E(F_x * (x - x_) * (x - x_)' * F_x')
%           = F_x * E((x - x_) * (x - x_)') * F_x'
%           = F_x * X * F_x'
%
%
%   Enjoy -- Joan