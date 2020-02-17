function [ro, RO_r, RO_n] = move(r, u, n, dt)
% dt is not used by this function.

a = r(3);
dx = u(1) + n(1);
da = u(2) + n(2);

ao = a + da;
dp = [dx;0];

if ao > pi
    ao = ao - 2*pi;
end
if ao < -pi
    ao = ao + 2*pi;
end

if nargout == 1
    to = fromFrame2D(r, dp);
else    
    [to, TO_r, TO_dp] = fromFrame2D(r, dp);
    AO_a  = 1;
    AO_da = 1;
    
    RO_r = [TO_r ; 0 0 AO_a];
    RO_n = [TO_dp(:,1) zeros(2,1) ; 0 AO_da];
end
ro = [to;ao];

end

function f()
%%
syms x y a dx da real
X = [x;y;a];
u = [dx;da];
[xo, XO_x, XO_u] = move(X, u, zeros(2,1));
simplify(XO_x - jacobian(xo,X))
simplify(XO_u - jacobian(xo,u))
end

