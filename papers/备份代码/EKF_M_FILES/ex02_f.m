%   px+ = px + vx*dt
%   py+ = py + vy*dt
%   vx+ = vx + ax*dt + nx
%   vy+ = vy + ay*dt + ny

% Jacobian notation
%
%   Y_x = dy/dx

function [xo, XO_x, XO_u, XO_n] = ex02_f(x, u, n, dt)

px = x(1);
py = x(2);
vx = x(3);
vy = x(4);
ax = u(1);
ay = u(2);
nx = n(1);
ny = n(2);

px = px + vx*dt;
py = py + vy*dt;
vx = vx + ax*dt + nx;
vy = vy + ay*dt + ny;

xo = [px;py;vx;vy];

if nargout > 1 % we want jacobians
    % transition jacobian
    XO_x = [...
        1  0  dt  0
        0  1  0  dt
        0  0  1   0
        0  0  0   1];
    
    % control jacobian
    XO_u = [...
        0  0
        0  0
        dt 0
        0  dt];

    % perturbation jacobian
    XO_n = [...
        0  0
        0  0
        1  0
        0  1];
    
end

end
