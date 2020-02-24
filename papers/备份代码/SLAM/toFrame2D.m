function [p_r, PR_r, PR_p] = toFrame2D(r , p)

t = r(1:2);
a = r(3);

R = [cos(a) -sin(a) ; sin(a) cos(a)];

p_r = R' * (p - t);

if nargout > 1
    px = p(1);
    py = p(2);
    x = t(1);
    y = t(2);
    
    PR_r = [...
        [ -cos(a), -sin(a),   cos(a)*(py - y) - sin(a)*(px - x)]
        [  sin(a), -cos(a), - cos(a)*(px - x) - sin(a)*(py - y)]];
    PR_p = R';
    
end

end

%%
function f()
%%
syms x y a px py real
r = [x y a]';
p = [px py]';
p_r = toFrame2D(r, p);
PR_r = jacobian(p_r, r)
end