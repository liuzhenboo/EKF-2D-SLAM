%p是全局坐标系的点，r是局部坐标系的位姿(x,y,a) 
% 投影模型
function [y, Y_r, Y_p] = project(r, p)


if nargout == 1    
    p_r = toFrame2D(r, p);
    %返回路标到局部坐标系原点的距离以及正切值
    y   = scan(p_r);
else
    
    [p_r, PR_r, PR_p] = toFrame2D(r, p);
    [y, Y_pr]   = scan(p_r);
    
    % 链式法则求导
    Y_r = Y_pr * PR_r;
    Y_p = Y_pr * PR_p;
        
end

end

function f()
%%
syms px py rx ry ra real
r = [rx;ry;ra];
p = [px;py];
[y, Y_r, Y_p] = project(r, p);
simplify(Y_r - jacobian(y,r))
simplify(Y_p - jacobian(y,p))
end