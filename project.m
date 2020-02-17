function [y, Y_r, Y_p] = project(r, p)

if nargout == 1
    
    p_r = toFrame2D(r, p);
    y   = scan(p_r);
    
else
    
    [p_r, PR_r, PR_p] = toFrame2D(r, p);
    [y, Y_pr]   = scan(p_r);
    
    % here the chain rule !
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