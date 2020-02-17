function [p, P_r, P_y] = backProject(r, y)

if nargout == 1
    
    p_r = invScan(y);
    p   = fromFrame2D(r, p_r);

else
    
    [p_r, PR_y]    = invScan(y);
    [p, P_r, P_pr] = fromFrame2D(r, p_r);
    
    % here the chain rule !
    P_y = P_pr * PR_y;
        
end

end

function f()
%%
syms rx ry ra yd ya real
r = [rx;ry;ra];
y = [yd;ya];
[p, P_r, P_y] = backProject(r, y);
simplify(P_r - jacobian(p,r))
simplify(P_y - jacobian(p,y))
end