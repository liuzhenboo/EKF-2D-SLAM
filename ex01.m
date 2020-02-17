% define system
% x+ = x + u * dt + n
% y  = x + v

dt = 1;

F_x = 1;
F_u = dt;
F_n = 1;
H = 0.5;

Q = 1;
R = 1;

% simulated variables

X = 7;
u = 1;

% estimated variables

x = 0;
P = 1e4;

% trajectories
tt = 0:dt:40;
XX = zeros(1, size(tt,2));
xx = zeros(1, size(tt,2));
yy = zeros(1, size(tt,2));
PP = zeros(1, size(tt,2));

% perturbation levels
q = sqrt(Q);
r = sqrt(R);

% start loop
i = 1;
for t = tt

    % simulate
    n = q * randn;
    X = F_x * X + F_u * u + F_n * n;
    v = r * randn;
    y = H*X + v;
    
    % estimate - prediction
    x = F_x * x + F_u * u;
    P = F_x * P * F_x' + F_n * Q * F_n';
    
    % correction
    e = H * x;
    E = H * P * H';
    
    z = y - e;
    Z = R + E;
    
    K = P * H' * Z^-1;
    
    x = x + K * z;
    P = P - K * H * P;
    
    % collect data
    XX(:,i) = X;
    xx(:,i) = x;
    yy(:,i) = y;
    PP(:,i) = diag(P);

    % update index
    i = i + 1;
end

% plot
plot(tt,XX,'r',tt,xx,'c',tt,yy,'b',tt,XX+3*sqrt(PP),'g',tt,XX-3*sqrt(PP),'g');
legend('truth','estimate','measurement','+/- 3 sigma')

