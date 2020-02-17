% SLAM algorithm

% I. INITIALIZE
%
%   0. System def.

% System noise
q = [0.01;0.005];
Q = diag(q.^2);
% Measurement noise
m = [.15; 1*pi/180];
M = diag(m.^2);
% randn('seed',1);

%
%   1. Simulator
%       R: robot pose u: control
%       W: features

R = [0;-2.5;0];
u = [0.1;0.05];
W = cloister(-4,4,-4,4);

y = zeros(2,size(W,2));
%   2. Estimator
x = zeros(numel(R)+numel(W), 1);
P = zeros(numel(x),numel(x));
mapspace = 1:numel(x);
l = zeros(2, size(W,2));

r = find(mapspace,numel(R));
mapspace(r) = 0;
x(r)   = R;
P(r,r) = 0;

%   3. Graphics
mapFig = figure(3);
cla;
axis([-6 6 -6 6])
axis square
WG = line('parent',gca,...
    'linestyle','none',...
    'marker','o',...
    'color','r',...
    'xdata',W(1,:),...
    'ydata',W(2,:));
RG = line('parent',gca,...
    'marker','.',...
    'color','r',...
    'xdata',R(1),...
    'ydata',R(2));
rG = line('parent',gca,...
    'linestyle','none',...
    'marker','+',...
    'color','b',...
    'xdata',x(r(1)),...
    'ydata',x(r(2)));

lG = line('parent',gca,...
    'linestyle','none',...
    'marker','+',...
    'color','b',...
    'xdata',[],...
    'ydata',[]);

eG = zeros(1,size(W,2));
for i = 1:numel(eG)
    eG(i) = line(...
        'parent', gca,...
        'color','g',...
        'xdata',[],...
        'ydata',[]);
end

reG = line(...
    'parent', gca,...
    'color','m',...
    'xdata',[],...
    'ydata',[]);

% II. Temporal loop

for t = 1:200
    
    % 1. Simulator
    n = q.*randn(2,1);
    % R = move(R, u, zeros(2,1));
    % R是机器人仿真位置；y是当前观测；
    R = move(R, u, n);
    for lid = 1:size(W,2)
        v = m.*randn(2,1);
        y(:,lid) = project(R, W(:,lid)) + v;
    end
    
    % 2. Filter
    %   a. Prediction
    %   CAUTION this is sub-optimal in CPU time
    % [x(r), R_r, R_n] = move(x(r), u, n);
    % x(r)是一步预测位置，R_r和R_n是x(r)对R和n在当前状态的雅可比矩阵
    [x(r), R_r, R_n] = move(x(r), u, [0,0]);
    P_rr = P(r,r);
    P(r,:) = R_r*P(r,:);
    P(:,r) = P(r,:)';
    P(r,r) = R_r*P_rr*R_r' + R_n*Q*R_n';
    
    %   b. correction
    %       i. known lmks
    lids = find(l(1,:));
    for lid = lids
        % expectation
        [e, E_r, E_l] = project(x(r), x(l(:,lid)));
        E_rl = [E_r E_l];
        rl   = [r l(:,lid)'];
        E    = E_rl * P(rl,rl) * E_rl';
        
        % measurement
        yi = y(:,lid);
        
        % innovation
        z = yi - e;
        if z(2) > pi
            z(2) = z(2) - 2*pi;
        end
        if z(2) < -pi
            z(2) = z(2) + 2*pi;
        end
        Z = M + E;
        
        % Kalman gain
        K = P(:, rl) * E_rl' * Z^-1;
        
        % update
        x = x + K * z;
        P = P - K * Z * K';
    end
    
    %       ii. init new lmks
    % check lmk availability
    lid = find(l(1,:)==0 , 1);
    % lid是landmark中未被初始化的点
    if ~isempty(lid)
        s = find(mapspace, 2);
        % s是机器人pose
        if ~isempty(s)
            mapspace(s) = 0;
            l(:,lid) = s';
            % measurement
            yi = y(:,lid);
            
            [x(l(:,lid)), L_r, L_y] = backProject(x(r), yi);
            P(s,:) = L_r * P(r,:);
            P(:,s) = P(s,:)';
            P(s,s) = L_r * P(r,r) * L_r' + L_y * M * L_y';
        end
    end
    
    % 3. Graphics
    
    set(RG, 'xdata', R(1), 'ydata', R(2));
    set(rG, 'xdata', x(r(1)), 'ydata', x(r(2)));
    lids = find(l(1,:));
    lx = x(l(1,lids));
    ly = x(l(2,lids));
    set(lG, 'xdata', lx, 'ydata', ly);
    for lid = lids
        le = x(l(:,lid));
        LE = P(l(:,lid),l(:,lid));
        [X,Y] = cov2elli(le,LE,3,16);
        set(eG(lid),'xdata',X,'ydata',Y);
    end
    if t > 1
        re = x(r(1:2));
        RE = P(r(1:2),r(1:2));
        [X,Y] = cov2elli(re,RE,3,16);
        set(reG,'xdata',X,'ydata',Y);
    end
    drawnow;
end









