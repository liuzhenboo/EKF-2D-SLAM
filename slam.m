%format long 
% I. 初始化
%
%   0. 定义系统.

% System noise
q = [0.01;0.02];
Q = diag(q.^2);
% Measurement noise
m = [.15; 1*pi/180];
M = diag(m.^2);
% randn('seed',1);

%
%   1. 仿真路标点环境
%       R: robot pose u: control
%       W: 设置所有路标点位置
R = [0;-2.5;0];
u = [0.1;0.05];
% 环形摆放的landmarks
jiaodu_perLandMark =6;  %取1,3,6,15,30,60...(360的倍数均可)
r1=3;
r2=2;
r3=3.5;
W = landmarks(r1,r2,r3,jiaodu_perLandMark);
sensor_r = 2;

% Id用来存放观测过的路标点的Id；如果用c++实现，建议使用map结构。
Id = zeros(1,size(W,2));
y_olds = zeros(3,size(W,2));
y_news = zeros(3,size(W,2));

%   2. 估计量初始化
%   状态量及协方差
x = zeros(numel(R)+numel(W), 1);
P = zeros(numel(x),numel(x));

mapspace = zeros(1,size(W,2));
r = [1 2 3];
s = [4 5];
x(r) = R;
%x(r) = [8;-2.5;0];
P(r,r) = 0;

%   3. 画图
mapFig = figure(1);
cla;
axis([-5 5 -5 5])
axis square
% 所有路标点
WG = line('parent',gca,...
    'linestyle','none',...
    'marker','.',...
    'color','m',...
    'xdata',W(1,:),...
    'ydata',W(2,:));
% 仿真下机器人初始位置
RG = line('parent',gca,...
    'marker','+',...
    'color','m',...
    'xdata',R(1),...
    'ydata',R(2));
% 假设的机器人初始位置
rG = line('parent',gca,...
    'linestyle','none',...
    'marker','+',...
    'color','k',...
    'xdata',x(r(1)),...
    'ydata',x(r(2)));
% 估计的路标点位置
lG = line('parent',gca,...
    'linestyle','none',...
    'marker','.',...
    'color','k',...
    'xdata',[],...
    'ydata',[]);

eG1 = zeros(1,size(W,2));
for i = 1:numel(eG1)
    eG1(i) = line(...
        'parent', gca,...
        'color','k',...
        'xdata',[],...
        'ydata',[]);
end

reG = line(...
    'parent', gca,...
    'color','r',...
    'xdata',[],...
    'ydata',[]);

sensor1 = line(...
    'parent', gca,...
    'color','m',...
    'xdata',[],...
    'ydata',[],...
    'LineStyle','--');
sensor2 = line(...
    'parent', gca,...
    'color','m',...
    'xdata',[],...
    'ydata',[],...
    'LineStyle','--');
Sensor1 = line(...
    'parent', gca,...
    'color','m',...
    'xdata',[],...
    'ydata',[],...
     'LineStyle','--');
 Sensor2 = line(...
    'parent', gca,...
    'color','m',...
    'xdata',[],...
    'ydata',[],...
     'LineStyle','--');
% II. 大循环；机器人每前进一步，循环一次

for t = 1:2000
%      if t == 200
%          sensor_r = 0.2;         
%      end
%      if t == 400
%         sensor_r =0.5; 
%      end 
%      if t == 600
%         sensor_r =0.8; 
%      end 
%      if t == 800
%         sensor_r =3; 
%      end 
%      
%      if t == 1000
%          sensor_r = 4;         
%      end
%      if t == 1200
%         sensor_r =1.5; 
%      end 
%      if t == 1400
%         sensor_r =2; 
%      end 
%      if t == 1600
%         sensor_r =3; 
%      end 

    % 1. Simulator
    n = q.*randn(2,1);
    % 下一时刻机器人位置；
    R = move(R, u, n);
    % 传感器获取的信息；i表示路标点的唯一ID标识号；yi表示观测到的特征点在当前坐标系的坐标，若是为零，表示该种路标点没有观测到。
    % 观测到的路标点有两种来源：
    % 1:曾经观测到过。EKF时候只需要根据正向观测方程project对当前状态量进行修正就可以了。
    % 2:之前未曾观测到过。这时候需要将状态向量增广，利用逆观测方程backProject初始化新增状态。
    % y_olds 表示曾经观测到的老路标点集合。
    % y_news 表示新发现的的路标点集合。
    i_olds=1;
    i_news=1;
    %仅仅保留一个探测到的路标temp=1
    temp =1;
    % 传感器的观测范围
    for i = 1:size(W,2)
        v = m.*randn(2,1);
         yi= project(R, W(:,i)) + v;
        %if (yi(2)>0) && (yi(1)>0) && (Id(i)==1)
        %if (Id(i)==1)
        if yi(1) < sensor_r && Id(i) == 1
            %(yi(1)*yi(1)+yi(2)*yi(2) <= sensor_r^2) && (Id(i) ==1 )
               y_olds(:,i_olds) = [yi(1);yi(2);i];
               i_olds = i_olds + 1;
        %(Id(i)==0) &&(yi(2)>0) &&(yi(1)>0) && temp ==1
        elseif  yi(1) < sensor_r &&  Id(i) == 0 %&& temp ==1
            %(yi(1)*yi(1)+yi(2)*yi(2) <= sensor_r^2) && (Id(i) ==0 )
                y_news(:,i_news) = [yi(1);yi(2);i];
                i_news = i_news + 1;
                Id(i) = 1;
                %temp = temp +1;
        end
    end
    
    for i = i_olds:size(W,2)
        y_olds(:,i) = [100;0;0];
    end
    for i = i_news:size(W,2)
        y_news(:,i) = [101;0;0];
    end
  
    % 2. Filter
    %   a. Prediction
    %   CAUTION this is sub-optimal in CPU time
    % [x(r), R_r, R_n] = move(x(r), u, n);
    % x(r)是一步预测位置，R_r和R_n是x(r)对R和n在当前状态的雅可比矩阵
    [x(r), R_r, R_n] = move(x(r), u, [0 0]);
    P_rr = P(r,r);
    P(r,:) = R_r*P(r,:);
    P(:,r) = P(r,:)';
    P(r,r) = R_r*P_rr*R_r' + R_n*Q*R_n';
    
 %     b. correction
  %        i. known lmks
         % 观测范围：当前时刻会观测到所有曾经被观测过的路标点；
          %对多个观测量的处理方式：对观测量逐个处理，每次根据对一个路标点的观测量对状态进行更新
    end_old = find(y_olds(1,:)==100,1);  
    if isempty(end_old)
        end_old=size(y_olds,2)+1;
    end
    
    for j = 1:(end_old-1)
        % expectation
        if isempty(j)
            break
        end
        id = find(mapspace==y_olds(3,j),1);
        v = [id*2+2 id*2+3];
        [e, E_r, E_l] = project(x(r), x(v));
        E_rl = [E_r E_l];
        rl   = [r v];
        E    = E_rl * P(rl,rl) * E_rl';
        
        % measurement
        yi_1 = y_olds(:,j);
        yi1 = yi_1(1:2,1);
        
        % innovation
        z = yi1 - e;
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
    
    %每个大循环会对状态进行增广，增加一个新的路标点状态量；如果等到路标点全部已经初始化，那么初始化部分就不会再执行。
     %     ii. init new lmks
    end_new = find(y_news(1,:)==101,1);
    if isempty(end_new)
        end_new=size(y_news,2)+1;
    end
    for m1 = 1:(end_new-1)
        if isempty(m1)
            break
        end
        id = find(mapspace==0,1);
        mapspace(id) = y_news(3,m1);
        
        % measurement
        yi_2 = y_news(:,m1);
        yi2 = yi_2(1:2,1);
        [x(s), L_r, L_y] = backProject(x(r ), yi2);
        P(s,:) = L_r * P(r,:);
        P(:,s) = P(s,:)';
        P(s,s) = L_r * P(r,r) * L_r' + L_y * M * L_y';
        s = s + [2 2];
    end
    
     % 3. Graphics
    
    set(RG, 'xdata', R(1), 'ydata', R(2));
    circle_x = linspace((R(1)-0.9999*sensor_r),(R(1)+0.9999*sensor_r));
    circle_y1 = sqrt(sensor_r^2 - (circle_x - R(1)).^2) + R(2);
    circle_y2 = R(2) - sqrt(sensor_r^2 - (circle_x - R(1)).^2);
    set(sensor1,'xdata',circle_x,'ydata',circle_y1);
    set(sensor2,'xdata',circle_x,'ydata',circle_y2);
    
    set(rG, 'xdata', x(r(1)), 'ydata', x(r(2)));
    Circle_x = linspace((x(r(1))-0.9999*sensor_r),(x(r(1))+0.9999*sensor_r));
    Circle_y1 = sqrt(sensor_r^2 - (Circle_x - x(r(1))).^2) + x(r(2));
    Circle_y2 = x(r(2)) - sqrt(sensor_r^2 - (Circle_x - x(r(1))).^2);
    %set(Sensor1,'xdata',Circle_x,'ydata',Circle_y1);
    %set(Sensor2,'xdata',Circle_x,'ydata',Circle_y2);    
        
  % 画出估计的路标点位置
  if s(1)==4
        continue
  end
  w = 2:((s(1)-2)/2);
  w = 2*w;
  lx = x(w);
  ly = x(w+1);
  set(lG, 'xdata', lx, 'ydata', ly);
  
  % 当前估计的路标点分为三种：
  % 1：刚刚探索发现的
  % 2：之前遇到过，现在重新遇见的
  % 3：之前遇到过，当前没有遇见
  %[X, Y]=[0 0];
  for i = 1:numel(eG1)
    %set(eG1(i),'xdata',X,'ydata',Y,'color','w');
     set(eG1(i),'color','y');   
  end
  %%%%%第一种：刚刚探索发现的（蓝色）
  for g1 = 1:(end_new-1)
      if isempty(g1)
            break
      end
      o1 = y_news(3,g1);
      h1 = find(mapspace==o1,1);
      %plot1 = ()
      temp1 = [2*h1+2;2*h1+3];
      le = x(temp1);
      LE = P(temp1,temp1);
      [X,Y] = cov2elli(le,LE,3,16);   
      %set(lG,'xdata',le(1),'ydata',le(2));
      set(eG1(o1),'xdata',X,'ydata',Y,'color','b');
  end
  %%%%第二种：之间遇到过，现在重新遇见的
  for g2 = 1:(end_old-1)
      if isempty(g2)
            break
      end
      o2 = y_olds(3,g2);
      h2 = find(mapspace==o2,1);
      temp2 = [2*h2+2;2*h2+3];
      le = x(temp2);
      LE = P(temp2,temp2);
      [X,Y] = cov2elli(le,LE,3,16);  
      set(eG1(o2),'xdata',X,'ydata',Y,'color','r');
  end
  %%%%第三种：之前遇到过，现在没有遇见
  v = find(mapspace==0,1);
  if isempty(v)
      v = size(mapspace,2)+1;
  end
  for g3 = 1:v-1
      if isempty(g3)
            break
      end
      %temp3 = [0;0];
      %a=mapspace(g3);
      a = find(y_olds(3,:)==mapspace(g3),1);
      b = find(y_news(3,:)==mapspace(g3),1);   
      if (isempty (a)) && (isempty(b)) 
         temp3 =  [2*g3+2;2*g3+3];
      
      %h2 = find(mapspace==o2,1);
      %temp2 = [2*h2+2;2*h2+3];
      le = x(temp3);
      LE = P(temp3,temp3);
      [X,Y] = cov2elli(le,LE,3,16);
      set(eG1(mapspace(g3)),'xdata',X,'ydata',Y,'color','k');
      end
  end

     if t > 1
         re = x(r(1:2));
         RE = P(r(1:2),r(1:2));
         [X,Y] = cov2elli(re,RE,3,16);
         set(reG,'xdata',X,'ydata',Y);
     end
   drawnow;
    pause(0.3);
    
end


