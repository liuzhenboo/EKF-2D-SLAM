%  https://github.com/liuzhenboo/EKF-2D-SLAM
% MIT License
% 
% Copyright (c) 2020 liuzhenboo
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% format long 
% I. 初始化
%
disp('EKF-2D-SLAM sample program start!!')
% 运动噪声
q = [0.01;0.01];
Q = diag(q.^2);
% 测量噪声
m = [.15; 1*pi/180];
M = diag(m.^2);

% R: 机器人初始位置
% u: 控制量
R = [0;-2.2;0];
u = [0.1;0.05];

% 设置外界路标点环境
% 环形摆放的landmarks
% W: 设置所有路标点位置
jiaodu_perLandMark =6;  %取1,3,6,15,30,60...(360的倍数均可)
r1=2;
r2=3;
r3=3.5;
W = landmarks(r1,r2,r3,jiaodu_perLandMark);

% 传感器探测半径
sensor_r = 2.5;

% Id容器用来判别当前探测到的路标点曾经是否被观测过；若没有观测过，那么此时需要将其加入Id容器。
% 这里使用W中每个点的索引作为路标点的id；Id初始化为一个足够大的零数组即可。
% Id(类型)==1，表示曾经观测过；Id(类型)==0，表示曾经没有观测过。
% 如果用c++实现，建议使用map结构。
Id = zeros(1,size(W,2));

% y_news表示当前新探测到的路标点，y_news(:,i)记录观测量和路标点类型
% 同理y_olds
y_olds = zeros(3,size(W,2));
y_news = zeros(3,size(W,2));

%   状态量及协方差初始化
x = zeros(numel(R)+numel(W), 1);
P = zeros(numel(x),numel(x));

% id_to_x_map：id------>>>id对应的状态变量在x中的位置
id_to_x_map = zeros(1,size(W,2));

% x和P初始化
r = [1 2 3];
x(r) = R;
%x(r) = [8;-2.5;0];
P(r,r) = 0;

% 每次状态增广在x中的位置
s = [4 5];

%主循环次数
% 125/每圈
 loop =250;
 
% 存放位姿仿真量
poses_ = zeros(3,loop);

% 存放位姿历史估计量
poses = zeros(3,loop);

 %  绘图
mapFig = figure(1);
cla;
axis([-5 5 -5 5])
axis square
%axis equal
% 所有路标点
WG = line('parent',gca,...
    'linestyle','none',...
    'marker','.',...
    'color','m',...
    'xdata',W(1,:),...
    'ydata',W(2,:));
% 仿真下机器人位置
RG = line('parent',gca,...
    'marker','+',...
    'color','r',...
    'xdata',R(1),...
    'ydata',R(2));
% 估计的机器人位置
rG = line('parent',gca,...
    'linestyle','none',...
    'marker','+',...
    'color','b',...
    'xdata',x(r(1)),...
    'ydata',x(r(2)));
% 估计的路标点位置
lG = line('parent',gca,...
    'linestyle','none',...
    'marker','+',...
    'color','k',...
    'xdata',[],...
    'ydata',[]);

% 估计的路标点协方差
eG1 = zeros(1,size(W,2));
for i = 1:numel(eG1)
    eG1(i) = line(...
        'parent', gca,...
        'color','k',...
        'xdata',[],...
        'ydata',[]);
end

% 估计的机器人位置
reG = line(...
    'parent', gca,...
    'color','r',...
    'xdata',[],...
    'ydata',[]);

% 传感器探测范围（以真实位置为圆心）
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

%传感器探测范围（以估计位置为圆心）
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

 true_pose = line(...
    'parent', gca,...
    'color','r',...
    'xdata',[],...
    'ydata',[],...
    'LineWidth',0.8);
     %'LineStyle','--');
 
 estimate_pose = line(...
    'parent', gca,...
    'color','b',...
    'xdata',[],...
    'ydata',[],...
    'LineWidth',0.8);
    % 'LineStyle','--');
 
 % II. 主循环；
 % 机器人每前进一步，循环一次
for t = 1:loop
%     if t == 125
%         u(1) = 0.2;
%         sensor_r = 4;
%     end
%     if t == 375
%         u(1) = 0.2;
%         sensor_r = 5;
%     end
    %不同探测半径
%      if t == 200
%           sensor_r = 1;         
%      end
%      if t == 400
%          sensor_r =1.5; 
%      end 
%      if t == 600
%         sensor_r =2; 
%      end 
%      if t == 800
%         sensor_r =2.5; 
%      end 
%      if t == 1000
%          sensor_r = 3;         
%      end

    % 1. 观测仿真
    n = q.*randn(2,1);
    % 下一时刻机器人真实位置；
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
    %temp =1;
    for i = 1:size(W,2)
        v = m.*randn(2,1);
         yi= project(R, W(:,i)) + v;
        if yi(1) < sensor_r && Id(i) == 1
               y_olds(:,i_olds) = [yi(1);yi(2);i];
               i_olds = i_olds + 1;
        elseif  yi(1) < sensor_r &&  Id(i) == 0 %&& temp ==1
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
  
    % 2. EKF滤波
    %   a. 预测
    % x(r)是一步预测位置，R_r和R_n是x(r)对R和n在当前状态的雅可比矩阵
    [x(r), R_r, R_n] = move(x(r), u, [0 0]);
    P_rr = P(r,r);
    P(r,:) = R_r*P(r,:);
    P(:,r) = P(r,:)';
    P(r,r) = R_r*P_rr*R_r' + R_n*Q*R_n';
    
 %     b. 修正
 % 对多个观测量的处理方式：对观测量逐个处理，每次根据对一个路标点的观测量对状态进行更新
    end_old = find(y_olds(1,:)==100,1);  
    if isempty(end_old)
        end_old=size(y_olds,2)+1;
    end
    
    for j = 1:(end_old-1)
        % expectation
        if isempty(j)
            break
        end
        id = find(id_to_x_map==y_olds(3,j),1);
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
    
     % 3. 状态增广
    % 每个大循环会对状态进行增广，增加一个新的路标点状态量；如果等到路标点全部已经初始化，那么初始化部分就不会再执行。   
    end_new = find(y_news(1,:)==101,1);
    if isempty(end_new)
        end_new=size(y_news,2)+1;
    end
    for m1 = 1:(end_new-1)
        if isempty(m1)
            break
        end
        id = find(id_to_x_map==0,1);
        id_to_x_map(id) = y_news(3,m1);
        
        % measurement
        yi_2 = y_news(:,m1);
        yi2 = yi_2(1:2,1);
        [x(s), L_r, L_y] = backProject(x(r ), yi2);
        P(s,:) = L_r * P(r,:);
        P(:,s) = P(s,:)';
        P(s,s) = L_r * P(r,r) * L_r' + L_y * M * L_y';
        s = s + [2 2];
    end
    
     % 4. 获取想要的信息
    % 获取poses信息
    poses(1,t) = x(1);
    poses(2,t) = x(2);
    poses(3,t) = x(3);   
    poses_(1,t) = R(1);
    poses_(2,t) = R(2);
    poses_(3,t) = R(3);
    % ...
    
     % 5. 绘图展示

     % 机器人仿真位置与传感器探测范围 
    set(RG, 'xdata', R(1), 'ydata', R(2));
    circle_x = linspace((R(1)-0.9999*sensor_r),(R(1)+0.9999*sensor_r));
    circle_y1 = sqrt(sensor_r^2 - (circle_x - R(1)).^2) + R(2);
    circle_y2 = R(2) - sqrt(sensor_r^2 - (circle_x - R(1)).^2);
    set(sensor1,'xdata',circle_x,'ydata',circle_y1);
    set(sensor2,'xdata',circle_x,'ydata',circle_y2);
    
    % 探测范围（估计位置为圆心）
    set(rG, 'xdata', x(r(1)), 'ydata', x(r(2)));
    Circle_x = linspace((x(r(1))-0.9999*sensor_r),(x(r(1))+0.9999*sensor_r));
    Circle_y1 = sqrt(sensor_r^2 - (Circle_x - x(r(1))).^2) + x(r(2));
    Circle_y2 = x(r(2)) - sqrt(sensor_r^2 - (Circle_x - x(r(1))).^2);
    %set(Sensor1,'xdata',Circle_x,'ydata',Circle_y1);
    %set(Sensor2,'xdata',Circle_x,'ydata',Circle_y2);    
    
    % 位置定位轨迹
    set(estimate_pose,'xdata',poses(1,1:t),'ydata',poses(2,1:t));
    set(true_pose,'xdata',poses_(1,1:t),'ydata',poses_(2,1:t));
    
    legend([estimate_pose true_pose lG WG],{'Estimate','Truth' 'Estimate landmark' 'True landmark'})
  % 如果第一次没有状态增广，即刻返回进行下一次循环
  if s(1)==4
        continue
  end
  
  % 估计的路标点位置
  w = 2:((s(1)-2)/2);
  w = 2*w;
  lx = x(w);
  ly = x(w+1);
  set(lG, 'xdata', lx, 'ydata', ly);
  
  % 画出估计路标点协方差椭圆
  % 估计的路标点分为三种：
  % 1：刚刚探索发现的
  % 2：之前遇到过，现在重新遇见的
  % 3：之前遇到过，当前没有遇见
  
% 先将所有路标点的协方差椭圆都赋值黑色
%   for i = 1:numel(eG1)
%      set(eG1(i),'color','k');   
%   end
  
%%%%%第一种：刚刚探索发现的（蓝色）
  for g1 = 1:(end_new-1)
      if isempty(g1)
            break
      end
      o1 = y_news(3,g1);
      h1 = find(id_to_x_map==o1,1);
      temp1 = [2*h1+2;2*h1+3];
      le = x(temp1);
      LE = P(temp1,temp1);
      [X,Y] = cov2elli(le,LE,3,16);   
      set(eG1(o1),'xdata',X,'ydata',Y,'color','b');
  end
  %%%%第二种：之间遇到过，现在重新遇见的（红色）
  for g2 = 1:(end_old-1)
      if isempty(g2)
            break
      end
      o2 = y_olds(3,g2);
      h2 = find(id_to_x_map==o2,1);
      temp2 = [2*h2+2;2*h2+3];
      le = x(temp2);
      LE = P(temp2,temp2);
      [X,Y] = cov2elli(le,LE,3,16);  
      set(eG1(o2),'xdata',X,'ydata',Y,'color','r');
  end
  %%%%第三种：之前遇到过，现在没有遇见（黑色）
  v = find(id_to_x_map==0,1);
  if isempty(v)
      v = size(id_to_x_map,2)+1;
  end
  for g3 = 1:v-1
      if isempty(g3)
            break
      end
      a = find(y_olds(3,:)==id_to_x_map(g3),1);
      b = find(y_news(3,:)==id_to_x_map(g3),1);   
      if (isempty (a)) && (isempty(b)) 
         temp3 =  [2*g3+2;2*g3+3];
            le = x(temp3);
      LE = P(temp3,temp3);
      [X,Y] = cov2elli(le,LE,3,16);
      set(eG1(id_to_x_map(g3)),'xdata',X,'ydata',Y,'color','k');
      end
  end

% 估计的机器人位置协方差椭圆（红色）
     if t > 1
         re = x(r(1:2));
         RE = P(r(1:2),r(1:2));
         [X,Y] = cov2elli(re,RE,3,16);
         set(reG,'xdata',X,'ydata',Y);
     end
   
   drawnow;
   
   pause(0.1);
    
end


