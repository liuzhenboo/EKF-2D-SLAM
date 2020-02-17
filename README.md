# NPU最优估计课程学习
姓名：刘振博

学号：2019201920

[github](https://github.com/liuzhenboo/EKF-2D-SLAM)
---

## 完成工作
- 一维状态量的KF仿真
- 二维状态量的EKF仿真
- 应用EKF实现2D-SLAM
## 一维状态量的KF仿真
系统建模：

    x+ = F_x *x + F_u * u + F_n * n
    y  = H * x + v
    其中：
    F_x = 1;
    F_u = 1;
    F_n = 1;
    u = 1；
    H = 0.5;
    Q = 1;
    R = 1;   

状态先验：

    x = 0;
    P = 1e4;

仿真初值：

    X = 7;

仿真结果：

![1](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/1.jpg)

## 二维状态量的EKF仿真

系统模型：
    
    x+ = f ( x, u, n )
    y  = h ( x ) + v

系统定义：
    
    x = [px py vx vy]'
    y = [d, a]'    
    u = [ax, ay]'
    n = [nx, ny]'
    v = [vd, va]'

    px+ = px + vx*dt
    py+ = py + vy*dt
    vx+ = vx + ax*dt + nx
    vy+ = vy + ay*dt + ny

    d = sqrt(px^2 + py^2) + vd
    a = atan2(py, px) + va

    Q = diag([.1 0.1].^2)
    R = diag([.1 1*pi/180].^2)

状态先验：

    x = [1 1 0 0]'
    P = diag([1 1 1 1].^2)

仿真初值：

    X = [2 1 -1 1]'


仿真结果：

![4](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/4.jpg)

## 应用EKF实现2D-SLAM
### 问题定义
某移动机器人按照给定的运动方程在一个环境中运动，并且环境中有n个固定点；这个机器人身上装有某种传感器，一定范围内该种传感器能够量测到某些固定点到自己的距离与角度，现在要根据运动方程和量测信息估算机器人的位置和n个固定点的位置。

![2](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/2.jpg)

### EKF-SLAM步骤

状态变量X为当前机器人的位置(x,y,α,...Mi...),Mi为截至到当前观测过的固定点坐标。

- 运动更新

运动更新时，固定点坐标不变，所以只需要根据运动方程更新(x,y,α)及其协方差与互协方差即可。

- 观测到曾经观测过的固定点

这时候依次对观测到的特征点信息进行EKF更新
    
    Landmark observations are processed in the EKF usually one-by-one


- 观测到新的固定点

这时候观测到新的固定点，需要进行状态增广。根据逆观测方程，使用观测信息推测出新加的增广状态均值与方差，然后加入到总体的状态与协方差矩阵中。

仿真结果：

![3](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/3.jpg)


