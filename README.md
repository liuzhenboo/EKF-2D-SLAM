
## EKF-2D-SLAM

### EKF-SLAM步骤

- 运动更新

运动更新时，固定点坐标不变，所以只需要根据运动方程更新(x,y,α)及其协方差与互协方差即可。

- 观测到曾经观测过的固定点

这时候依次对观测到的特征点信息进行EKF更新
    
    Landmark observations are processed in the EKF usually one-by-one


- 观测到新的固定点

这时候观测到新的固定点，需要进行状态增广。根据逆观测方程，使用观测信息推测出新加的增广状态均值与方差，然后加入到总体的状态与协方差矩阵中。

### Matlab代码
主文件为slam.m，运行即可。

### 结果展示
- 传感器探测范围与路标点

![2](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/2.PNG)

- 第一次状态增广

![1](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/1.PNG)

- 状态持续扩大

![4](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/4.PNG)


- 状态增广已停止

![5](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/5.PNG)

![6](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/6.PNG)

---------
## 改动
2020/2/21增加了轨迹显示

![7](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/7.PNG)

![8](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/8.PNG)

![9](https://github.com/liuzhenboo/EKF-2D-SLAM/raw/master/videos/9.PNG)


