
## 应用EKF实现3D-SLAM

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


