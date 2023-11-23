# MAS_code
some code about Multi-Agent Systems

仓库中的每个文件夹是一个 工作空间 或者是 Windows的文件夹

### 常用指令
```
rostopic pub -r 10 /robot_1/cmd_vel geometry_msgs/Twist "linear:  x: -0.50
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
```
rostopic pub -r 10 /robot_1/cmd_vel geometry_msgs/Twist '{linear: {x: -0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.0}}'
```
```
rostopic pub -r 10 /robot_2/cmd_vel geometry_msgs/Twist '{linear: {x: -0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.0}}'
```
```
rostopic pub -r 10 /robot_3/cmd_vel geometry_msgs/Twist '{linear: {x: -0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.0}}'
```

## containment
合围数值仿真文件
- 领导者固定不动
- 领导者移动
- 时滞
- 使用微分方程与不适用微分方程
- 生成拓扑图
- 非线性非完整约束系统的离散化

## demo05_ws
自己写的独轮车文件
- rviz
- gazebo

## event-trigger
事件触发器测试学习程序

## Nonholonomic_Wheeled_Mobile_Robots
simulink仿真文件

## turtlebot3
合围仿真文件

### 运行说明
进入/MAS_code/turtlebot3文件夹

先source一下：

```
source devel/setup.bash
```
然后开一个gazebo环境，例如：
```
roslaunch gazebo_swarm_robot_tb3 comtainment_test.launch 
```
也可以是其他的：

- one.launch：1个差速轮小车
- two.launch：2个差速轮小车
- three.launch：3个差速轮小车
- four.launch：4个差速轮小车 (哈哈哈,懒得一个一个修改，就弄了这么多launch)
- gazebo_swarm_robot_5.launch：5个差速轮小车
- gazebo_swarm_robot_6.launch：6个差速轮小车
- gazebo_swarm_robot_8.launch：8个差速轮小车
- comtainment_test.launch：位置稍微摆了一下，让小车相互之间不发生碰撞

最后rosrun gazebo_swarm_robot_tb3 + 下面的文件就行

### 各个文件说明
**头文件：**
- swarm_robot_control_h.py：没写完的头文件
- swarm_robot_control_new.py：可以用的头文件

**测试文件：**
- test.py：测试写的头文件
- test_new.py：测试写的头文件，测试机器人移动
- main_test.py：角度一致性文件

**leader静止：**
- init.py：随机初始化小车位置
- leader_follower.py：领航者-跟随者协调测试
- containment_l.py：采用拉普拉斯矩阵计算
- containment_l_save_data.py：采用拉普拉斯矩阵计算，并保存位置速度信息
- containment_test.py：测试文件，**没有使用拉普拉斯矩阵**
- containment_test_save_data.py：测试文件，**没有使用拉普拉斯矩阵**,并保存位置,速度信息,比上一个文件更好一点
- 1to0.py：查询相对坐标

**leader移动：**
- 01leadermove.py：

gazebo环境

## visualization
在gazebo环境中写出来的txt处理成图像的程序与结果

ramdon原本是turtlebot3的随机初始化位置的文件，运行时有问题，暂时存放在这里

词云文件：可以生成词云
