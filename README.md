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

gazebo环境

## visualization
在gazebo环境中写出来的txt处理成图像的程序与结果

ramdon原本是turtlebot3的随机初始化位置的文件，运行时有问题，暂时存放在这里

词云文件：可以生成词云