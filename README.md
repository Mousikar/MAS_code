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
- ten.launch：有六个跟随者和四个领航者，论文中的拓扑
- ten_copy.launch：和上面的数字不一样，用的拓扑也不一样，这个是好看的那个拓扑
- ten_env.launch：加上环境
- ten_evn_waffle：leader变成华夫饼
- ten_waffle：静止的在运行一次，没有环境


            r_star_his = np.array([[-7+1, -2-0.5], 
                                    [-4+1, -2-0.5], 
                                    [-4+1, 1-0.5],
                                    [-7+1, 1-0.5]])
            if points[9][0]>-7.0:
                r_star_his = np.array([[-4+0.75, -2], 
                                        [-2+0.5, -2], 
                                        [-2+0.5, 0-0.25],
                                        [-4+0.75, 0-0.25]])
            if points[9][0]>-4.0:
                r_star_his = np.array([[-1.5+0.5, -1+0.2], 
                                        [0+0.4, -1+0.2], 
                                        [0+0.4, 0.5+0.1],
                                        [-1.5+0.5, 0.5+0.1]])
            if points[9][0]>-1.50:
                r_star_his = np.array([[-1+0.5, -1], 
                                        [1+2, -1], 
                                        [1+2, 1+0.5],
                                        [-1+0.5, 1+0.5]])
            if points[9][0]>-1.00:
                r_star_his = np.array([[4, -1], 
                                        [6, -1], 
                                        [6, 1],
                                        [4, 1]])

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
- 01leadermove.py：只有leaders移动
- 02followermove.py：跟随者也移动
- 03evn：加入了环境
- 04evnleaderstatic：重新运行leader静止

gazebo环境

常用数字：

> x
[0.794977753962915, 0.3790231396676309, 1.8048786399510626, 1.251074731564699, 1.0533437545365198, 1.3934534044238047]

> y
[1.7049624374714418, 1.372056327223472, 0.27101344204100175, 1.983481168032575, 0.24473791850076831, 0.2812299222008847]

> theta
[0.04852951618804386, 0.11485607989602155, 0.1875520697388747, 0.03905712996081047, 0.045874120545113285, 0.1300420218035415]

[-0.643769852556447, -1.949867341443639, 2.5273189294463365, 0.7883746571131551, 0.16749938924467234, 1.2354436898907468]

> rx
[2.7226665517276967, 1.1206481479519168, 2.8050626657514415, 1.4949144042014761]

> ry
[2.7580415785153365, 1.438738110314806, 2.8396083267771886, 2.975531939197893]

> 稳定时候的位置

2.0 3.5 4.0 3.5 4.25 4.125

3.5 2.75 3.75 3.5 3.875 3.8125

## visualization
在gazebo环境中写出来的txt处理成图像的程序与结果

ramdon原本是turtlebot3的随机初始化位置的文件，运行时有问题，暂时存放在这里

词云文件：可以生成词云
