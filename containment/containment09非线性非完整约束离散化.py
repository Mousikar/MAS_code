import numpy as np
import random
import matplotlib.pyplot as plt
import math

# 随机种子
random.seed(12345)

# 迭代设置
iter = 1000
T = 0.001
num_follower = 6
num_leader = 4

# 初值
x = [0 + 4 * random.random(),
     0 + 4 * random.random(),
     0 + 4 * random.random(),
     0 + 4 * random.random(),
     0 + 4 * random.random(),
     0 + 4 * random.random()]
y = [6 + 4 * random.random(),
     6 + 4 * random.random(),
     6 + 4 * random.random(),
     6 + 4 * random.random(),
     6 + 4 * random.random(),
     6 + 4 * random.random()]
theta = [-3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random()]
hat_theta_d = [0,0,0,0,0,0]
dot_hat_theta_d = [0,0,0,0,0,0]
rx = [6 + 4 * random.random(),
     6 + 4 * random.random(),
     6 + 4 * random.random(),
     6 + 4 * random.random()]
ry = [0 + 4 * random.random(),
     0 + 4 * random.random(),
     0 + 4 * random.random(),
     0 + 4 * random.random()]
dot_rx = [0,0,0,0]
dot_ry = [0,0,0,0]

# # --------------------------------------------------------------------------------------------
# print(x, '\n', y, '\n', theta)

# fig, ax  = plt.subplots(figsize=(8, 8))     # 可视化初值
# ax.set_xlim(-1, 10)
# ax.set_ylim(-1, 10)
# plt.grid()
# plt.scatter(x, y, color='blue')      # 绘制点
# plt.scatter(rx, ry, color='red')      # 绘制点
# for i in range(len(x)):             # 绘制箭头
#     dx = 0.5 * np.cos(theta[i])     # 计算箭头的x方向分量
#     dy = 0.5 * np.sin(theta[i])     # 计算箭头的y方向分量
#     plt.arrow(x[i], y[i], dx, dy, head_width=0.1, head_length=0.2, fc='black', ec='black')
# plt.title('turtlebot position and pose')    # 设置图形标题和坐标轴标签
# plt.xlabel('X/(m)')
# plt.ylabel('Y/(m)')
# plt.show()    # 显示图形
# # --------------------------------------------------------------------------------------------

# 系数
k1 = 0.2
k2 = 0.4
k3 = 0.01
R = 100

# 网络拓扑
A_F = np.array([[0,1,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,1,0,0,0,0],
                [1,0,0,0,0,1],
                [0,0,1,0,1,0]])   # 跟随者邻接矩阵
B = np.array([[1,0,0,0,0,0],
              [0,2,0,0,0,0],
              [0,0,1,0,0,0],
              [0,0,0,1,0,0],
              [0,0,0,0,1,0],
              [0,0,0,0,0,0]])   # 跟随者能接受到的领导者信息总和
A_LF = np.array([[1,0,0,0],
                 [0,1,1,0],
                 [0,0,0,1],
                 [1,0,0,0],
                 [0,1,0,0],
                 [0,0,0,0]])    # 领导者和跟随者的耦合邻接矩阵

# 先写没有时延的版本

# 初始化变量
ux = [0,0,0,0,0,0]
uy = [0,0,0,0,0,0]
theta_d = [0,0,0,0,0,0]
v = [0,0,0,0,0,0]
omega = [0,0,0,0,0,0]
ddot_hat_theta_d = [0,0,0,0,0,0]

# 历史变量
x_history = []
y_history = []
theta_history = []

for k in range(iter):
    hat_ex = [0,0,0,0,0,0]
    hat_ey = [0,0,0,0,0,0]
    dot_dx = [0,0,0,0,0,0]
    dot_dy = [0,0,0,0,0,0]
    # 控制方程
    for i in range(num_follower):
        for j in range(num_follower):
            hat_ex[i] = hat_ex[i] - A_F[i][j] * (x[i] - x[j])
            hat_ey[i] = hat_ey[i] - A_F[i][j] * (y[i] - y[j])
        for j in range(num_leader):
            hat_ex[i] = hat_ex[i] - A_LF[i][j] * (x[i] - rx[j])
            hat_ey[i] = hat_ey[i] - A_LF[i][j] * (y[i] - ry[j])
            dot_dx[i] = dot_dx[i] + A_LF[i][j] * dot_rx[j]
            dot_dy[i] = dot_dy[i] + A_LF[i][j] * dot_ry[j]
        ux[i] = dot_dx[i] + k1 * hat_ex[i]
        uy[i] = dot_dy[i] + k2 * hat_ey[i]
        theta_d[i] = math.atan2(uy[i], ux[i])
        # -------------------------------保证theta_d在0和2 * pi之间---------------------------------------
        if i == 0:
            theta_d1 = theta_d[i]
            kk = 0
        delta = - 0.8 * np.pi ** 2
        if theta_d[i] * theta_d1 < delta:
            if theta_d[i] < 0:
                kk = kk + 1
            else:
                kk = kk - 1
        theta_d1 = theta_d[i]
        theta_d[i] = theta_d[i] + 2 * np.pi * kk
        # -------------------------------保证theta_d在0和2 * pi之间---------------------------------------

        v[i] = math.sqrt(uy[i]**2 + ux[i]**2)

        ddot_hat_theta_d[i] = - R**2 * (hat_theta_d[i] - theta_d[i]) - 2 * R * dot_hat_theta_d[i]        # 线性二阶微分器
        dot_hat_theta_d[i] = dot_hat_theta_d[i] + ddot_hat_theta_d[i] * T
        hat_theta_d[i] = hat_theta_d[i] + dot_hat_theta_d[i] * T
        
        omega[i] = dot_hat_theta_d[i] + k3 * (theta_d[i] - theta[i])      # 暂时不加上饱和函数

    # 系统方程
    for i in range(num_follower):
        x[i] = x[i] + v[i] * T * np.cos(theta[i])    # v和x是N个智能体的速度和x坐标
        y[i] = y[i] + v[i] * T * np.sin(theta[i])    # y是N个智能体的y坐标
        theta[i] = theta[i] + omega[i] * T
    print(x)
    # print(x, '\n', y, '\n', theta)
    x_history.append([x[0],x[1],x[2],x[3],x[4],x[5]])
    y_history.append([y[0],y[1],y[2],y[3],y[4],y[5]])
    theta_history.append([theta[0],theta[1],theta[2],theta[3],theta[4],theta[5]])

x_history = np.array(x_history)
y_history = np.array(y_history)
theta_history = np.array(theta_history)
# --------------------------------------------------------------------------------------------
fig, ax  = plt.subplots(figsize=(8, 8))     # 可视化初值
# ax.set_xlim(-1, 10)
# ax.set_ylim(-1, 10)
plt.grid()
plt.scatter(x, y, color='blue')      # 绘制点
plt.scatter(rx, ry, color='red')      # 绘制点
for i in range(len(x)):             # 绘制箭头
    dx = 0.5 * np.cos(theta[i])     # 计算箭头的x方向分量
    dy = 0.5 * np.sin(theta[i])     # 计算箭头的y方向分量
    plt.arrow(x[i], y[i], dx, dy, head_width=0.1, head_length=0.2, fc='blue', ec='black')
plt.plot(x_history, y_history, lw=2)
# 绘制轨迹和速度向量
for k in range(num_follower):
    plt.plot(x_history[:,k], y_history[:,k], lw=2)
plt.title('turtlebot position and pose')    # 设置图形标题和坐标轴标签
plt.xlabel('X/(m)')
plt.ylabel('Y/(m)')
plt.show()    # 显示图形
# --------------------------------------------------------------------------------------------
