import numpy as np
import random
import matplotlib.pyplot as plt
import math

from scipy.spatial import ConvexHull
import matplotlib.animation as animation
from shapely.geometry import Point, Polygon
from tqdm import tqdm

# 随机种子
random.seed(1234)

# 迭代设置
iter = 2000
T = 0.008
num_follower = 6
num_leader = 4

# 初值
x = [1 + 4 * random.random(),
     1 + 4 * random.random(),
     1 + 4 * random.random(),
     1 + 4 * random.random(),
     1 + 4 * random.random(),
     1 + 4 * random.random()]
y = [5 + 4 * random.random(),
     5 + 4 * random.random(),
     5 + 4 * random.random(),
     5 + 4 * random.random(),
     5 + 4 * random.random(),
     5 + 4 * random.random()]
theta = [-3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random()]
hat_theta_d = [0,0,0,0,0,0]
dot_hat_theta_d = [0,0,0,0,0,0]
rx = [1 + 4 * random.random(),
     1 + 4 * random.random(),
     1 + 4 * random.random(),
     1 + 4 * random.random()]
ry = [1 + 4 * random.random(),
     1 + 4 * random.random(),
     1 + 4 * random.random(),
     1 + 4 * random.random()]
dot_rx = [0,0,0,0]
dot_ry = [0,0,0,0]

# # --------------------------------------------------------------------------------------------
# print(x, '\n', y, '\n', theta)
fig, ax  = plt.subplots(figsize=(8, 8))     # 可视化初值
ax.set_xlim(-1, 10)
ax.set_ylim(-1, 10)
plt.grid()
plt.scatter(x, y, color='blue')      # 绘制点
plt.scatter(rx, ry, color='red')      # 绘制点
for i in range(len(x)):             # 绘制箭头
    dx = 0.5 * np.cos(theta[i])     # 计算箭头的x方向分量
    dy = 0.5 * np.sin(theta[i])     # 计算箭头的y方向分量
    plt.arrow(x[i], y[i], dx, dy, head_width=0.1, head_length=0.2, fc='black', ec='black')
# plt.title('turtlebot position and pose')    # 设置图形标题和坐标轴标签
# plt.xlabel('X/(m)')
# plt.ylabel('Y/(m)')
# plt.show()    # 显示图形
# # --------------------------------------------------------------------------------------------

# 系数
k1 = 0.4
k2 = 0.4
k3 = 0.5
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
L = - A_F
for i in range(num_follower):
    L[i,i] = np.sum(A_F[i,:])
L1 = L + B
L2 = - A_LF
xishu = np.dot(np.linalg.inv(L1), L2)

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
v_history = []
omega_history = []
theta_d1 = [0,0,0,0,0,0]
kk = [0,0,0,0,0,0]
hat_ex_history = []
hat_ey_history = []
errx_actual_history = []
erry_actual_history = []

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
        if k == 0:
            theta_d1[i] = theta_d[i]
            kk[i] = 0
        delta = - 0.7 * np.pi ** 2
        if theta_d[i] * theta_d1[i] < delta:
            if theta_d[i] < 0:
                kk[i] = kk[i] + 1
            else:
                kk[i] = kk[i] - 1
        theta_d1[i] = theta_d[i]
        theta_d[i] = theta_d[i] + 2 * np.pi * kk[i]
        # -------------------------------保证theta_d在0和2 * pi之间---------------------------------------

        v[i] = math.sqrt(uy[i]**2 + ux[i]**2)

        ddot_hat_theta_d[i] = - R**2 * (hat_theta_d[i] - theta_d[i]) - 2 * R * dot_hat_theta_d[i]        # 线性二阶微分器
        dot_hat_theta_d[i] = dot_hat_theta_d[i] + ddot_hat_theta_d[i] * T
        hat_theta_d[i] = hat_theta_d[i] + dot_hat_theta_d[i] * T
        
        # omega[i] = dot_hat_theta_d[i] + k3 * (theta_d[i] - theta[i])      # 暂时不加上饱和函数
        # --------------------------------加上饱和函数-----------------------------------------
        de = 0.1
        kkk = 1/de
        xites = 0.95
        if np.abs(theta_d[i] - theta[i])>de:
            sats = np.sign(theta_d[i] - theta[i])
        else:
            sats = kkk * (theta_d[i] - theta[i])
        omega[i] = dot_hat_theta_d[i] + k3 * (theta_d[i] - theta[i]) + xites * sats      # 加上饱和函数
        # --------------------------------加上饱和函数-----------------------------------------


    # 系统方程
    for i in range(num_follower):
        x[i] = x[i] + v[i] * T * np.cos(theta[i])    # v和x是N个智能体的速度和x坐标
        y[i] = y[i] + v[i] * T * np.sin(theta[i])    # y是N个智能体的y坐标
        theta[i] = theta[i] + omega[i] * T
        
    errx_actual = x + np.dot(xishu, rx)
    erry_actual = y + np.dot(xishu, ry)
    # print(x)
    # print(x, '\n', y, '\n', theta)
    x_history.append([x[0],x[1],x[2],x[3],x[4],x[5]])
    y_history.append([y[0],y[1],y[2],y[3],y[4],y[5]])
    theta_history.append([theta[0],theta[1],theta[2],theta[3],theta[4],theta[5]])
    v_history.append([v[0],v[1],v[2],v[3],v[4],v[5]])
    omega_history.append([omega[0],omega[1],omega[2],omega[3],omega[4],omega[5]])
    hat_ex_history.append([-hat_ex[0],-hat_ex[1],-hat_ex[2],-hat_ex[3],-hat_ex[4],-hat_ex[5]])
    hat_ey_history.append([-hat_ey[0],-hat_ey[1],-hat_ey[2],-hat_ey[3],-hat_ey[4],-hat_ey[5]])
    errx_actual_history.append([errx_actual[0],errx_actual[1],errx_actual[2],errx_actual[3],errx_actual[4],errx_actual[5]])
    erry_actual_history.append([erry_actual[0],erry_actual[1],erry_actual[2],erry_actual[3],erry_actual[4],erry_actual[5]])

x_history = np.array(x_history)
y_history = np.array(y_history)
theta_history = np.array(theta_history)
v_history = np.array(v_history)
omega_history = np.array(omega_history)
hat_ex_history = np.array(hat_ex_history)
hat_ey_history = np.array(hat_ey_history)
errx_actual_history = np.array(errx_actual_history)
erry_actual_history = np.array(erry_actual_history)
# --------------------------------------------------------------------------------------------
plt.scatter(x, y, color='blue')      # 绘制点
plt.scatter(rx, ry, color='red')      # 绘制点
for i in range(len(x)):             # 绘制箭头
    dx = 0.5 * np.cos(theta[i])     # 计算箭头的x方向分量
    dy = 0.5 * np.sin(theta[i])     # 计算箭头的y方向分量
    plt.arrow(x[i], y[i], dx, dy, head_width=0.1, head_length=0.2, fc='blue', ec='black')
plt.plot(x_history, y_history, lw=2)
plt.title('turtlebot position and pose')    # 设置图形标题和坐标轴标签
plt.xlabel('X/(m)')
plt.ylabel('Y/(m)')
plt.show()    # 显示图形
# --------------------------------------------------------------------------------------------
plt.subplots(figsize=(16, 12))
plt.subplot(2, 4, 1)
plt.grid()
for k in range(num_follower):   # 绘制轨迹和速度向量
    plt.plot(x_history[:,k], lw=2)
    plt.plot(y_history[:,k], lw=2)
plt.title('turtlebot position')    # 设置图形标题和坐标轴标签
plt.xlabel('t/(ms)')
plt.ylabel('X/(m)')
# --------------------------------------------------------------------------------------------
plt.subplot(2, 4, 5)
plt.grid()
for k in range(num_follower):   # 绘制轨迹和速度向量
    plt.plot(theta_history[:,k], lw=2)
plt.title('turtlebot pose')    # 设置图形标题和坐标轴标签
plt.xlabel('t/(ms)')
plt.ylabel('theta/(rad)')
# --------------------------------------------------------------------------------------------
plt.subplot(2, 4, 2)
plt.grid()
plt.plot(v_history, lw=2)
plt.title('turtlebot velocity')    # 设置图形标题和坐标轴标签
plt.xlabel('t/(ms)')
plt.ylabel('v/(m/s)')
# --------------------------------------------------------------------------------------------
plt.subplot(2, 4, 6)
plt.grid()
plt.plot(omega_history, lw=2)
plt.title('turtlebot angular')    # 设置图形标题和坐标轴标签
plt.xlabel('t/(ms)')
plt.ylabel('omega/(rad/s)')
# --------------------------------------------------------------------------------------------
plt.subplot(2, 4, 3)
plt.grid()
for i in range(num_follower):
    plt.plot(hat_ex_history[:,i], lw=2, label=f"follower{i}")
plt.legend()# 添加图例
plt.title('turtlebot x error')    # 设置图形标题和坐标轴标签
plt.xlabel('t/(ms)')
plt.ylabel('X/(m)')
# --------------------------------------------------------------------------------------------
plt.subplot(2, 4, 7)
plt.grid()
for i in range(num_follower):
    plt.plot(hat_ey_history[:,i], lw=2, label=f"follower{i}")
plt.legend()# 添加图例
plt.title('turtlebot y error')    # 设置图形标题和坐标轴标签
plt.xlabel('t/(ms)')
plt.ylabel('X/(m)')
# --------------------------------------------------------------------------------------------
plt.subplot(2, 4, 4)
plt.grid()
for i in range(num_follower):
    plt.plot(errx_actual_history[:,i], lw=2, label=f"follower{i}")
plt.legend()# 添加图例
plt.title('turtlebot x error')    # 设置图形标题和坐标轴标签
plt.xlabel('t/(ms)')
plt.ylabel('X/(m)')
# --------------------------------------------------------------------------------------------
plt.subplot(2, 4, 8)
plt.grid()
for i in range(num_follower):
    plt.plot(erry_actual_history[:,i], lw=2, label=f"follower{i}")
plt.legend()# 添加图例
plt.title('turtlebot y error')    # 设置图形标题和坐标轴标签
plt.xlabel('t/(ms)')
plt.ylabel('X/(m)')

plt.show()    # 显示图形

# -------------------------------------------------------------------------------------------
fig, ax  = plt.subplots(figsize=(10, 10))     # 可视化初值
ax.set_xlim(-1, 10)
ax.set_ylim(-1, 10)

leader_labels = []
follower_labels = []
for i in range(num_leader):
    leader_labels.append(ax.text(rx[i], ry[i], f"Leader {i+1}", ha='center', va='center', color='r', fontsize=8, fontweight='bold', alpha=0.3))
for i in range(num_follower):
    follower_labels.append(ax.text(x_history[0, i], y_history[0, i], f"Follower {i+1}", ha='center', va='center', color='b', fontsize=8, fontweight='bold', alpha=0.3))

leader_scatter = ax.scatter(rx, ry, c='r', marker='o', label='Leaders')      # 绘制点
leader_positions=np.zeros([num_leader, 2])
leader_positions[:,0]=rx
leader_positions[:,1]=ry
leader_scatter.set_offsets(leader_positions)

# Calculate the convex hull for leaders only
hull_line, = ax.plot([], [], c='r', linewidth=2)
hull = ConvexHull(leader_positions)
hull_vertices = np.append(hull.vertices, hull.vertices[0])  # Closing the hull by connecting the first vertex again
hull_line.set_xdata(leader_positions[hull_vertices, 0])
hull_line.set_ydata(leader_positions[hull_vertices, 1])

plt.plot(x_history, y_history, lw=2, alpha=0.3)
slice = 100

for fr in range(iter):             # 绘制箭头
    if fr % slice == 0:
        for i in range(num_follower):             # 绘制箭头
            dx = 0.5 * v_history[fr, i] * np.cos(theta_history[fr, i])     # 计算箭头的x方向分量
            dy = 0.5 * v_history[fr, i] * np.sin(theta_history[fr, i])     # 计算箭头的y方向分量
            # 更新箭头对象的位置和方向
            plt.arrow(x_history[fr,i], y_history[fr,i], dx, dy, head_width=0.05, head_length=0.1, fc='black', ec='black', alpha=0.4)
            ax.scatter(x_history[fr,i], y_history[fr,i], c='b', marker='.', label='Followers', alpha=0.3)


plt.title('turtlebot position and pose')    # 设置图形标题和坐标轴标签
plt.xlabel('X/(m)')
plt.ylabel('Y/(m)')
plt.show()    # 显示图形
# -------------------------------------------------制作动画-------------------------------------------
# fig, ax  = plt.subplots(figsize=(8, 8))     # 可视化初值
# ax.set_xlim(-1, 10)
# ax.set_ylim(-1, 10)
# plt.grid()
# # ----
# leader_scatter = ax.scatter(rx, ry, c='r', marker='o', label='Leaders')
# follower_scatter = ax.scatter(x, y, c='b', marker='o', label='Followers')
# hull_line, = ax.plot([], [], c='r', linewidth=2)

# leader_labels = []
# follower_labels = []
# for i in range(num_leader):
#     leader_labels.append(ax.text(rx[i], ry[i], f"Leader {i+1}", ha='center', va='center', color='r', fontsize=8, fontweight='bold', alpha=0.3))
# for i in range(num_follower):
#     follower_labels.append(ax.text(x[i], y[i], f"Follower {i+1}", ha='center', va='center', color='b', fontsize=8, fontweight='bold', alpha=0.3))
# # ---------------------------------------------
# leader_positions=np.zeros([num_leader, 2])
# leader_positions[:,0]=rx
# leader_positions[:,1]=ry
# leader_scatter.set_offsets(leader_positions)

# for i in range(num_leader):
#     leader_labels[i].set_position((leader_positions[i,0] + 0.8, leader_positions[i,1] + 0.1))

# # Calculate the convex hull for leaders only
# hull = ConvexHull(leader_positions)
# hull_vertices = np.append(hull.vertices, hull.vertices[0])  # Closing the hull by connecting the first vertex again
# hull_line.set_xdata(leader_positions[hull_vertices, 0])
# hull_line.set_ydata(leader_positions[hull_vertices, 1])

# for i in range(len(x)):             # 绘制箭头
#     dx = 0.5 * np.cos(theta[i])     # 计算箭头的x方向分量
#     dy = 0.5 * np.sin(theta[i])     # 计算箭头的y方向分量
#     plt.arrow(x[i], y[i], dx, dy, head_width=0.1, head_length=0.2, fc='black', ec='black')
# plt.plot(x_history, y_history, lw=2)


# slice = 100
# follower_positions=np.zeros([num_follower, 2])

# for fr in range(iter):             # 绘制箭头
#     if fr % slice == 0:
#         for i in range(num_follower):             # 绘制箭头
#             dx = 0.5 * v_history[fr, i] * np.cos(theta_history[fr, i])     # 计算箭头的x方向分量
#             dy = 0.5 * v_history[fr, i] * np.sin(theta_history[fr, i])     # 计算箭头的y方向分量
#             # 更新箭头对象的位置和方向
#             plt.arrow(x_history[fr,i], y_history[fr,i], dx, dy, head_width=0.05, head_length=0.1, fc='black', ec='black')
#             ax.scatter(x_history[fr,i], y_history[fr,i], c='b', marker='o', label='Followers')

# plt.show()

# def update(frame):
#     if frame % slice == 0:
#         follower_positions[:,0]=x_history[frame,:]
#         follower_positions[:,1]=y_history[frame,:]
#         follower_scatter.set_offsets(follower_positions[:,:])
#         # # Update labels
#         # for i in range(num_follower):
#         #     follower_labels[i].set_position((follower_positions[i, 0] + 0.8, follower_positions[i, 1] + 0.1))
#         # 显示进度条
#         with tqdm(total=iter/slice) as pbar:
#             pbar.update(frame/slice)
#     return follower_scatter,  #*follower_labels

# # Create the animation
# ani = animation.FuncAnimation(fig, update, frames=iter, interval=2*T, blit=True)

# # Save the animation as a gif
# ani.save('turtlebot_trajectories.gif', writer='pillow')
