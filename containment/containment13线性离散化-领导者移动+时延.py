import numpy as np
import random
import matplotlib.pyplot as plt
import math
from scipy.spatial import ConvexHull
import matplotlib.animation as animation
from shapely.geometry import Point, Polygon
from tqdm import tqdm

# 随机种子
random.seed(88)

# 迭代设置
t_sum = 10
T = 0.0001
iter = int(np.floor(t_sum/T))
num_follower = 6
num_leader = 4

# 初值
x = [0 + 2 * random.random(),
     0 + 2 * random.random(),
     0 + 2 * random.random(),
     0 + 2 * random.random(),
     0 + 2 * random.random(),
     0 + 2 * random.random()]
y = [0 + 2 * random.random(),
     0 + 2 * random.random(),
     0 + 2 * random.random(),
     0 + 2 * random.random(),
     0 + 2 * random.random(),
     0 + 2 * random.random()]
theta = [-3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random(),
         -3.14 + 6.28 * random.random()]
hat_theta_d = [0,0,0,0,0,0]
dot_hat_theta_d = [0,0,0,0,0,0]
rx = [1 + 2 * random.random(),
     1 + 2 * random.random(),
     1 + 2 * random.random(),
     1 + 2 * random.random()]
ry = [1 + 2 * random.random(),
     1 + 2 * random.random(),
     1 + 2 * random.random(),
     1 + 2 * random.random()]
dot_rx = [5/T/iter,5/T/iter,5/T/iter,5/T/iter]
dot_ry = [5/T/iter,5/T/iter,5/T/iter,5/T/iter]

# 系数
k1 = 3.8
k2 = 3.8
k3 = 0.12
R = 100

# 网络拓扑
# A_F = np.array([[0,1,0,0,0,0],
#                 [0,0,0,0,0,0],
#                 [0,0,0,0,0,0],
#                 [0,1,0,0,0,0],
#                 [1,0,0,0,0,1],
#                 [0,0,1,0,1,0]])   # 跟随者邻接矩阵
# B = np.array([[1,0,0,0,0,0],
#               [0,2,0,0,0,0],
#               [0,0,1,0,0,0],
#               [0,0,0,1,0,0],
#               [0,0,0,0,1,0],
#               [0,0,0,0,0,0]])   # 跟随者能接受到的领导者信息总和
# A_LF = np.array([[1,0,0,0],
#                  [0,1,1,0],
#                  [0,0,0,1],
#                  [1,0,0,0],
#                  [0,1,0,0],
#                  [0,0,0,0]])    # 领导者和跟随者的耦合邻接矩阵
A_F = np.array([[0,1,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,1,0,0,0,0],
                [1,0,0,0,0,1],
                [0,0,1,0,1,0]])   # 跟随者邻接矩阵
B = np.array([[0,0,0,0,0,0],
              [0,2,0,0,0,0],
              [0,0,1,0,0,0],
              [0,0,0,1,0,0],
              [0,0,0,0,1,0],
              [0,0,0,0,0,0]])   # 跟随者能接受到的领导者信息总和
A_LF = np.array([[0,0,0,0],
                 [0,1,1,0],
                 [0,0,0,1],
                 [1,0,0,0],
                 [0,1,0,0],
                 [0,0,0,0]])    # 领导者和跟随者的耦合邻接矩阵
# A_F = np.array([[0,1,0,0,0,0],
#                 [0,0,0,0,0,0],
#                 [0,0,0,0,0,0],
#                 [0,1,0,0,0,0],
#                 [1,0,0,0,0,1],
#                 [0,0,1,0,1,0]])   # 跟随者邻接矩阵
# B = np.array([[1,0,0,0,0,0],
#               [0,2,0,0,0,0],
#               [0,0,1,0,0,0],
#               [0,0,0,1,0,0],
#               [0,0,0,0,1,0],
#               [0,0,0,0,0,1]])   # 跟随者能接受到的领导者信息总和 6接到3的信息
# A_LF = np.array([[1,0,0,0],
#                  [0,1,1,0],
#                  [0,0,0,1],
#                  [1,0,0,0],
#                  [0,1,0,0],
#                  [0,0,1,0]])    # 领导者和跟随者的耦合邻接矩阵 6接到3的信息
L = - A_F
for i in range(num_follower):
    L[i,i] = np.sum(A_F[i,:])
L1 = L + B
L2 = - A_LF
xishu = np.dot(np.linalg.inv(L1), L2)

# 再写有时延的版本，稍后把时变的时延加到循环中
tau = np.array([[0.009, 0.010, 0.002, 0.004, 0.002, 0.009],
              [0.009, 0.010, 0.002, 0.004, 0.002, 0.009],
              [0.009, 0.010, 0.002, 0.004, 0.002, 0.009],
              [0.009, 0.010, 0.002, 0.004, 0.002, 0.009],
              [0.009, 0.010, 0.002, 0.004, 0.002, 0.009],
              [0.009, 0.010, 0.002, 0.004, 0.002, 0.009]])  # 初始化时延
d = np.floor(tau/0.001).astype(int)

non_zero_indices = A_F != 0    # 找到 A 中不等于0的元素的位置
# ------------------------------主要修改这里--------------------------------
tau_actual = [0, 0, 0, 0, 0, 0]
d[non_zero_indices] = tau_actual    # 使用布尔索引将新值赋予 d 矩阵

dmax = np.max(d)

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
rx_history = []
ry_history = []
hat_ex_history = []
hat_ey_history = []
errx_actual_history = []
erry_actual_history = []
hat_evx_history = []
hat_evy_history = []

for i in range(dmax):
    x_history.append(x)
    y_history.append(y)
    theta_history.append(theta)
    rx_history.append(rx)
    ry_history.append(ry)
    

for k in range(iter):
    hat_ex = [0,0,0,0,0,0]
    hat_ey = [0,0,0,0,0,0]
    dot_dx = [0,0,0,0,0,0]
    dot_dy = [0,0,0,0,0,0]
    hat_evx = [0,0,0,0,0,0]
    hat_evy = [0,0,0,0,0,0]
    # 控制方程
    for i in range(num_follower):
        for j in range(num_follower):
            hat_ex[i] = hat_ex[i] - A_F[i][j] * (x[i] - x[j])
            hat_ey[i] = hat_ey[i] - A_F[i][j] * (y[i] - y[j])
            # hat_ex[i] = hat_ex[i] - A_F[i][j] * (x[i] - x_history[-d[i][j]][j])
            # hat_ey[i] = hat_ey[i] - A_F[i][j] * (y[i] - y_history[-d[i][j]][j])
        for j in range(num_leader):
            hat_ex[i] = hat_ex[i] - A_LF[i][j] * (x[i] - rx[j])
            hat_ey[i] = hat_ey[i] - A_LF[i][j] * (y[i] - ry[j])
            # hat_ex[i] = hat_ex[i] - A_LF[i][j] * (x[i] - rx_history[-d[i][j]][j])
            # hat_ey[i] = hat_ey[i] - A_LF[i][j] * (y[i] - ry_history[-d[i][j]][j])
            # dot_dx[i] = dot_dx[i] + A_LF[i][j] * dot_rx[j]
            # dot_dy[i] = dot_dy[i] + A_LF[i][j] * dot_ry[j]
        dot_dx[i] = dot_rx[1]
        dot_dy[i] = dot_ry[1]
        hat_evx[i] = dot_rx[1] - v[i] * np.sin(theta[i])
        hat_evy[i] = dot_ry[1] - v[i] * np.cos(theta[i])
        ux[i] = dot_dx[i] + k1 * hat_ex[i]
        uy[i] = dot_dy[i] + k2 * hat_ey[i]

    errx_actual = np.dot(L1,x) + np.dot(L2, rx)
    erry_actual = np.dot(L1,y) + np.dot(L2, ry)
    # 系统方程
    for i in range(num_follower):
        x[i] = x[i] + ux[i] * T   # v和x是N个智能体的速度和x坐标
        y[i] = y[i] + uy[i] * T  # y是N个智能体的y坐标
    for i in range(num_leader):
        rx[i] = rx[i] + dot_rx[i] * T
        ry[i] = ry[i] + dot_ry[i] * T

    # errx_actual = x + np.dot(xishu, rx)
    # erry_actual = y + np.dot(xishu, ry)
    
    # print(x)
    # print(x, '\n', y, '\n', theta)
    x_history.append([x[0],x[1],x[2],x[3],x[4],x[5]])
    y_history.append([y[0],y[1],y[2],y[3],y[4],y[5]])
    theta_history.append([theta[0],theta[1],theta[2],theta[3],theta[4],theta[5]])
    v_history.append([v[0],v[1],v[2],v[3],v[4],v[5]])
    omega_history.append([omega[0],omega[1],omega[2],omega[3],omega[4],omega[5]])
    rx_history.append([rx[0],rx[1],rx[2],rx[3]])
    ry_history.append([ry[0],ry[1],ry[2],ry[3]])
    hat_ex_history.append([-hat_ex[0],-hat_ex[1],-hat_ex[2],-hat_ex[3],-hat_ex[4],-hat_ex[5]])
    hat_ey_history.append([-hat_ey[0],-hat_ey[1],-hat_ey[2],-hat_ey[3],-hat_ey[4],-hat_ey[5]])
    errx_actual_history.append([errx_actual[0],errx_actual[1],errx_actual[2],errx_actual[3],errx_actual[4],errx_actual[5]])
    erry_actual_history.append([erry_actual[0],erry_actual[1],erry_actual[2],erry_actual[3],erry_actual[4],erry_actual[5]])
    hat_evx_history.append([hat_evx[0],hat_evx[1],hat_evx[2],hat_evx[3],hat_evx[4],hat_evx[5]])
    hat_evy_history.append([hat_evy[0],hat_evy[1],hat_evy[2],hat_evy[3],hat_evy[4],hat_evy[5]])

# 指定保存的文件名
file_name = "containment_l.txt"

# 将测试点保存到文本文件中
with open(file_name, "w") as file:
    for i in range(iter):
        file.write("%.50f,%.50f\n" % (errx_actual_history[i][0], hat_ex_history[i][0]))

x_history = np.array(x_history)
y_history = np.array(y_history)
rx_history = np.array(rx_history)
ry_history = np.array(ry_history)
theta_history = np.array(theta_history)
v_history = np.array(v_history)
omega_history = np.array(omega_history)
hat_ex_history = np.array(hat_ex_history)
hat_ey_history = np.array(hat_ey_history)
errx_actual_history = np.array(errx_actual_history)
erry_actual_history = np.array(erry_actual_history)
hat_evx_history = np.array(hat_ex_history)
hat_evy_history = np.array(hat_ey_history)

# plt.subplot(2, 1, 1)
# plt.grid()
# for i in range(num_follower):
#     plt.plot(hat_evx_history[:,i], lw=2, label=f"follower{i}")
# plt.legend()# 添加图例
# plt.title('turtlebot vx error')    # 设置图形标题和坐标轴标签
# plt.xlabel('t/(ms)')
# plt.ylabel('X/(m/s)')
# # --------------------------------------------------------------------------------------------
# plt.subplot(2, 1, 2)
# plt.grid()
# for i in range(num_follower):
#     plt.plot(hat_evy_history[:,i], lw=2, label=f"follower{i}")
# plt.legend()# 添加图例
# plt.title('turtlebot vy error')    # 设置图形标题和坐标轴标签
# plt.xlabel('t/(ms)')
# plt.ylabel('X/(m/s)')
# plt.show()
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
plt.savefig(f"turtlebot_trajectories T={T}, iter={iter}, delay={tau_actual} k={[k1,k2,k3]} error.png")
plt.show()    # 显示图形

# -------------------------------------------------------------------------------------------
fig, ax  = plt.subplots(figsize=(10, 10))     # 可视化初值
ax.set_xlim(-1, 11)
ax.set_ylim(-1, 11)

leader_labels = []
follower_labels = []
for i in range(num_leader):
    leader_labels.append(ax.text(rx[i], ry[i], f"Leader {i+1}", ha='center', va='center', color='r', fontsize=8, fontweight='bold', alpha=0.3))
for i in range(num_follower):
    follower_labels.append(ax.text(x_history[0, i], y_history[0, i], f"Follower {i+1}", ha='center', va='center', color='b', fontsize=8, fontweight='bold', alpha=0.3))

ax.scatter(x, y, c='b', marker='o', label='Followers')      # 绘制点
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

plt.plot(x_history[dmax:,:], y_history[dmax:,:], lw=2, alpha=0.3)
slice = np.floor(iter/20)

for fr in range(iter):             # 绘制箭头
    if fr % slice == 0:
        for i in range(num_follower):             # 绘制箭头
            dx = 0.5 * v_history[fr, i] * np.cos(theta_history[fr, i])     # 计算箭头的x方向分量
            dy = 0.5 * v_history[fr, i] * np.sin(theta_history[fr, i])     # 计算箭头的y方向分量
            # 更新箭头对象的位置和方向
            plt.arrow(x_history[fr,i], y_history[fr,i], dx, dy, head_width=0.05, head_length=0.1, fc='black', ec='black', alpha=0.4)
            ax.scatter(x_history[fr,i], y_history[fr,i], c='b', marker='.', label='Followers', alpha=0.3)


plt.title(f'turtlebot position and pose T={T}, iter={iter}')    # 设置图形标题和坐标轴标签
plt.xlabel('X/(m)')
plt.ylabel('Y/(m)')
plt.savefig(f"turtlebot_trajectories T={T}, iter={iter}, delay={tau_actual} k={[k1,k2,k3]}.png")
plt.show()    # 显示图形

# -------------------------------------------------制作动画-------------------------------------------
fig, ax  = plt.subplots(figsize=(8, 8))     # 可视化初值
ax.set_xlim(-1, 11)
ax.set_ylim(-1, 11)
plt.grid()
# ----
leader_scatter = ax.scatter(rx, ry, c='r', marker='o', label='Leaders')
follower_scatter = ax.scatter(x, y, c='b', marker='o', label='Followers')
hull_line, = ax.plot([], [], c='r', linewidth=2)

leader_labels = []
follower_labels = []
for i in range(num_leader):
    leader_labels.append(ax.text(rx[i], ry[i], f"Leader {i+1}", ha='center', va='center', color='r', fontsize=8, fontweight='bold', alpha=0.3))
for i in range(num_follower):
    follower_labels.append(ax.text(x[i], y[i], f"Follower {i+1}", ha='center', va='center', color='b', fontsize=8, fontweight='bold', alpha=0.3))

for i in range(len(x)):             # 绘制箭头
    dx = 0.5 * np.cos(theta[i])     # 计算箭头的x方向分量
    dy = 0.5 * np.sin(theta[i])     # 计算箭头的y方向分量
    plt.arrow(x[i], y[i], dx, dy, head_width=0.1, head_length=0.2, fc='black', ec='black')
plt.plot(x_history[dmax:,:], y_history[dmax:,:], lw=2, alpha=0.3)
plt.title(f"turtlebot_trajectories T={T}, iter={iter}, delay={tau_actual}")

slic = 100
slice = int(np.floor(iter/slic))
follower_positions=np.zeros([num_follower, 2])
leader_positions=np.zeros([num_leader, 2])

def update(frame):
    # if frame % slice == 0:
    follower_positions[:,0]=x_history[frame*slice+dmax,:]
    follower_positions[:,1]=y_history[frame*slice+dmax,:]
    follower_scatter.set_offsets(follower_positions[:,:])
    
    # ---------------------------------------------
    leader_positions[:,0]=rx_history[frame*slice,:]
    leader_positions[:,1]=ry_history[frame*slice,:]
    leader_scatter.set_offsets(leader_positions)

    for i in range(num_leader):
        leader_labels[i].set_position((leader_positions[i,0] + 0.8, leader_positions[i,1] + 0.1))
    for i in range(num_follower):
        follower_labels[i].set_position((follower_positions[i, 0] + 0.8, follower_positions[i, 1] + 0.1))

    # Calculate the convex hull for leaders only
    hull = ConvexHull(leader_positions)
    hull_vertices = np.append(hull.vertices, hull.vertices[0])  # Closing the hull by connecting the first vertex again
    hull_line.set_xdata(leader_positions[hull_vertices, 0])
    hull_line.set_ydata(leader_positions[hull_vertices, 1])
    print(frame)
    return follower_scatter, leader_scatter, hull_line

ani = animation.FuncAnimation(fig, update, frames=int(np.floor(iter/slice)), interval=1000*slice*T, blit=True)    # Create the animation

ani.save(f"turtlebot_trajectories T={T}, iter={iter}, delay={tau_actual} k={[k1,k2,k3]}.gif", writer='pillow')    # f"turtlebot_trajectories T={T}, iter={iter}.gif"  'turtlebot_trajectories.gif'