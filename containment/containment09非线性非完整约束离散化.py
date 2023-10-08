import numpy as np
import random

# 随机种子
random.seed(12345)

# 迭代设置
iter = 1000
T = 0.02

# 初值
x=[0+4*random.random(),
   0+4*random.random(),
   0+4*random.random(),
   0+4*random.random(),
   0+4*random.random(),
   0+4*random.random()]
y=[6+4*random.random(),
   6+4*random.random(),
   6+4*random.random(),
   6+4*random.random(),
   6+4*random.random(),
   6+4*random.random()]
theta=[-3.14+6.28*random.random(),
       -3.14+6.28*random.random(),
       -3.14+6.28*random.random(),
       -3.14+6.28*random.random(),
       -3.14+6.28*random.random(),
       -3.14+6.28*random.random()]
hat_theta_d=[0,0,0,0,0,0]

# 系数
k1=0.2
k2=0.2
k3=0.3
R=100

for i in range(iter):
	# 控制方程
    hat_e[i]=-np.sum
    # 系统方程
    x[i]=x[i]+v[i]*T*np.cos(theta[i])    # v和x是N个智能体的速度和x坐标
    y[i]=y[i]+v[i]*T*np.sin(theta[i])    # y是N个智能体的y坐标
    theta[i]=theta[i]+omega[i]*T