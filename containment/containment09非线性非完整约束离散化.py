import numpy as np

# 迭代设置
iter = 1000
T = 0.02

# 初值
import numpy as np
rd = np.random.RandomState(888) # 随机种子
follower_positions=np.array([[0, 6],
                            [0, 6],
                            [0, 6],
                            [0, 6],
                            [0, 6],
                            [0, 6]]) + 4 * rd.random((6,2))


for i in range(iter):
	# 控制方程
    hat_e[i]=-np.sum
    # 系统方程
    x[i]=x[i]+v[i]*T*np.cos(theta[i])    # v和x是N个智能体的速度和x坐标
    y[i]=y[i]+v[i]*T*np.sin(theta[i])    # y是N个智能体的y坐标
    theta[i]=theta[i]+omega[i]*T