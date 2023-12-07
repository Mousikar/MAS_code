import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

# 设置中文显示的字体
font_path = "/usr/share/fonts/myfontdir/SimHei.ttf"  # 替换为你的中文字体文件路径
fontprop = FontProperties(fname=font_path)

# # 读取原始文本文件的内容
# with open("followermove-1.txt", "r") as file:
#     lines = file.readlines()

# # 去掉每行末尾的空格和逗号
# cleaned_lines = [line.strip(", \n") + "\n" for line in lines]

# # 保存处理后的数据为新的文件
# new_file_name = "followermove-1clean.txt"
# with open(new_file_name, "w") as new_file:
#     new_file.writelines(cleaned_lines)

# print(f"处理后的数据已保存到 {new_file_name}")

# '''
# 从文本文件读取数据，使用逗号作为分隔符
data = np.loadtxt("followermove-1clean.txt", delimiter=",")

# 提取每列数据
num_columns = data.shape[1]  # 获取数据列数
column_names = [f"Column {i+1}" for i in range(num_columns)]

# 可视化每列数据 坐标变换
plt.figure(figsize=(10, 6))
for i in range(2):
    plt.plot(data[:, [i+j*5 for j in range(6)]])

plt.xlabel('迭代次数', fontproperties=fontprop)
plt.ylabel('位姿', fontproperties=fontprop)
plt.title('坐标变化', fontproperties=fontprop)
plt.legend()
plt.grid()
plt.show()

# 可视化每列数据 坐标变换 二维图像
plt.figure(figsize=(10, 6))

for j in range(6):
    plt.plot(data[:, [0+j*5]],data[:, [1+j*5]])

plt.xlabel('x')
plt.ylabel('y')
plt.title('二维坐标图像', fontproperties=fontprop)
plt.legend()
plt.grid()
plt.show()

# 可视化每列数据 坐标变换 角度变化
plt.figure(figsize=(10, 6))
for i in range(1):
    plt.plot(data[:, [2+i+j*5 for j in range(6)]])

plt.xlabel('迭代次数', fontproperties=fontprop)
plt.ylabel('theta')
plt.title('角度变换', fontproperties=fontprop)
plt.legend()
plt.grid()
plt.show()

# 可视化每列数据 控制输入
plt.figure(figsize=(10, 6))
for i in range(2):
    plt.plot(data[:, [3+i+j*5 for j in range(6)]])

plt.xlabel('迭代次数', fontproperties=fontprop)
plt.ylabel('v和w', fontproperties=fontprop)
plt.title('控制输入', fontproperties=fontprop)
plt.legend()
plt.grid()
plt.show()
# '''