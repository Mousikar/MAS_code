import numpy as np
import matplotlib.pyplot as plt


# # 读取原始文本文件的内容
# with open("test_points20230810.txt", "r") as file:
#     lines = file.readlines()

# # 去掉每行末尾的空格和逗号
# cleaned_lines = [line.strip(", \n") + "\n" for line in lines]

# # 保存处理后的数据为新的文件
# new_file_name = "cleaned_test_points.txt"
# with open(new_file_name, "w") as new_file:
#     new_file.writelines(cleaned_lines)

# print(f"处理后的数据已保存到 {new_file_name}")

# 从文本文件读取数据，使用逗号作为分隔符
data = np.loadtxt("cleaned_test_points.txt", delimiter=",")

# 提取每列数据
num_columns = data.shape[1]  # 获取数据列数
column_names = [f"Column {i+1}" for i in range(num_columns)]

# 可视化每列数据 坐标变换
plt.figure(figsize=(10, 6))
for i in range(3):
    plt.plot(data[:, [i+j*5 for j in range(6)]])

plt.xlabel('Row Index')
plt.ylabel('Value')
plt.title('Visualization of Columns')
plt.legend()
plt.grid()
plt.show()

# 可视化每列数据 控制输入
plt.figure(figsize=(10, 6))
for i in range(2):
    plt.plot(data[:, [3+i+j*5 for j in range(6)]])

plt.xlabel('Row Index')
plt.ylabel('Value')
plt.title('Visualization of Columns')
plt.legend()
plt.grid()
plt.show()