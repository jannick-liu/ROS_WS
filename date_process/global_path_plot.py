#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt

# 定义从 CSV 文件中读取坐标并绘制图形的函数
def plot_coordinates_from_csv(csv_file, label, color, linestyle='-'):
    df = pd.read_csv(csv_file)
    print(f"Columns in {csv_file}: {df.columns}")  # 打印列名
    
    # 确保 x 和 y 是一维的
    x_vals = df["x"].values.flatten()
    y_vals = df["y"].values.flatten()
    
    plt.plot(x_vals, y_vals, label=label, color=color, linestyle=linestyle)

# 读取 CSV 文件路径
a_star_csv_file = os.path.expanduser('~/ros_ws/date_process/global_path/A_star.csv')
dijkstra_csv_file = os.path.expanduser('~/ros_ws/date_process/global_path/Dijikstra.csv')
geo_csv_file = os.path.expanduser('~/ros_ws/date_process/global_path/geo.csv')

# 创建图形
plt.figure(figsize=(8, 6))

# 绘制 A* 路径
plot_coordinates_from_csv(a_star_csv_file, label="A* Path", color="blue")

# 绘制 Dijkstra 路径
plot_coordinates_from_csv(dijkstra_csv_file, label="Dijkstra Path", color="black")

# 绘制 Geo 路径（中线）
df_geo = pd.read_csv(geo_csv_file)
x_vals_geo = df_geo["x_val"].values.flatten()
y_vals_geo = df_geo["y_val"].values.flatten()
plt.plot(x_vals_geo, y_vals_geo, label="Algorithm in this paper", color="red")

# 绘制上边界（虚线）
x_up_vals = df_geo["x_up_val"].values.flatten()
y_up_vals = df_geo["y_up_val"].values.flatten()
plt.plot(x_up_vals, y_up_vals, label="Upper Bound of the robot", color="orange", linestyle="--")

# 绘制下边界（虚线）
x_down_vals = df_geo["x_down_val"].values.flatten()
y_down_vals = df_geo["y_down_val"].values.flatten()
plt.plot(x_down_vals, y_down_vals, label="Lower Bound of the robot", color="y", linestyle="--")

# 绘制障碍物（圆形）
obstacles = [
    ((4, 0), 0.5),
    ((5, 0), 0.8),
    ((6, 0), 0.3)
]

ax = plt.gca()
for idx, (center, radius) in enumerate(obstacles):
    # 为第一个障碍物添加标签，其他不添加
    label = "Obstacle" if idx == 0 else "_nolegend_"
    circle = plt.Circle(center, radius, color='gray', alpha=0.5, edgecolor='black', label=label)
    ax.add_patch(circle)

# 图形设置
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")

# 设置x和y的范围
plt.xlim(-0.5, 8.5)  # x范围为(0, 8)
plt.ylim(-3.25, 3.25)  # y范围为(-3, 3)

# 绘制人行道边界（绿色的y=-3和y=3线）
plt.axhline(y=-2.5, color='green', linestyle='--', label="Sidewalk Boundary")
plt.axhline(y=2.5, color='green', linestyle='--', label="Sidewalk Boundary")

# 添加图例
plt.legend()
plt.legend(loc='upper left')
plt.grid(True)
plt.axis("equal")

# 显示图形
plt.show()
