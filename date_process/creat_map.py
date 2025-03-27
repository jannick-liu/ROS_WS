import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cv2
import yaml

# 定义地图的尺寸
map_width = 20  # 长度 10米
map_height = 8  # 宽度 8米
resolution = 0.05  # 每个像素对应 0.05 米

# 计算图像的大小（以像素为单位）
img_width = int(map_width / resolution)  # 图像宽度
img_height = int(map_height / resolution)  # 图像高度

# 创建空白的地图，初始化为全白（表示空地）
map_data = np.ones((img_height, img_width), dtype=np.uint8) * 255  # 255为白色

# 中心位置（坐标轴在中点）
center_x = 1
center_y = map_height/2

# 绘制障碍物（圆形），坐标为(5,4)的右侧（5米，4米）
#obstacle_1_center = (5, 4)  # 第一个障碍物位置
#obstacle_2_center = (6, 4)  # 第二个障碍物位置
#obstacle_3_center = (7, 4)  # 第三个障碍物位置

# 障碍物半径
#radius_1 = 0.5  # 0.5 米
#radius_2 = 0.8  # 0.8 米
#radius_3 = 0.3  # 0.3 米

# 将障碍物绘制为圆形，转换为像素坐标
#def draw_circle(image, cx, cy, radius, value=0):
#    """
#    在图像上画一个圆，value为圆的颜色（0: 黑色表示障碍物，255: 白色表示空地）
#    """
#    cx_pixel = int(cx / resolution)
#    cy_pixel = int(cy / resolution)
#    radius_pixel = int(radius / resolution)
    
    # 创建圆形的掩膜
#    cv2.circle(image, (cx_pixel, cy_pixel), radius_pixel, (value), thickness=-1)

# 绘制障碍物
#draw_circle(map_data, obstacle_1_center[0], obstacle_1_center[1], radius_1)
#draw_circle(map_data, obstacle_2_center[0], obstacle_2_center[1], radius_2)
#draw_circle(map_data, obstacle_3_center[0], obstacle_3_center[1], radius_3)

# 保存路径
pgm_filename = "/home/hao/ros_ws/src/panther_simulation/maps/empty.pgm"
yaml_filename = "/home/hao/ros_ws/src/panther_simulation/maps/empty.yaml"

# 保存 PGM 图像
cv2.imwrite(pgm_filename, map_data)

# 创建 yaml 配置文件
yaml_data = {
    'image': pgm_filename,
    'resolution': resolution,
    'origin': [center_x, center_y, 0],  # 原点偏移为 -3.33米，图像底部对齐4米
    'occupied_thresh': 0.65,
    'free_thresh': 0.2,
    'negate': 0
}

# 保存 yaml 文件
with open(yaml_filename, 'w') as yaml_file:
    yaml.dump(yaml_data, yaml_file)

print(f"地图文件 {pgm_filename} 和 {yaml_filename} 已生成！")

