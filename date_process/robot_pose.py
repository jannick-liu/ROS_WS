#!/usr/bin/env python3

import os
import rosbag
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
from matplotlib.patches import Circle
import numpy as np

# 计算路径的总距离
def calculate_distance(x_vals, y_vals):
    distance = 0.0
    for i in range(1, len(x_vals)):
        dx = x_vals[i] - x_vals[i-1]
        dy = y_vals[i] - y_vals[i-1]
        distance += np.sqrt(dx**2 + dy**2)
    return distance

# 读取 .bag 文件中的 robot_pose（根据实际消息类型修改）
def read_robot_pose(bag_file, topic):
    bag = rosbag.Bag(bag_file)
    x_vals = []
    y_vals = []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        if topic_name == topic:
            x_vals.append(msg.x)
            y_vals.append(msg.y)
    bag.close()
    return x_vals, y_vals

# 读取 .bag 文件中的全局路径
def read_global_plan(bag_file, topic):
    bag = rosbag.Bag(bag_file)
    x_vals = []
    y_vals = []
    for topic_name, msg, t in bag.read_messages(topics=[topic]):
        if topic_name == topic:
            for pose in msg.poses:
                x_vals.append(pose.pose.position.x)
                y_vals.append(pose.pose.position.y)
    bag.close()
    return x_vals, y_vals

# 读取障碍物信息
def read_obstacles(bag_file, center_topic, radius_topic):
    bag = rosbag.Bag(bag_file)
    centers = []
    radii = []

    for topic_name, msg, t in bag.read_messages(topics=[center_topic, radius_topic]):
        if topic_name == center_topic:
            for pose in msg.poses:
                centers.append((pose.position.x, pose.position.y))
        elif topic_name == radius_topic:
            for data in msg.data:
                radii.append(data)
    
    bag.close()
    return centers, radii

def read_odom_path(bag_file, max_x=15.1):
    x_vals = []
    y_vals = []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic_name, msg, t in bag.read_messages(topics=['/odom']):
            if msg.pose.pose.position.x > max_x:  # 如果超过了x=15.1，停止添加
                break
            x_vals.append(msg.pose.pose.position.x)
            y_vals.append(msg.pose.pose.position.y)
    return x_vals, y_vals

# 读取两个数据集
bag_file_1 = os.path.expanduser('~/ros_ws/src/nmpc_local_planner/bagfiles/2025-03-05-11-25-42.bag')
bag_file_2 = os.path.expanduser('~/ros_ws/src/geo_local_planner/bagfiles/2025-03-05-11-23-23.bag')
dwa_bag_file = os.path.expanduser('~/ros_ws/src/dwa/bagfiles/2025-03-05-11-29-00.bag')  # DWA 路径文件
TEB_bag_file = os.path.expanduser('~/ros_ws/src/TEB/bagfiles/2025-03-05-11-27-38.bag')

# 获取 robot_pose 数据
robot_x_1, robot_y_1 = read_robot_pose(bag_file_1, '/move_base/nmpcPlannerROS/robot_pose')
robot_x_2, robot_y_2 = read_robot_pose(bag_file_2, '/move_base/GeoPlannerROS/robot_pose')

# 获取全局路径数据
plan_x_1, plan_y_1 = read_global_plan(bag_file_1, '/move_base/GlobalPlanner/plan')
plan_x_2, plan_y_2 = read_global_plan(bag_file_2, '/move_base/GlobalPlanner/plan')

# 获取 DWA 路径数据
dwa_x, dwa_y = read_odom_path(dwa_bag_file)
teb_x, teb_y = read_odom_path(TEB_bag_file)

# 获取障碍物数据
obs_centers_1, obs_radii_1 = read_obstacles(bag_file_1, '/move_base/nmpcPlannerROS/obs_center', '/move_base/nmpcPlannerROS/obs_radius')
obs_centers_2, obs_radii_2 = read_obstacles(bag_file_2, '/move_base/GeoPlannerROS/obs_center', '/move_base/GeoPlannerROS/obs_radius')

# 计算并显示各个路径的总距离
distance_robot_1 = calculate_distance(robot_x_1, robot_y_1)
distance_robot_2 = calculate_distance(robot_x_2, robot_y_2)
distance_dwa = calculate_distance(dwa_x, dwa_y)
distance_teb = calculate_distance(teb_x, teb_y)

print(f"Total distance (NMPC): {distance_robot_1:.2f} meters")
print(f"Total distance (GeoPlanner): {distance_robot_2:.2f} meters")
print(f"Total distance (dwa): {distance_dwa:.2f} meters")
print(f"Total distance (TEB): {distance_teb:.2f} meters")

# 绘制轨迹
plt.figure(figsize=(8, 6))

# 绘制机器人路径 (NMPC)
plt.plot(robot_x_1, robot_y_1, label="NMPC", color="blue")

# 绘制机器人路径 (GeoPlanner)
plt.plot(robot_x_2, robot_y_2, label="Algorithm in this paper", color="red")

# 绘制 DWA 路径
plt.plot(dwa_x, dwa_y, label="dwa Path", color="green")

#TEB
plt.plot(teb_x, teb_y, label="TEB Path", color="orange")

# 绘制全局路径 (Global Plan)
plt.plot(plan_x_1, plan_y_1, label="Global Plan", color="purple", linestyle="--")


## 绘制障碍物
#ax = plt.gca()
#for i in range(len(obs_centers_2)):
##for i in range(10):
#    center_x, center_y = obs_centers_2[i]
#    radius = obs_radii_2[i] if i < len(obs_radii_2) else 0
#    circle = Circle((center_x, center_y), radius, color='gray', alpha=0.3, label="Obstacle" if i == 0 else "")
#    ax.add_patch(circle)

 # 指定障碍物的坐标和半径
obstacle_coords = [(3.90963, -0.94063), (5.9177, 2.19054), (7.96832, -0.588483)]
obstacle_radii = [0.5, 0.5, 0.5]

# 绘制障碍物
ax = plt.gca()
for i, (center_x, center_y) in enumerate(obstacle_coords):
    radius = obstacle_radii[i]
    circle = Circle((center_x, center_y), radius, color='gray', alpha=0.3, label="Obstacle" if i == 0 else "")
    ax.add_patch(circle)


# 添加 y=3 和 y=-3 的边界线
plt.axhline(y=2.5, color='g', linestyle='--', label='Boundary on the sidewalk')
plt.axhline(y=-2.5, color='g', linestyle='--', label='Boundary on the sidewalk')

# 假设车辆宽度为 1.0 米
w = 1

# 计算车辆的上下边界轨迹
y_upper = np.array(robot_y_2) + w / 2
y_lower = np.array(robot_y_2) - w / 2
plt.plot(robot_x_2, y_upper, linestyle="--", color="orange", label="Upper Bound of the robot")
plt.plot(robot_x_2, y_lower, linestyle="--", color="y", label="Lower Bound of the robot")


# 设置x和y的范围
plt.xlim(-0.5, 16.5)  # x范围为(0, 8)
plt.ylim(-3.25, 3.25)  # y范围为(-3, 3)
# 图形设置
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
#plt.title("Comparison of Robot Paths (TrajektoryPlanner, NMPC, Algorithm in this paper)")
plt.legend()
plt.legend(loc='upper right')
plt.grid(True)
plt.axis("equal")

# 显示图形
plt.show()
