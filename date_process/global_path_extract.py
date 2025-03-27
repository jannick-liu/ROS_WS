#!/usr/bin/env python3

import os
import csv
import rosbag
from geometry_msgs.msg import PoseStamped

# 定义函数读取并保存坐标信息到 CSV 文件
def save_coordinates_to_csv(bag_file, output_csv_file):
    # 打开源 .bag 文件
    bag = rosbag.Bag(bag_file, 'r')

    # 创建 CSV 文件用于保存坐标数据
    with open(output_csv_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        # 写入 CSV 文件的头部
        writer.writerow(['x', 'y'])
        
        # 读取 /move_base/GlobalPlanner/plan 话题的数据
        for topic, msg, t in bag.read_messages(topics=['/move_base/GlobalPlanner/plan']):
            # 遍历路径中的每个位置
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                # 将 x 和 y 坐标写入 CSV 文件
                writer.writerow([x, y])

    # 关闭 .bag 文件
    bag.close()

    print(f"坐标信息已保存到 {output_csv_file}")

# 初始化 ROS 节点
import rospy
rospy.init_node('bag_reader', anonymous=True)

# 使用 os.path.expanduser 展开路径
bag_file = os.path.expanduser('~/ros_ws/bagfiles/2025-02-27-15-04-07.bag')
output_csv_file = os.path.expanduser('~/ros_ws/date_process/global_path/Dijikstra.csv')

# 调用保存坐标信息的函数
save_coordinates_to_csv(bag_file, output_csv_file)
