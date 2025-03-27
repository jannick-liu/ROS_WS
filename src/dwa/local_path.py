import rosbag
import matplotlib.pyplot as plt

# 定义 odom 消息的存储位置
x_pos = []
y_pos = []

# 打开 bag 文件
bag_file = "/home/hao/ros_ws/src/dwa_local_planner/bagfiles/2025-03-03-11-25-51.bag"
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/odom']):
        # 提取位置数据
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        x_pos.append(x)
        y_pos.append(y)

# 画出轨迹
if len(x_pos) > 0 and len(y_pos) > 0:
    plt.plot(x_pos, y_pos)
    plt.title('Robot trajectory')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid(True)
    plt.show()
else:
    print("No odom data found in the bag file.")
