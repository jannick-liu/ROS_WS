# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "/home/hao/ros_ws/devel/include;/home/hao/ros_ws/src/obstacle_detector/include".split(';') if "/home/hao/ros_ws/devel/include;/home/hao/ros_ws/src/obstacle_detector/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;nodelet;rviz;std_msgs;std_srvs;geometry_msgs;sensor_msgs;tf;laser_geometry;message_runtime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lscans_merger;-lobstacle_extractor;-lobstacle_tracker;-lobstacle_publisher;-lobstacle_detector_nodelets;-lobstacle_detector_gui".split(';') if "-lscans_merger;-lobstacle_extractor;-lobstacle_tracker;-lobstacle_publisher;-lobstacle_detector_nodelets;-lobstacle_detector_gui" != "" else []
PROJECT_NAME = "obstacle_detector"
PROJECT_SPACE_DIR = "/home/hao/ros_ws/devel"
PROJECT_VERSION = "1.0.0"
