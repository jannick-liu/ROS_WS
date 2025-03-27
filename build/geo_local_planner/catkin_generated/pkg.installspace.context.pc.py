# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "base_local_planner;dynamic_reconfigure;nav_msgs;pluginlib;sensor_msgs;roscpp;tf2;tf2_ros;mbf_costmap_core;costmap_2d;costmap_converter;joy;nav_core".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lgeo_local_planner".split(';') if "-lgeo_local_planner" != "" else []
PROJECT_NAME = "geo_local_planner"
PROJECT_SPACE_DIR = "/home/hao/ros_ws/install"
PROJECT_VERSION = "0.0.0"
