cmake_minimum_required(VERSION 3.0.2)
project(nmpc_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            base_local_planner
            cmake_modules
            costmap_2d
            costmap_converter
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            sensor_msgs
            roscpp
            tf2
            tf2_geometry_msgs
            tf2_ros
            mbf_costmap_core
            mbf_msgs
            pcl_conversions
            pcl_ros
            joy
        )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(casadi REQUIRED)

remove_definitions(-DDISABLE_LIBUSB-1.0)


# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/NmpcLocalPlannerReconfigure.cfg
)

catkin_package(
    INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
    LIBRARIES nmpc_local_planner ${EXTERNAL_LIBS}
    CATKIN_DEPENDS
        base_local_planner
        dynamic_reconfigure
        nav_msgs
        pluginlib
        sensor_msgs
        roscpp
        tf2
        tf2_ros
        mbf_costmap_core
        costmap_2d
        costmap_converter
        joy
        nav_core
)


#set(IPOPT_INCLUDE_DIRS /usr/include)
set(IPOPT_LIBRARY_DIRS ~/robot-code/CoinIpopt/install/lib)
# set(IPOPT_LIBRARY_DIRS /usr/ipopt3124/lib)
# include_directories("/usr/include/coin")
# include_directories("/usr/include/coin/ThirdParty")
set(CASADI_INCLUDE_DIR ~/robot-code/CoinIpopt/install/include)

# find casadi library
find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()

include_directories(
    include
    ${EXTERNAL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CASADI_INCLUDE_DIR}
    ${IPOPT_INCLUDE_DIRS}
    )

link_directories(${PCL_LIBRARY_DIRS} ${IPOPT_LIBRARY_DIRS} ${CASADI_LIBRARIES})

add_definitions(${EIGEN3_DEFINITIONS} ${PCL_DEFINITIONS})


# include_directories("/home/mylog/software/ipopt3124/include/coin")
# include_directories("/home/mylog/software/ipopt3124/include/coin/ThirdParty")

add_library(nmpc_local_planner 
    src/nmpc_planner_ros.cpp
    src/nmpc_planner.cpp
    src/ref_path_planner.cpp
    src/obstacle_update.cpp) # src/nmpc_config.cpp
  
    
add_dependencies(nmpc_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nmpc_local_planner ipopt ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CASADI_LIBRARIES})


#add_executable(geo_path_planner scripts/geo_path_planner.cpp)
#target_link_libraries(geo_path_planner geo_local_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CASADI_LIBRARIES})
#add_dependencies(geo_path_planner geo_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



# add_executable(geo_path_planner scripts/geo_path_planner.cpp)
# target_link_libraries(geo_path_planner geo_local_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CASADI_LIBRARIES})
# add_dependencies(geo_path_planner geo_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(TARGETS nmpc_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(FILES blp_plugin.xml
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)