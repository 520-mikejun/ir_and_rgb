cmake_minimum_required(VERSION 3.0.2)
project(mvs_camera_ros)

# 设置cv_bridge的路径
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

# 找到所需的catkin包
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  rosbag
)

# 找到OpenCV包
find_package(OpenCV REQUIRED)

# catkin包信息
catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES mvs_camera_ros 
  CATKIN_DEPENDS roscpp cv_bridge image_transport sensor_msgs std_msgs
  DEPENDS OpenCV
)

# 包含目录
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# 添加MVS库
add_library(MVS_Lib INTERFACE)
add_library(MVS::MVS ALIAS MVS_Lib)
target_include_directories(MVS_Lib INTERFACE /opt/MVS/include)
target_link_directories(MVS_Lib INTERFACE /opt/MVS/lib/64)
target_link_libraries(MVS_Lib INTERFACE MvCameraControl)

# 添加MVS相机库
add_library(MVS_Camera src/mvs_camera.cpp)
target_include_directories(MVS_Camera PUBLIC include)
target_link_libraries(MVS_Camera PUBLIC MVS::MVS ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

# 添加可执行文件
add_executable(${PROJECT_NAME}_node src/mvs_camera_ros_node.cpp)
add_dependencies(
    ${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# 链接库
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} MVS_Camera ${OpenCV_LIBRARIES})

# 安装目标
install(
    TARGETS ${PROJECT_NAME}_node
    RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
