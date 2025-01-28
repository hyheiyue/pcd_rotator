cmake_minimum_required(VERSION 3.8)
project(pcd_rotator)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# 包含目录
include_directories(
  include
  ${pcl_ros_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(pcd_rotator
  src/pcd_rotator.cpp
)

# 链接库
target_link_libraries(pcd_rotator
  ${pcl_ros_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 安装目标
install(TARGETS pcd_rotator
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()