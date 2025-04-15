cmake_minimum_required(VERSION 3.5)
project(uav_chase)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# 允许使用C++写出ros2节点
find_package(rclcpp REQUIRED)
# ros2标准信息
find_package(std_msgs REQUIRED)
# 传感器数据信息
find_package(sensor_msgs REQUIRED)
# px4信息
find_package(px4_msgs REQUIRED)
# 地理坐标解析库
find_package(GeographicLib 2.4 REQUIRED)
# 导入师兄结果需要的msg
find_package(uav_common_msg REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(uav_chase
        ./src/uav_chase.cpp
)
ament_target_dependencies(uav_chase
        rclcpp
        sensor_msgs
        px4_msgs
        uav_common_msg
)

install(TARGETS
        uav_chase
        DESTINATION lib/${PROJECT_NAME})