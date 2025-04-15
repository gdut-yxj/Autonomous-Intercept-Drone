cmake_minimum_required(VERSION 3.5)
project(ibvs_servo_point)

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
find_package(rclcpp REQUIRED)
find_package(VISP REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(px4_msgs REQUIRED)
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


include_directories(./include/ibvs_servo_point/)



add_executable(ibvs_servo_point ./src/main.cpp
)

target_link_libraries(ibvs_servo_point
        ${VISP_LIBRARIES}
        ${OpenCV_LIBS}
)


ament_target_dependencies(ibvs_servo_point rclcpp OpenCV px4_msgs uav_common_msg cv_bridge)				#添加节点的依赖

install(TARGETS
        ibvs_servo_point
        DESTINATION lib/${PROJECT_NAME})


        
# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
        
        
