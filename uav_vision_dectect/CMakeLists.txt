cmake_minimum_required(VERSION 3.5)
project(uav_vision_dectect)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 70 75 80)
endif(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(uav_common_msg REQUIRED)


find_package(VISP REQUIRED)
include_directories(${VISP_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

include_directories(
        ./include/uav_vision_dectect/
        ./include/yolov5_tensorRT
        /usr/local/cuda/include
        /opt/TensorRT-8.6/include/
        ./include/light_track
        ./include/light_track/ncnn

)


link_directories(
        /usr/local/cuda/lib64
        /opt/TensorRT-8.6/lib/
        ./include/light_track/lib
)

add_executable(uav_vision_dectect
        ./src/main.cpp
        ./src/uav_topic_subscrib.cpp
        ./src/uav_control.cpp
        ./src/calibrator.cpp
        ./src/preprocess.cu
        ./src/yololayer.cu
        ./src/LightTrack.cpp
)

target_link_libraries(uav_vision_dectect
        -lopencv_world
        -lncnn
        -lvulkan
        nvinfer
        cudart
        ${OpenCV_LIBS}
        ${VISP_LIBRARIES}
)


#添加节点的依赖
ament_target_dependencies(uav_vision_dectect
    rclcpp
    cv_bridge
    sensor_msgs
    OpenCV
    px4_msgs
    uav_common_msg
)


install(TARGETS
        uav_vision_dectect
        DESTINATION lib/${PROJECT_NAME}
)

