cmake_minimum_required(VERSION 3.8)
project(rm_vision_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==================== 依赖查找 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)

# CUDA & TensorRT
find_package(CUDA REQUIRED)
find_library(NVINFER_LIB nvinfer)
find_library(NVONNXPARSER_LIB nvonnxparser)

# 传递源码根目录宏给 detector_node，用于回退查找 weights
add_compile_definitions(SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../rm_vision")

# ==================== TRT YOLO 库 ====================
add_library(trt_yolo SHARED
  src/trt_yolo.cpp
)
target_include_directories(trt_yolo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(trt_yolo
  ${OpenCV_LIBS}
  ${NVINFER_LIB}
  ${CUDA_LIBRARIES}
)

# ==================== video_node 可执行文件 ====================
add_executable(video_node src/video_node.cpp)
target_include_directories(video_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(video_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)

# ==================== detector_node 可执行文件 ====================
add_executable(detector_node src/detector_node.cpp)
target_include_directories(detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDA_INCLUDE_DIRS}
)
target_link_libraries(detector_node
  trt_yolo
)
ament_target_dependencies(detector_node
  rclcpp
  sensor_msgs
  std_msgs
  rm_interfaces
  cv_bridge
  OpenCV
  ament_index_cpp
)

# ==================== 安装 ====================
install(TARGETS video_node detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS trt_yolo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 如果 weights 目录存在，也安装
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/weights")
  install(DIRECTORY weights/
    DESTINATION share/${PROJECT_NAME}/weights
  )
endif()

ament_package()
