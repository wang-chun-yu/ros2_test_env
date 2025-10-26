cmake_minimum_required(VERSION 3.8)
project(u_webrtc)

# C++ 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 查找 nlohmann_json（用于 JSON 解析）
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  message(STATUS "nlohmann_json not found, using header-only version")
  # 如果系统没有安装 nlohmann_json，可以使用 header-only 版本
  # 或者通过 apt 安装：sudo apt-get install nlohmann-json3-dev
endif()

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(webrtc_streamer_node
  src/main.cpp
  src/image_subscriber.cpp
  src/image_converter.cpp
  src/webrtc_streamer.cpp
  src/peer_connection_wrapper.cpp
  src/signaling_client.cpp
  src/video_encoder.cpp
  src/config_manager.cpp
)

# 链接 ROS2 依赖
ament_target_dependencies(webrtc_streamer_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# 链接其他库
target_link_libraries(webrtc_streamer_node
  ${OpenCV_LIBS}
)

# 如果找到了 nlohmann_json，链接它
if(nlohmann_json_FOUND)
  target_link_libraries(webrtc_streamer_node nlohmann_json::nlohmann_json)
endif()

# 安装可执行文件
install(TARGETS webrtc_streamer_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装配置文件和启动文件
install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过一些不必要的 lint 检查
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  # 可以添加 GTest 单元测试
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(test_image_converter test/test_image_converter.cpp)
  # target_link_libraries(test_image_converter ${OpenCV_LIBS})
endif()

ament_package()



