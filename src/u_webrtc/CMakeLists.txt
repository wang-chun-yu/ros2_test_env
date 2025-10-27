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

# 查找 LibDataChannel（可选）
# 注意：包名是 LibDataChannel（首字母大写）
find_package(LibDataChannel QUIET)
if(LibDataChannel_FOUND)
  message(STATUS "✅ Found LibDataChannel - WebRTC functionality enabled")
  message(STATUS "   Version: ${LibDataChannel_VERSION}")
  message(STATUS "   Include: ${LibDataChannel_INCLUDE_DIRS}")
  add_definitions(-DUSE_LIBDATACHANNEL)
else()
  message(WARNING "⚠️  LibDataChannel not found - using framework implementation only")
  message(STATUS "To enable full WebRTC support, install libdatachannel:")
  message(STATUS "  See: scripts/install_libdatachannel.sh")
endif()

# 查找 libvpx（视频编码器，可选）
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(VPX QUIET vpx)
  if(VPX_FOUND)
    message(STATUS "✅ Found libvpx - VP8/VP9 encoding enabled")
    message(STATUS "   Version: ${VPX_VERSION}")
    add_definitions(-DUSE_LIBVPX)
  else()
    message(WARNING "⚠️  libvpx not found - using framework encoder")
    message(STATUS "To enable VP8/VP9 encoding, install libvpx:")
    message(STATUS "  sudo apt-get install libvpx-dev")
  endif()
endif()

# 查找 WebSocket++（信令客户端，可选）
find_package(Boost QUIET COMPONENTS system thread)
if(Boost_FOUND)
  # 检查 WebSocket++ 头文件
  find_path(WEBSOCKETPP_INCLUDE_DIR 
    NAMES websocketpp/config/asio_client.hpp
    PATHS /usr/include /usr/local/include
  )
  
  if(WEBSOCKETPP_INCLUDE_DIR)
    message(STATUS "✅ Found WebSocket++ - Real WebSocket signaling enabled")
    message(STATUS "   Path: ${WEBSOCKETPP_INCLUDE_DIR}")
    add_definitions(-DUSE_WEBSOCKETPP)
    set(WEBSOCKETPP_FOUND TRUE)
  else()
    message(WARNING "⚠️  WebSocket++ not found - using framework signaling client")
    message(STATUS "To enable WebSocket signaling, install websocketpp:")
    message(STATUS "  sudo apt-get install libwebsocketpp-dev libboost-dev")
  endif()
else()
  message(WARNING "⚠️  Boost not found - WebSocket++ requires Boost")
endif()

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
  ${VPX_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 仅在找到 WebSocket++ 时添加其包含目录
if(WEBSOCKETPP_FOUND AND WEBSOCKETPP_INCLUDE_DIR)
  include_directories(${WEBSOCKETPP_INCLUDE_DIR})
endif()

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
  src/rtp_packetizer.cpp
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

# 如果找到了 LibDataChannel，链接它
if(LibDataChannel_FOUND)
  target_link_libraries(webrtc_streamer_node LibDataChannel::LibDataChannel)
  message(STATUS "✅ Linked LibDataChannel::LibDataChannel")
endif()

# 如果找到了 libvpx，链接它
if(VPX_FOUND)
  target_link_libraries(webrtc_streamer_node ${VPX_LIBRARIES})
  message(STATUS "✅ Linked libvpx: ${VPX_LIBRARIES}")
endif()

# 如果找到了 WebSocket++，链接 Boost 库
if(WEBSOCKETPP_FOUND)
  target_link_libraries(webrtc_streamer_node 
    ${Boost_LIBRARIES}
    pthread
    ssl
    crypto
  )
  message(STATUS "✅ Linked WebSocket++ dependencies: Boost, OpenSSL")
endif()

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



