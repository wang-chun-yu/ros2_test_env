cmake_minimum_required(VERSION 3.10)
project(ipc_examples)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加可执行文件
add_executable(pipe_example pipe_example.cpp)
add_executable(named_pipe_example named_pipe_example.cpp)
add_executable(shared_memory_example shared_memory_example.cpp)

# 链接必要的库
target_link_libraries(pipe_example pthread)
target_link_libraries(named_pipe_example pthread)
target_link_libraries(shared_memory_example pthread rt) 