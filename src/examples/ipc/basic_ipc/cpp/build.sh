#!/bin/bash

# 创建构建目录
mkdir -p build
cd build

# 运行CMake
cmake ..

# 编译
make

# 返回上级目录
cd ..

echo "编译完成。可以运行以下命令来测试示例："
echo "./build/pipe_example"
echo "./build/named_pipe_example"
echo "./build/shared_memory_example" 