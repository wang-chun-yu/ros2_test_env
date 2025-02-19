# 使用 ROS 2 Humble 官方基础镜像
FROM osrf/ros:humble-desktop

# 设置环境变量
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble
ENV ROS_PYTHON_VERSION 3

RUN apt-get clean && \
    apt-get autoclean

# COPY ../apt/sources.list /etc/apt/

# 添加缺失的 GPG 公钥 (可能需要)
# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 3B4FE6ACC0B21F32

# 更新包管理器并安装必要的开发工具和依赖
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    vim \
    git \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# 安装额外的 Python 包（如果需要）
# RUN pip3 install --upgrade pip -i https://pypi.tuna.tsinghua.edu.cn/simple
# RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple numpy pandas

# 启动时进入 bash
CMD ["/bin/bash"]





