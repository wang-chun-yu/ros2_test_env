# 使用 ROS 2 Humble 官方基础镜像
FROM osrf/ros:humble-desktop

# 设置环境变量
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble
ENV ROS_PYTHON_VERSION 3

RUN apt-get clean && \
    apt-get autoclean

# 使用阿里云镜像源加速下载
COPY apt/sources.list /etc/apt/sources.list

# 导入 ROS 2 官方软件源的 GPG 密钥并配置仓库（避免 apt-key 过期问题）
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

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

RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# 启动时进入 bash
CMD ["/bin/bash"]





