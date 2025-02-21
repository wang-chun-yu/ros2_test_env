FROM ros_humble:latest

# 安装必要的包
RUN apt-get install -y passwd

# 创建新用户和用户组
RUN groupadd -r chunyu && useradd -r -g chunyu chunyu

# 创建工作目录并设置权限
RUN mkdir -p /home/chunyu && chown -R chunyu:chunyu /home/chunyu

# 设置root用户密码
RUN echo 'chunyu:21020421' | chpasswd

# 给新用户添加 sudo 权限（如果需要）
RUN usermod -aG sudo chunyu

# 切换到新创建的用户
USER chunyu

# 设置工作目录
WORKDIR /home/chunyu





