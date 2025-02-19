FROM ros_humble:latest

# 创建新用户和用户组
RUN groupadd -r chunyu && useradd -r -g chunyu chunyu

# 创建工作目录并设置权限
RUN mkdir -p /home/chunyu && chown -R chunyu:chunyu /home/chunyu

# 切换到新创建的用户
USER chunyu

# 设置工作目录
WORKDIR /home/chunyu





