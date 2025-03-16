# 终端、会话、进程组、信号的关系
| 概念 | 作用   |
|----|------|
| 终端（Terminal） | 用户与系统交互的窗口（如 tty 或 pts），管理会话。  |
| 会话（Session）  | 由 Shell 创建，包含多个进程组，绑定到一个终端。  | 
| 进程组（Process Group）  | 由多个进程组成，可同时管理多个进程（前台/后台）。  | 
| 信号（Signals）| Ctrl+C 影响前台进程组，kill 可终止整个进程组。 |
它们的关系如下：
- 一个终端关联一个会话
- 一个会话可以包含多个进程组
- 一个进程组包含多个进程
# 终端
- 查看终端设备
![Alt text](<2025-03-17 00-05-42屏幕截图.png>)
- 终端的作用
- 负责接收用户输入并显示输出
- 维护会话，用户组织多个进程
- 提供控制功能（如信号SIGINT由Ctrl+C触发）
# 会话
- 查看当前会话
![Alt text](<2025-03-17 00-35-08屏幕截图.png>)
显示同个sid号的都属于同个会话
![Alt text](<2025-03-17 00-40-09屏幕截图.png>)
- 会话（Session）是一个进程集合，它们共享同一个会话 ID（SID, Session ID），通常由Shell 启动时创建。
- 会话的首个进程称为会话首进程（Session Leader），通常是 Shell。
- 终端关联到一个会话，会话中的进程共享该终端。
# 进程组
## 概念
进程组是一组相关的进程，它们共享一个进程组 ID（PGID, Process Group ID）。
- 进程组的首个进程称为进程组领导者（Process Group Leader）。
- 进程组通常由 Shell 创建，用于管理前台和后台任务。
# 补充：Shell
## 什么是shell
Shell 是用户与 Linux 内核交互的命令行解释器，负责解析和执行用户输入的命令。它既是一个命令行环境，又是一种编程语言，支持变量、控制流、函数等特性。
- 常见的shell的类型
- 查看当前使用的shell
```
echo $SHELL
```
## shell进程与环境
- shell进程的启动
Shell 作为一个进程，可以由以下方式启动：
    - 用户登录后，/etc/passwd 指定的默认 Shell 被加载。
    - 终端仿真器（如 gnome-terminal）启动时，会启动一个新的 Shell 进程。
    - 运行 bash、sh 等命令可以启动新的 Shell 实例。
查看当前Shell进程ID：
```
echo $$
```
- 环境变量
Shell 维护一组环境变量，用于存储系统信息、用户配置等。
    - 查看所有环境变量
    ```
    env
    ```
    - 查看某个环境变量
    ```
    echo $PATH
    ```
    - 设置环境变量（仅当前会话有效）
    ```
    MY_VAR="hello"
    echo $MY_VAR
    ```
    - 导出环境变量（子进程可用）
    ```
    export MY_VAR="hello"
    ```
    - 使变量永久生效（添加到 ~/.bashrc 或 ~/.bash_profile）
    ```
    echo 'export MY_VAR="hello"' >> ~/.bashrc

    ```
## Shell配置文件
## Shell脚本基础
## Shell进程管理