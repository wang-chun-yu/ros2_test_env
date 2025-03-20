#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <string>
#include <cstring>
#include <array>

void show_pipe_explanation() {
    std::cout << R"(
管道通信说明：
-------------
1. 特点：
   - 管道是最基本的IPC机制之一
   - 管道是单向的，只能在一个方向上传输数据
   - 管道只能用于有亲缘关系的进程（如父子进程）间通信

2. 工作原理：
   - pipe()系统调用创建一对文件描述符
   - 一个用于读取(fd[0])，一个用于写入(fd[1])
   - 数据以字节流的形式在管道中传输
   - 管道是基于内核实现的缓冲区

3. 使用场景：
   - 父进程需要与子进程交换数据
   - 需要进行单向数据传输
   - 数据量较小且为流式传输

4. 注意事项：
   - 必须正确关闭不使用的管道端
   - 管道缓冲区大小有限
   - 需要考虑阻塞情况
)" << std::endl;
}

int main() {
    show_pipe_explanation();
    std::cout << "\n开始管道通信示例：" << std::endl;
    std::cout << "-------------------" << std::endl;

    // 创建管道
    int pipe_fd[2];
    if (pipe(pipe_fd) == -1) {
        std::cerr << "创建管道失败" << std::endl;
        return 1;
    }

    // 创建子进程
    pid_t pid = fork();

    if (pid == -1) {
        std::cerr << "创建子进程失败" << std::endl;
        return 1;
    }

    if (pid > 0) {  // 父进程
        // 关闭写端
        close(pipe_fd[1]);

        // 从管道读取数据
        std::array<char, 1024> buffer;
        std::cout << "父进程等待数据..." << std::endl;

        ssize_t bytes_read = read(pipe_fd[0], buffer.data(), buffer.size());
        if (bytes_read > 0) {
            std::cout << "父进程收到消息: " << std::string(buffer.data(), bytes_read);
        }

        // 关闭读端
        close(pipe_fd[0]);

        // 等待子进程结束
        int status;
        waitpid(pid, &status, 0);
        std::cout << "子进程已结束" << std::endl;

    } else {  // 子进程
        // 关闭读端
        close(pipe_fd[0]);

        std::cout << "子进程准备发送数据..." << std::endl;
        sleep(2);  // 模拟一些处理时间

        // 向管道写入数据
        std::string message = "你好，这是来自子进程的消息！\n";
        write(pipe_fd[1], message.c_str(), message.length());

        std::cout << "子进程发送完成" << std::endl;

        // 关闭写端
        close(pipe_fd[1]);
        return 0;
    }

    return 0;
} 