#include <iostream>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <signal.h>
#include <string>
#include <array>
#include <filesystem>
#include <vector>

// 定义命名管道的路径
const char* FIFO_PATH = "/tmp/example_fifo";
// 定义读取进程数量
const int NUM_READERS = 3;

// 清理函数
void cleanup(int signum = 0) {
    std::filesystem::remove(FIFO_PATH);
    if (signum != 0) {
        exit(0);
    }
}

void show_fifo_explanation() {
    std::cout << R"(
命名管道（FIFO）通信示例 - 多读取者模式：
------------------------------------
1. 特点：
   - 命名管道是一种特殊的文件类型
   - 可以支持多个读取者
   - 每条消息只能被一个读取者接收
   - 读取者之间互相竞争消息

2. 工作原理：
   - 一个写入者，多个读取者
   - 写入的消息会被其中一个读取者接收
   - 采用轮询方式分发消息
   - 读取者独立运行

3. 使用场景：
   - 负载均衡
   - 任务分发
   - 多进程数据处理

4. 注意事项：
   - 需要正确处理文件权限
   - 要注意清理命名管道文件
   - 多个读取者的同步问题
)" << std::endl;
}

void writer_process() {
    std::cout << "写入进程启动..." << std::endl;

    // 以写入方式打开命名管道
    int fd = open(FIFO_PATH, O_WRONLY);
    if (fd == -1) {
        std::cerr << "打开命名管道失败" << std::endl;
        return;
    }

    // 写入数据
    for (int i = 0; i < 10; ++i) {  // 增加消息数量，让每个读取者都有机会接收到消息
        std::string message = "消息 " + std::to_string(i+1) + " 从写入进程发送\n";
        std::cout << "写入: " << message;
        write(fd, message.c_str(), message.length());
        sleep(1);  // 降低发送速度，便于观察
    }

    std::cout << "写入进程结束" << std::endl;
    close(fd);
}

void reader_process(int reader_id) {
    std::cout << "读取进程 " << reader_id << " 启动..." << std::endl;

    // 以读取方式打开命名管道
    int fd = open(FIFO_PATH, O_RDONLY);
    if (fd == -1) {
        std::cerr << "读取进程 " << reader_id << " 打开命名管道失败" << std::endl;
        return;
    }

    // 读取数据
    std::array<char, 1024> buffer;
    ssize_t bytes_read;
    while ((bytes_read = read(fd, buffer.data(), buffer.size())) > 0) {
        std::cout << "读取进程 " << reader_id << " 读取: " 
                  << std::string(buffer.data(), bytes_read);
    }

    std::cout << "读取进程 " << reader_id << " 结束" << std::endl;
    close(fd);
}

int main() {
    show_fifo_explanation();
    std::cout << "\n开始命名管道通信示例（多读取者）：" << std::endl;
    std::cout << "--------------------------------" << std::endl;

    // 注册信号处理函数
    signal(SIGINT, cleanup);

    // 创建命名管道
    cleanup();  // 确保开始时是干净的
    if (mkfifo(FIFO_PATH, 0666) == -1) {
        std::cerr << "创建命名管道失败" << std::endl;
        return 1;
    }

    // 存储子进程ID
    std::vector<pid_t> child_pids;

    // 创建多个读取者进程
    for (int i = 0; i < NUM_READERS; ++i) {
        pid_t pid = fork();

        if (pid == -1) {
            std::cerr << "创建子进程失败" << std::endl;
            cleanup();
            return 1;
        }

        if (pid == 0) {  // 子进程
            reader_process(i + 1);
            return 0;
        } else {  // 父进程
            child_pids.push_back(pid);
        }
    }

    // 父进程作为写入者
    writer_process();

    // 等待所有子进程结束
    for (pid_t pid : child_pids) {
        int status;
        waitpid(pid, &status, 0);
    }

    // 清理命名管道
    cleanup();

    return 0;
} 