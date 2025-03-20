#include <iostream>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <string.h>
#include <signal.h>
#include <random>
#include <chrono>
#include <array>

// 共享内存名称
const char* SHM_NAME = "/example_shm";
// 共享内存大小（字节）
const size_t SHM_SIZE = 4096;

// 共享内存结构
struct SharedData {
    double data[256];
    bool updated;
};

// 清理函数
void cleanup(int signum = 0) {
    shm_unlink(SHM_NAME);
    if (signum != 0) {
        exit(0);
    }
}

void show_shm_explanation() {
    std::cout << R"(
共享内存通信说明：
---------------
1. 特点：
   - 最快的IPC机制
   - 多个进程可以直接访问同一块内存
   - 需要自行处理同步问题
   - 适合大量数据共享

2. 工作原理：
   - 在物理内存中分配一块区域
   - 将该内存区域映射到多个进程的地址空间
   - 进程可以直接读写这块内存
   - 避免了数据复制的开销

3. 使用场景：
   - 需要高性能数据共享
   - 大量数据需要在进程间传递
   - 实时数据处理

4. 注意事项：
   - 需要手动处理同步
   - 注意内存泄漏
   - 及时清理共享内存
   - 处理好进程退出时的资源释放
)" << std::endl;
}

void writer_process() {
    std::cout << "写入进程启动..." << std::endl;

    // 创建共享内存
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd == -1) {
        std::cerr << "创建共享内存失败" << std::endl;
        return;
    }

    // 设置共享内存大小
    if (ftruncate(fd, SHM_SIZE) == -1) {
        std::cerr << "设置共享内存大小失败" << std::endl;
        close(fd);
        return;
    }

    // 映射共享内存
    SharedData* shared_data = static_cast<SharedData*>(
        mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)
    );

    if (shared_data == MAP_FAILED) {
        std::cerr << "映射共享内存失败" << std::endl;
        close(fd);
        return;
    }

    // 初始化随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);

    // 写入数据
    for (int i = 0; i < 5; ++i) {
        // 生成随机数据
        double sum = 0;
        for (int j = 0; j < 256; ++j) {
            shared_data->data[j] = dis(gen);
            sum += shared_data->data[j];
        }
        double mean = sum / 256;

        shared_data->updated = true;
        std::cout << "写入进程更新数据: 均值=" << mean << std::endl;
        sleep(1);
    }

    // 清理
    munmap(shared_data, SHM_SIZE);
    close(fd);
}

void reader_process() {
    std::cout << "读取进程启动..." << std::endl;

    // 打开共享内存
    int fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (fd == -1) {
        std::cerr << "打开共享内存失败" << std::endl;
        return;
    }

    // 映射共享内存
    SharedData* shared_data = static_cast<SharedData*>(
        mmap(nullptr, SHM_SIZE, PROT_READ, MAP_SHARED, fd, 0)
    );

    if (shared_data == MAP_FAILED) {
        std::cerr << "映射共享内存失败" << std::endl;
        close(fd);
        return;
    }

    // 读取数据
    bool last_updated = false;
    while (true) {
        if (shared_data->updated != last_updated) {
            double sum = 0;
            for (int i = 0; i < 256; ++i) {
                sum += shared_data->data[i];
            }
            double mean = sum / 256;
            std::cout << "读取进程读取数据: 均值=" << mean << std::endl;
            last_updated = shared_data->updated;
        }
        usleep(100000);  // 休眠100ms
    }

    // 清理
    munmap(shared_data, SHM_SIZE);
    close(fd);
}

int main() {
    show_shm_explanation();
    std::cout << "\n开始共享内存通信示例：" << std::endl;
    std::cout << "--------------------" << std::endl;

    // 注册信号处理函数
    signal(SIGINT, cleanup);

    // 确保开始时共享内存是干净的
    cleanup();

    // 创建子进程
    pid_t pid = fork();

    if (pid == -1) {
        std::cerr << "创建子进程失败" << std::endl;
        return 1;
    }

    try {
        if (pid > 0) {  // 父进程作为写入者
            writer_process();
            // 等待子进程结束
            int status;
            waitpid(pid, &status, 0);
            // 清理共享内存
            cleanup();
        } else {  // 子进程作为读取者
            reader_process();
            return 0;
        }
    } catch (const std::exception& e) {
        std::cerr << "发生错误: " << e.what() << std::endl;
        cleanup();
        return 1;
    }

    return 0;
} 