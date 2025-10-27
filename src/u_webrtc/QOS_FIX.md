# QoS 兼容性修复

## 问题描述

WebRTC 连接成功建立，但浏览器没有显示视频。

## 根本原因

**QoS 策略不匹配**

```bash
# 检查话题信息
ros2 topic info /camera/color/image_raw -v

Publisher (相机):
  Reliability: RELIABLE  ← 可靠传输

Subscription (webrtc_streamer):
  Reliability: BEST_EFFORT  ← 尽力而为传输
```

在 ROS2 中，QoS 策略必须兼容：
- ✅ RELIABLE 发布者 + RELIABLE 订阅者 = 可以通信
- ✅ BEST_EFFORT 发布者 + BEST_EFFORT/RELIABLE 订阅者 = 可以通信
- ❌ RELIABLE 发布者 + BEST_EFFORT 订阅者 = **无法通信**

## 修复方案

修改 `src/image_subscriber.cpp`，将 `SensorDataQoS()`（默认 BEST_EFFORT）改为显式的 RELIABLE QoS：

### 修改前
```cpp
_subscription = this->create_subscription<sensor_msgs::msg::Image>(
    _topicName,
    rclcpp::SensorDataQoS(),  // BEST_EFFORT
    std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1)
);
```

### 修改后
```cpp
auto qos = rclcpp::QoS(10)
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)  // 与相机匹配
    .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

_subscription = this->create_subscription<sensor_msgs::msg::Image>(
    _topicName,
    qos,
    std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1)
);
```

## 应用修复

```bash
# 1. 重新编译
cd ~/work
rm -rf build/u_webrtc install/u_webrtc
colcon build --packages-select u_webrtc
source install/setup.bash

# 2. 启动节点
ros2 launch u_webrtc webrtc_stream.launch.py

# 3. 验证 QoS 匹配
ros2 topic info /camera/color/image_raw -v
# 应该看到：
# Publisher: Reliability: RELIABLE
# Subscription: Reliability: RELIABLE  ← 现在匹配了！
```

## 验证结果

### 日志中应该看到
```
[INFO] ImageSubscriber 已创建，订阅话题: /camera/color/image_raw (RELIABLE QoS)
[INFO] WebRTC 连接正常，已处理 30 帧  ← 帧数增加！
[INFO] WebRTC 连接正常，已处理 60 帧
```

### 浏览器中应该看到
- ✅ 视频画面显示
- ✅ 图像实时更新

## 相关知识

### ROS2 QoS 策略

| 策略 | 说明 | 使用场景 |
|------|------|----------|
| RELIABLE | 保证消息传递，有重传机制 | 关键数据、控制命令 |
| BEST_EFFORT | 尽力传递，不保证，无重传 | 传感器数据流、视频流 |

### QoS 兼容矩阵

| 发布者 \ 订阅者 | RELIABLE | BEST_EFFORT |
|-----------------|----------|-------------|
| RELIABLE        | ✅       | ❌          |
| BEST_EFFORT     | ✅       | ✅          |

### 为什么相机使用 RELIABLE

大多数 ROS2 相机驱动（如 realsense、usb_cam）默认使用 RELIABLE QoS，因为：
1. 保证图像不丢失
2. 适合低频相机（10-30 fps）
3. 在局域网环境下延迟可接受

### 为什么 WebRTC 应该使用 RELIABLE

虽然 WebRTC 本身更适合 BEST_EFFORT（因为它有自己的丢包处理），但在 ROS2 中：
1. **必须与相机 QoS 匹配**才能接收数据
2. ROS2 内部通信通常在 localhost 或局域网，丢包率很低
3. RELIABLE 的额外开销可接受

## 故障排查

### 如何确认 QoS 不匹配

```bash
# 检查话题详细信息
ros2 topic info <topic_name> -v

# 查找：
# - Publisher: Reliability: RELIABLE/BEST_EFFORT
# - Subscription: Reliability: RELIABLE/BEST_EFFORT

# 如果不匹配（RELIABLE + BEST_EFFORT），就会出现：
# - 话题存在
# - 有发布者和订阅者
# - 但没有数据传输
```

### 如何测试数据流

```bash
# 查看话题频率
ros2 topic hz <topic_name>
# 如果有数据，会显示：average rate: XX.XXX

# 查看单条消息
ros2 topic echo <topic_name> --once
```

## 总结

这是一个典型的 ROS2 QoS 兼容性问题：
- ✅ WebRTC 连接成功（信令、ICE、DTLS、SRTP 都正常）
- ✅ ROS2 节点正常运行
- ❌ 但由于 QoS 不匹配，图像数据无法从相机传到 WebRTC 节点

修复后，整个流程畅通无阻！🎉

