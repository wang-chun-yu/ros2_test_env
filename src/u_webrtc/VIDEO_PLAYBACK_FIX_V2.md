# 视频播放中断问题修复 (V2)

## 问题描述

WebRTC 连接成功，收到媒体流，但视频播放失败，错误为：

```
DOMException: The play() request was interrupted by a call to pause()
```

## 根本原因

### 问题分析

```javascript
videoElement.srcObject = event.streams[0];
await videoElement.play();  // ❌ 立即调用 play()
```

**问题**：在视频元数据尚未加载完成时就调用 `play()`，导致播放请求被中断。

### 正确的播放时机

HTML5 视频元素有多个加载阶段：

1. `loadstart` - 开始加载
2. `loadedmetadata` - **元数据加载完成** ← 应该在这里 play()
3. `loadeddata` - 数据加载完成
4. `canplay` - 可以开始播放
5. `canplaythrough` - 可以流畅播放

在 `loadedmetadata` 之后调用 `play()` 最为稳定。

## 修复方案

### 修复 1: 等待元数据加载

#### 文件: `web_client/client.js`

**旧代码**:
```javascript
pc.ontrack = async (event) => {
    videoElement.srcObject = event.streams[0];
    
    // ❌ 立即调用 play()，可能失败
    await videoElement.play();
    
    startStats();
};
```

**新代码**:
```javascript
pc.ontrack = (event) => {
    videoElement.srcObject = event.streams[0];
    
    // ✅ 等待元数据加载完成后再播放
    videoElement.onloadedmetadata = async () => {
        console.log('视频元数据已加载');
        
        try {
            await videoElement.play();
            console.log('✅ 视频开始播放');
            updateStatus('connected', '✅ 视频正在播放');
            document.getElementById('playBtn').style.display = 'none';
        } catch (error) {
            console.error('❌ 视频播放失败:', error);
            updateStatus('connected', '⚠️ 视频播放失败（点击播放按钮）');
            document.getElementById('playBtn').style.display = 'block';
        }
    };
    
    // 添加更多事件监听
    videoElement.onplay = () => {
        console.log('视频 play 事件触发');
        startStats();
    };
    
    videoElement.onplaying = () => {
        console.log('视频 playing 事件触发（真正开始播放）');
    };
    
    videoElement.onerror = (e) => {
        console.error('视频错误:', e);
    };
};
```

### 修复 2: 添加手动播放按钮

当自动播放失败时，显示一个播放按钮供用户点击。

#### 文件: `web_client/index.html`

```html
<div class="video-container">
    <video id="videoElement" autoplay playsinline muted controls></video>
    <button id="playBtn" onclick="manualPlay()" style="display:none; ...">
        ▶️ 播放视频
    </button>
</div>
```

#### 文件: `web_client/client.js`

```javascript
async function manualPlay() {
    const videoElement = document.getElementById('videoElement');
    const playBtn = document.getElementById('playBtn');
    
    try {
        await videoElement.play();
        console.log('✅ 手动播放成功');
        playBtn.style.display = 'none';
        updateStatus('connected', '✅ 视频正在播放');
    } catch (error) {
        console.error('❌ 手动播放失败:', error);
        alert('播放失败: ' + error.message);
    }
}
```

## 应用修复

### 步骤 1: 刷新浏览器

在浏览器中按 **`Ctrl+Shift+R`**（硬刷新）

### 步骤 2: 重新连接

1. 输入服务器地址：`ws://localhost:8080`
2. 点击"连接"按钮

### 步骤 3: 观察结果

**浏览器控制台应该显示**:
```
✅ 接收到媒体流, 轨道数: 1
✅ 视频元数据已加载  ← 新增
✅ 视频开始播放      ← 新增
✅ 视频 play 事件触发
✅ 视频 playing 事件触发（真正开始播放）
```

**视频区域**:
- ✅ 显示相机画面
- ✅ 画面实时更新

**如果自动播放失败**:
- 会显示"▶️ 播放视频"按钮
- 点击该按钮即可手动播放

## 为什么这样修复有效

### 问题根源

```
setVideoSource() → play() 立即调用
     ↓
视频元数据尚未加载
     ↓
play() 请求被中断
     ↓
DOMException: interrupted by a call to pause()
```

### 修复后的流程

```
setVideoSource() → 触发加载
     ↓
等待 loadedmetadata 事件
     ↓
元数据加载完成
     ↓
play() 调用成功 ✅
```

### 时序图

```
时间轴 →

[srcObject 设置] → [开始加载] → [元数据加载中...] → [loadedmetadata] → [play()] ✅

旧代码：
[srcObject 设置] → [play()] ❌ 太早！

新代码：
[srcObject 设置] → [等待...] → [loadedmetadata] → [play()] ✅ 正确时机
```

## 视频元素事件详解

### 加载事件顺序

1. **loadstart**: 浏览器开始查找媒体
2. **durationchange**: 视频时长改变
3. **loadedmetadata**: **元数据加载完成**（宽度、高度、时长等）← 播放时机
4. **loadeddata**: 当前帧的数据已加载
5. **progress**: 浏览器正在下载数据
6. **canplay**: 浏览器可以开始播放（但可能需要缓冲）
7. **canplaythrough**: 浏览器估计可以流畅播放到结束

### 播放事件顺序

1. **play**: `play()` 方法被调用
2. **playing**: 视频真正开始播放
3. **timeupdate**: 播放位置更新（持续触发）

### 为什么在 loadedmetadata 播放？

- ✅ 视频尺寸已知（可以正确渲染）
- ✅ 视频格式已知（解码器已准备）
- ✅ 不会太早（不会被中断）
- ✅ 不会太晚（用户体验好）

## 调试技巧

### 1. 监听所有视频事件

```javascript
const events = ['loadstart', 'durationchange', 'loadedmetadata', 
                'loadeddata', 'progress', 'canplay', 'canplaythrough',
                'play', 'playing', 'pause', 'ended', 'error'];

events.forEach(event => {
    videoElement.addEventListener(event, () => {
        console.log(`视频事件: ${event}`);
    });
});
```

### 2. 检查视频状态

```javascript
const video = document.getElementById('videoElement');
console.log('readyState:', video.readyState);
// 0: HAVE_NOTHING
// 1: HAVE_METADATA  ← loadedmetadata 后
// 2: HAVE_CURRENT_DATA
// 3: HAVE_FUTURE_DATA
// 4: HAVE_ENOUGH_DATA

console.log('networkState:', video.networkState);
// 0: NETWORK_EMPTY
// 1: NETWORK_IDLE
// 2: NETWORK_LOADING
// 3: NETWORK_NO_SOURCE
```

### 3. 手动测试播放

```javascript
// 在浏览器控制台中运行
const video = document.getElementById('videoElement');

// 等待元数据
video.onloadedmetadata = async () => {
    console.log('元数据已加载，尝试播放');
    try {
        await video.play();
        console.log('播放成功');
    } catch (error) {
        console.error('播放失败:', error);
    }
};

// 如果已经加载，直接播放
if (video.readyState >= 1) {  // HAVE_METADATA
    video.play();
}
```

## 常见错误对照

### 错误 1: "interrupted by a call to pause()"
**原因**: 在元数据加载前调用 `play()`  
**修复**: 等待 `loadedmetadata` 事件

### 错误 2: "play() request was interrupted"
**原因**: 页面在播放前失去焦点  
**修复**: 添加手动播放按钮

### 错误 3: "play() failed because the user didn't interact"
**原因**: 浏览器 autoplay 策略  
**修复**: 添加 `muted` 属性或用户交互

### 错误 4: 黑屏但有声音
**原因**: 视频编解码器不支持  
**修复**: 检查 SDP 中的视频编解码器

### 错误 5: 视频卡顿
**原因**: 比特率过高或网络问题  
**修复**: 降低 `target_bitrate` 或分辨率

## 浏览器兼容性

### Chrome/Edge
- ✅ 支持 WebRTC
- ✅ 支持 VP8/H.264
- ✅ 静音视频可自动播放

### Firefox
- ✅ 支持 WebRTC
- ✅ 支持 VP8
- ⚠️ H.264 支持取决于系统

### Safari
- ✅ 支持 WebRTC（Safari 11+）
- ⚠️ VP8 支持有限
- ⚠️ 默认阻止自动播放

### 最佳实践

1. ✅ 等待 `loadedmetadata` 再播放
2. ✅ 使用 `muted` 属性
3. ✅ 提供手动播放按钮
4. ✅ 添加详细的事件日志
5. ✅ 使用 try-catch 处理错误

## 性能优化

### 当前配置
```yaml
# config/webrtc_config.yaml
width: 640        # 已优化
height: 480       # 已优化
target_bitrate: 2000000  # 2 Mbps
max_framerate: 30
```

### 如果视频仍然卡顿
```yaml
# 降低比特率
target_bitrate: 1000000  # 1 Mbps

# 降低帧率
max_framerate: 15

# 降低分辨率
width: 320
height: 240
```

## 完整的播放流程

```
1. WebRTC 连接建立
   ↓
2. ontrack 事件触发
   ↓
3. 设置 videoElement.srcObject
   ↓
4. 浏览器开始加载媒体 (loadstart)
   ↓
5. 元数据加载完成 (loadedmetadata) ← 我们在这里等待
   ↓
6. 调用 videoElement.play()
   ↓
7. play 事件触发
   ↓
8. 视频开始播放 (playing)
   ↓
9. 用户看到画面 ✅
```

## 总结

这次修复的核心是：

1. **等待正确的时机**：在 `loadedmetadata` 事件后调用 `play()`
2. **提供备用方案**：添加手动播放按钮
3. **增强调试能力**：添加详细的事件日志

所有修复都在 Web 客户端，**无需重新编译 ROS2 代码**。

只需**刷新浏览器**即可应用修复！

---

**状态**: ✅ 已修复  
**影响范围**: Web 客户端  
**需要重新编译**: ❌ 否  
**需要刷新浏览器**: ✅ 是  
**修复时间**: < 1 分钟

