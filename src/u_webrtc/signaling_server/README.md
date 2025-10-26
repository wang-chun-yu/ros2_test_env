# WebRTC 信令服务器

这是一个简单的 WebSocket 信令服务器，用于 WebRTC 连接的信令交换。

## 安装

```bash
pip install -r requirements.txt
```

## 运行

```bash
python3 server.py
```

服务器将在 `ws://0.0.0.0:8080` 上监听。

## 功能

- 接受 WebSocket 连接
- 转发 WebRTC 信令消息（Offer, Answer, ICE Candidates）
- 支持多个客户端

## 消息格式

所有消息都是 JSON 格式：

```json
{
    "type": "offer|answer|candidate",
    "sdp": "...",  // 用于 offer/answer
    "candidate": "..."  // 用于 candidate
}
```



