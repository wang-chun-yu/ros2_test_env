#!/usr/bin/env python3
"""
简单的 WebSocket 信令服务器示例
用于 WebRTC 信令交换
"""

import asyncio
import json
import logging
from typing import Set
import websockets
from websockets.server import WebSocketServerProtocol

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 存储所有连接的客户端
clients: Set[WebSocketServerProtocol] = set()


async def handle_client(websocket: WebSocketServerProtocol, path: str):
    """处理客户端连接"""
    # 添加客户端到集合
    clients.add(websocket)
    client_addr = websocket.remote_address
    logger.info(f"新客户端连接: {client_addr}, 当前客户端数: {len(clients)}")
    
    try:
        async for message in websocket:
            try:
                # 解析 JSON 消息
                data = json.loads(message)
                msg_type = data.get('type', 'unknown')
                
                logger.info(f"收到来自 {client_addr} 的消息: {msg_type}")
                
                # 转发消息给所有其他客户端
                if len(clients) > 1:
                    # 广播给除了发送者之外的所有客户端
                    await asyncio.gather(
                        *[client.send(message) 
                          for client in clients 
                          if client != websocket],
                        return_exceptions=True
                    )
                    logger.info(f"消息已转发给 {len(clients) - 1} 个客户端")
                else:
                    logger.warning("没有其他客户端可以转发消息")
                
            except json.JSONDecodeError as e:
                logger.error(f"JSON 解析错误: {e}")
                await websocket.send(json.dumps({
                    'type': 'error',
                    'message': 'Invalid JSON format'
                }))
            except Exception as e:
                logger.error(f"处理消息时出错: {e}")
                
    except websockets.exceptions.ConnectionClosed:
        logger.info(f"客户端 {client_addr} 断开连接")
    finally:
        # 从集合中移除客户端
        clients.discard(websocket)
        logger.info(f"客户端已移除，当前客户端数: {len(clients)}")


async def main():
    """启动 WebSocket 服务器"""
    host = "0.0.0.0"
    port = 8080
    
    logger.info("=" * 50)
    logger.info("WebRTC 信令服务器启动中...")
    logger.info(f"监听地址: ws://{host}:{port}")
    logger.info("=" * 50)
    
    async with websockets.serve(handle_client, host, port):
        logger.info("服务器已启动，等待客户端连接...")
        await asyncio.Future()  # 永远运行


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("\n服务器已停止")



