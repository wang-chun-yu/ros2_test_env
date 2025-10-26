/**
 * ROS2 WebRTC 视频流客户端
 */

let ws = null;
let pc = null;
let videoElement = null;
let statsInterval = null;

// ICE 服务器配置
const iceServers = {
    iceServers: [
        { urls: 'stun:stun.l.google.com:19302' },
        { urls: 'stun:stun1.l.google.com:19302' }
    ]
};

/**
 * 页面加载完成
 */
window.onload = function() {
    videoElement = document.getElementById('videoElement');
    updateStatus('disconnected', '未连接');
};

/**
 * 连接到信令服务器
 */
async function connect() {
    const serverUrl = document.getElementById('serverUrl').value;
    
    if (!serverUrl) {
        alert('请输入服务器地址');
        return;
    }
    
    try {
        updateStatus('connecting', '连接中...');
        
        // 创建 WebSocket 连接
        ws = new WebSocket(serverUrl);
        
        ws.onopen = () => {
            console.log('WebSocket 已连接');
            updateStatus('connected', '已连接到信令服务器');
            document.getElementById('connectBtn').disabled = true;
            document.getElementById('disconnectBtn').disabled = false;
            
            // 创建 PeerConnection
            createPeerConnection();
        };
        
        ws.onmessage = async (event) => {
            const message = JSON.parse(event.data);
            await handleSignalingMessage(message);
        };
        
        ws.onerror = (error) => {
            console.error('WebSocket 错误:', error);
            updateStatus('disconnected', '连接错误');
        };
        
        ws.onclose = () => {
            console.log('WebSocket 已断开');
            updateStatus('disconnected', '已断开连接');
            document.getElementById('connectBtn').disabled = false;
            document.getElementById('disconnectBtn').disabled = true;
        };
        
    } catch (error) {
        console.error('连接失败:', error);
        updateStatus('disconnected', '连接失败: ' + error.message);
    }
}

/**
 * 断开连接
 */
function disconnect() {
    if (pc) {
        pc.close();
        pc = null;
    }
    
    if (ws) {
        ws.close();
        ws = null;
    }
    
    if (statsInterval) {
        clearInterval(statsInterval);
        statsInterval = null;
    }
    
    updateStatus('disconnected', '已断开连接');
}

/**
 * 创建 PeerConnection
 */
function createPeerConnection() {
    pc = new RTCPeerConnection(iceServers);
    
    // 处理 ICE 候选
    pc.onicecandidate = (event) => {
        if (event.candidate) {
            console.log('发送 ICE 候选');
            sendMessage({
                type: 'candidate',
                candidate: event.candidate.candidate
            });
        }
    };
    
    // 处理连接状态变化
    pc.onconnectionstatechange = () => {
        console.log('连接状态:', pc.connectionState);
        updateStatus('connected', '连接状态: ' + pc.connectionState);
    };
    
    // 处理接收到的媒体流
    pc.ontrack = (event) => {
        console.log('接收到媒体流');
        videoElement.srcObject = event.streams[0];
        
        // 开始统计
        startStats();
    };
    
    console.log('PeerConnection 已创建');
}

/**
 * 处理信令消息
 */
async function handleSignalingMessage(message) {
    console.log('收到信令消息:', message.type);
    
    try {
        if (message.type === 'offer') {
            // 设置远端描述
            await pc.setRemoteDescription(new RTCSessionDescription({
                type: 'offer',
                sdp: message.sdp
            }));
            
            // 创建 Answer
            const answer = await pc.createAnswer();
            await pc.setLocalDescription(answer);
            
            // 发送 Answer
            sendMessage({
                type: 'answer',
                sdp: answer.sdp
            });
            
            console.log('已发送 Answer');
            
        } else if (message.type === 'candidate') {
            // 添加 ICE 候选
            await pc.addIceCandidate(new RTCIceCandidate({
                candidate: message.candidate
            }));
            console.log('已添加 ICE 候选');
        }
    } catch (error) {
        console.error('处理信令消息失败:', error);
    }
}

/**
 * 发送信令消息
 */
function sendMessage(message) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(message));
    }
}

/**
 * 更新状态显示
 */
function updateStatus(state, message) {
    const statusElement = document.getElementById('status');
    statusElement.className = `status ${state}`;
    statusElement.textContent = `状态: ${message}`;
}

/**
 * 开始统计信息更新
 */
function startStats() {
    if (statsInterval) {
        clearInterval(statsInterval);
    }
    
    let frameCount = 0;
    let lastBytes = 0;
    
    statsInterval = setInterval(async () => {
        if (!pc) return;
        
        try {
            const stats = await pc.getStats();
            let inboundStats = null;
            
            stats.forEach(report => {
                if (report.type === 'inbound-rtp' && report.mediaType === 'video') {
                    inboundStats = report;
                }
            });
            
            if (inboundStats) {
                const currentBytes = inboundStats.bytesReceived || 0;
                const bitrate = ((currentBytes - lastBytes) * 8 / 1000).toFixed(2);
                lastBytes = currentBytes;
                
                frameCount = inboundStats.framesReceived || 0;
                
                document.getElementById('stats').innerHTML = `
                    接收帧数: ${frameCount}<br>
                    连接状态: ${pc.connectionState}<br>
                    比特率: ${bitrate} kbps
                `;
            }
        } catch (error) {
            console.error('获取统计信息失败:', error);
        }
    }, 1000);
}

/**
 * 页面卸载时清理资源
 */
window.onbeforeunload = () => {
    disconnect();
};



