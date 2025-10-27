// 调试脚本 - 在浏览器控制台中复制粘贴并运行

console.log('=== WebRTC 诊断开始 ===\n');

// 1. 检查视频元素
const video = document.getElementById('videoElement');
console.log('1. 视频元素状态:');
console.log('  srcObject:', video.srcObject);
console.log('  readyState:', video.readyState, ['HAVE_NOTHING', 'HAVE_METADATA', 'HAVE_CURRENT_DATA', 'HAVE_FUTURE_DATA', 'HAVE_ENOUGH_DATA'][video.readyState]);
console.log('  paused:', video.paused);
console.log('  currentTime:', video.currentTime);
console.log('  videoWidth:', video.videoWidth);
console.log('  videoHeight:', video.videoHeight);
console.log('');

// 2. 检查媒体流
if (video.srcObject) {
    console.log('2. 媒体流信息:');
    const tracks = video.srcObject.getTracks();
    console.log('  轨道数量:', tracks.length);
    tracks.forEach((track, idx) => {
        console.log(`  轨道 ${idx}:`, track.kind, track.label, 'enabled:', track.enabled, 'readyState:', track.readyState);
    });
    console.log('');
}

// 3. 检查 PeerConnection
if (window.pc) {
    console.log('3. PeerConnection 状态:');
    console.log('  connectionState:', window.pc.connectionState);
    console.log('  iceConnectionState:', window.pc.iceConnectionState);
    console.log('  signalingState:', window.pc.signalingState);
    console.log('');
    
    // 4. 检查 receivers
    console.log('4. Receivers 信息:');
    const receivers = window.pc.getReceivers();
    console.log('  Receiver 数量:', receivers.length);
    receivers.forEach((receiver, idx) => {
        console.log(`  Receiver ${idx}:`, receiver.track.kind, receiver.track.label);
    });
    console.log('');
    
    // 5. 获取远端 SDP
    console.log('5. 远端 SDP (Offer):');
    if (window.pc.remoteDescription) {
        console.log(window.pc.remoteDescription.sdp);
    } else {
        console.log('  无远端描述');
    }
    console.log('');
    
    // 6. 获取本地 SDP
    console.log('6. 本地 SDP (Answer):');
    if (window.pc.localDescription) {
        console.log(window.pc.localDescription.sdp);
    } else {
        console.log('  无本地描述');
    }
    console.log('');
    
    // 7. 获取详细统计
    console.log('7. 详细统计信息:');
    window.pc.getStats().then(stats => {
        stats.forEach(report => {
            if (report.type === 'inbound-rtp' && report.mediaType === 'video') {
                console.log('  Inbound RTP 视频统计:');
                console.log('    framesReceived:', report.framesReceived);
                console.log('    framesDecoded:', report.framesDecoded);
                console.log('    framesDropped:', report.framesDropped);
                console.log('    bytesReceived:', report.bytesReceived);
                console.log('    packetsReceived:', report.packetsReceived);
                console.log('    packetsLost:', report.packetsLost);
                console.log('    codec:', report.codecId);
            }
            if (report.type === 'codec' && report.mimeType && report.mimeType.includes('video')) {
                console.log('  视频编解码器:');
                console.log('    mimeType:', report.mimeType);
                console.log('    clockRate:', report.clockRate);
                console.log('    payloadType:', report.payloadType);
            }
        });
    });
} else {
    console.log('PeerConnection 未创建');
}

console.log('\n=== 诊断结束 ===');
console.log('请将以上完整输出发送给开发者');

