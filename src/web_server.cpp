#include "web_server.h"


// 全局变量
AsyncWebServer *server = nullptr;
WebServerMode webserverMode = WS_MODE_OFF;

// WiFi配置（与main.cpp保持一致）
const char *ssid = "OpenWIFI2.4G@320";
const char *password = "SUshe320";

// 主页HTML（简化版）
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-C3 NFC门禁</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            margin: 50px 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 600px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #2E86C1;
            margin-bottom: 30px;
        }
        .status {
            padding: 15px;
            margin: 20px 0;
            border-radius: 5px;
            background-color: #D4E6F1;
            font-size: 16px;
        }
        .status.active {
            background-color: #A9DFBF;
        }
        .btn-group {
            display: flex;
            flex-direction: column;
            align-items: center;
            margin: 20px 0;
        }
        .btn {
            display: inline-block;
            width: 200px;
            padding: 15px 30px;
            margin: 10px;
            text-decoration: none;
            color: white;
            background-color: #2E86C1;
            border-radius: 5px;
            font-size: 16px;
            transition: background-color 0.3s;
            cursor: pointer;
            border: none;
        }
        .btn:hover {
            background-color: #1B4F72;
        }
        .btn-danger {
            background-color: #E74C3C;
        }
        .btn-danger:hover {
            background-color: #C0392B;
        }
        .btn-success {
            background-color: #27AE60;
        }
        .btn-success:hover {
            background-color: #1E8449;
        }
        .info {
            margin-top: 20px;
            padding: 15px;
            background-color: #F8F9F9;
            border-radius: 5px;
            font-size: 14px;
            color: #666;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32-C3 NFC门禁系统</h1>
        
        <div class="status %STATUS_CLASS%">
            <strong>Web服务器状态：</strong> %STATUS_TEXT%
        </div>
        
        <div class="info">
            <strong>IP地址：</strong> <span id="ip"></span><br>
            <strong>WiFi：</strong> <span id="wifi"></span>
        </div>
        
        <div class="btn-group">
            <a href="/files" class="btn btn-success">文件管理</a>
            <a href="/servo" class="btn">舵机管理</a>
            <a href="/ota" class="btn">OTA升级</a>
            <button onclick="restartSystem()" class="btn btn-danger">重启系统</button>
        </div>
        
        <script>
            document.getElementById('ip').innerHTML = window.location.hostname;
            
            function restartSystem() {
                if (confirm('确定要重启系统吗？\n\n重启后将回到正常门禁模式，Web服务器将停止运行。')) {
                    fetch('/restart', { method: 'POST' })
                        .then(response => {
                            if (response.ok) {
                                alert('系统正在重启，请稍候...');
                            } else {
                                alert('重启失败，请重试');
                            }
                        })
                        .catch(error => {
                            alert('请求失败：' + error);
                        });
                }
            }
        </script>
    </div>
</body>
</html>
)rawliteral";

// OTA上传页面HTML
const char ota_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>OTA 固件升级</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            margin: 50px 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 600px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #2E86C1;
        }
        .upload-box {
            border: 2px dashed #2E86C1;
            padding: 30px;
            margin: 20px 0;
            border-radius: 5px;
        }
        .progress-container {
            width: 100%;
            background-color: #ddd;
            border-radius: 5px;
            margin: 20px 0;
            display: none;
        }
        .progress-bar {
            width: 0%;
            height: 30px;
            background-color: #4CAF50;
            border-radius: 5px;
            text-align: center;
            line-height: 30px;
            color: white;
            transition: width 0.3s;
        }
        .status {
            padding: 15px;
            margin: 20px 0;
            border-radius: 5px;
            display: none;
        }
        .status.success {
            background-color: #A9DFBF;
            color: #1D8348;
        }
        .status.error {
            background-color: #F1948A;
            color: #B03A2E;
        }
        .status.info {
            background-color: #D4E6F1;
            color: #1B4F72;
        }
        .btn {
            padding: 12px 24px;
            margin: 10px;
            color: white;
            background-color: #2E86C1;
            border: none;
            border-radius: 5px;
            font-size: 16px;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        .btn:hover {
            background-color: #1B4F72;
        }
        .btn:disabled {
            background-color: #ccc;
            cursor: not-allowed;
        }
        .back-link {
            display: block;
            margin-top: 20px;
            color: #2E86C1;
            text-decoration: none;
        }
        .back-link:hover {
            text-decoration: underline;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>OTA 固件升级</h1>
        
        <div class="upload-box">
            <p style="margin-bottom: 20px;">请选择 .bin 固件文件上传</p>
            <input type="file" id="file" accept=".bin">
        </div>
        
        <button id="uploadBtn" class="btn" onclick="uploadFile()">上传固件</button>
        
        <div class="progress-container" id="progressContainer">
            <div class="progress-bar" id="progressBar">0%</div>
        </div>
        
        <div class="status info" id="statusInfo">
            准备就绪
        </div>
        
        <div class="status success" id="statusSuccess">
            <strong>✅ 上传成功！</strong><br>
            设备将自动重启...
        </div>
        
        <div class="status error" id="statusError">
            <strong>❌ 上传失败！</strong><br>
            <span id="errorMsg"></span>
        </div>
        
        <a href="/" class="back-link">← 返回主页</a>
    </div>
    
    <script>
        let uploadInProgress = false;
        
        function uploadFile() {
            const fileInput = document.getElementById('file');
            const file = fileInput.files[0];
            
            if (!file) {
                showStatus('error', '请先选择一个固件文件');
                return;
            }
            
            if (!file.name.endsWith('.bin')) {
                showStatus('error', '请选择 .bin 格式的固件文件');
                return;
            }
            
            if (uploadInProgress) return;
            uploadInProgress = true;
            
            const uploadBtn = document.getElementById('uploadBtn');
            uploadBtn.disabled = true;
            
            showStatus('info', '正在上传，请稍候...');
            document.getElementById('progressContainer').style.display = 'block';
            
            const xhr = new XMLHttpRequest();
            const formData = new FormData();
            formData.append('file', file);
            
            xhr.upload.onprogress = function(e) {
                if (e.lengthComputable) {
                    const percent = Math.round((e.loaded / e.total) * 100);
                    document.getElementById('progressBar').style.width = percent + '%';
                    document.getElementById('progressBar').textContent = percent + '%';
                }
            };
            
            xhr.onload = function() {
                if (xhr.status === 200) {
                    showStatus('success', '');
                    document.getElementById('progressBar').style.width = '100%';
                    document.getElementById('progressBar').textContent = '100%';
                } else {
                    showStatus('error', '服务器错误: ' + xhr.status);
                }
                uploadBtn.disabled = false;
                uploadInProgress = false;
            };
            
            xhr.onerror = function() {
                showStatus('error', '网络错误，请重试');
                uploadBtn.disabled = false;
                uploadInProgress = false;
            };
            
            xhr.open('POST', '/update', true);
            xhr.send(formData);
        }
        
        function showStatus(type, message) {
            document.getElementById('statusInfo').style.display = type === 'info' ? 'block' : 'none';
            document.getElementById('statusSuccess').style.display = type === 'success' ? 'block' : 'none';
            document.getElementById('statusError').style.display = type === 'error' ? 'block' : 'none';
            
            if (type === 'error' && message) {
                document.getElementById('errorMsg').textContent = message;
            }
        }
    </script>
</body>
</html>
)rawliteral";

// 文件管理页面HTML（支持目录导航，无表情符号）
const char files_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>文件管理 - LittleFS</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            margin: 50px 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #2E86C1;
            margin-bottom: 30px;
        }
        .path-bar {
            display: flex;
            align-items: center;
            margin: 20px 0;
            padding: 10px;
            background-color: #f0f0f0;
            border-radius: 5px;
        }
        .path-bar span {
            margin-right: 10px;
        }
        .path-bar input {
            flex: 1;
            padding: 8px;
            border: 1px solid #ddd;
            border-radius: 3px;
            background-color: white;
        }
        .btn {
            padding: 8px 16px;
            margin-left: 10px;
            color: white;
            background-color: #2E86C1;
            border: none;
            border-radius: 5px;
            font-size: 14px;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        .btn:hover {
            background-color: #1B4F72;
        }
        .btn-sm {
            padding: 4px 8px;
            font-size: 12px;
            margin-left: 5px;
            background-color: #E74C3C;
        }
        .btn-sm:hover {
            background-color: #C0392B;
        }
        .upload-section {
            border: 2px dashed #2E86C1;
            padding: 20px;
            margin: 20px 0;
            border-radius: 5px;
        }
        .file-list {
            margin-top: 30px;
            text-align: left;
        }
        .item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 12px;
            margin: 5px 0;
            background-color: #F8F9F9;
            border-radius: 5px;
            border-left: 4px solid #2E86C1;
            cursor: pointer;
        }
        .item.directory {
            border-left-color: #27AE60;
            font-weight: bold;
        }
        .item:hover {
            background-color: #e9e9e9;
        }
        .item .name {
            flex: 1;
        }
        .item .size {
            margin-right: 20px;
            color: #666;
            font-size: 14px;
        }
        .status {
            padding: 15px;
            margin: 20px 0;
            border-radius: 5px;
            display: none;
        }
        .status.success {
            background-color: #A9DFBF;
            color: #1D8348;
        }
        .status.error {
            background-color: #F1948A;
            color: #B03A2E;
        }
        .status.info {
            background-color: #D4E6F1;
            color: #1B4F72;
        }
        .back-link {
            display: block;
            margin-top: 20px;
            color: #2E86C1;
            text-decoration: none;
        }
        .back-link:hover {
            text-decoration: underline;
        }
        .empty {
            text-align: center;
            color: #999;
            padding: 30px;
        }
        .storage-info {
            margin: 20px 0;
            padding: 15px;
            background-color: #F8F9F9;
            border-radius: 5px;
            text-align: left;
        }
        .storage-info .info-text {
            margin-bottom: 10px;
            font-size: 14px;
            color: #333;
        }
        .storage-info .progress-container {
            width: 100%;
            height: 20px;
            background-color: #ddd;
            border-radius: 10px;
            overflow: hidden;
        }
        .storage-info .progress-bar {
            height: 100%;
            background-color: #4CAF50;
            width: 0%;
            transition: width 0.5s ease;
            border-radius: 10px;
        }
        .storage-info .progress-bar.warning {
            background-color: #FFC107;
        }
        .storage-info .progress-bar.danger {
            background-color: #F44336;
        }
        .storage-info .progress-text {
            margin-top: 5px;
            font-size: 12px;
            color: #666;
            text-align: right;
        }
        .icon {
            display: inline-block;
            vertical-align: middle;
            margin-right: 8px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>LittleFS 文件管理</h1>

        <div class="path-bar">
            <span>当前路径:</span>
            <input type="text" id="currentPath" value="/" readonly>
            <button class="btn" onclick="goToParent()">上级目录</button>
            <button class="btn" onclick="createDir()">新建文件夹</button>
        </div>

        <div class="storage-info">
            <div class="info-text">已用: <span id="usedStorage">0</span> KB / 总计: <span id="totalStorage">0</span> KB (剩余: <span id="freeStoragePercent">0</span>%)</div>
            <div class="progress-container">
                <div class="progress-bar" id="storageProgressBar"></div>
            </div>
            <div class="progress-text">剩余: <span id="freeStorage">0</span> KB</div>
        </div>

        <div class="upload-section">
            <p>上传文件到当前目录</p>
            <input type="file" id="file" onchange="validateFile()">
            <button class="btn" onclick="uploadFile()" id="uploadBtn">上传</button>
        </div>

        <div id="status" class="status info" style="display:none;"></div>

        <div class="file-list">
            <div id="fileListContainer"></div>
        </div>

        <a href="/" class="back-link">← 返回主页</a>
    </div>

    <script>
        let currentPath = '/';

        window.onload = function() {
            loadList(currentPath);
            loadStorageInfo();
        };

        // SVG图标
        const dirIcon = `<svg class="icon" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#27AE60" stroke-width="2"><path d="M22 19a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h5l2 3h9a2 2 0 0 1 2 2z"></path></svg>`;
        const fileIcon = `<svg class="icon" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#2E86C1" stroke-width="2"><path d="M13 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V9z"></path><polyline points="13 2 13 9 20 9"></polyline></svg>`;

        // 加载存储信息
        function loadStorageInfo() {
            fetch('/api/fs/info')
                .then(response => {
                    if (!response.ok) throw new Error('HTTP ' + response.status);
                    return response.json();
                })
                .then(data => {
                    document.getElementById('usedStorage').textContent = data.used;
                    document.getElementById('totalStorage').textContent = data.total;
                    document.getElementById('freeStorage').textContent = data.free;
                    document.getElementById('freeStoragePercent').textContent = data.freePercent;
                    
                    const progressBar = document.getElementById('storageProgressBar');
                    progressBar.style.width = (100 - data.freePercent) + '%';
                    
                    // 根据剩余空间设置颜色
                    progressBar.className = 'progress-bar';
                    if (data.freePercent < 20) {
                        progressBar.classList.add('danger');
                    } else if (data.freePercent < 50) {
                        progressBar.classList.add('warning');
                    }
                })
                .catch(err => {
                    console.error('加载存储信息失败:', err);
                });
        }

        // 验证文件大小
        function validateFile() {
            const fileInput = document.getElementById('file');
            const uploadBtn = document.getElementById('uploadBtn');
            const file = fileInput.files[0];
            
            if (!file) {
                uploadBtn.disabled = false;
                return;
            }
            
            // 重新加载存储信息
            loadStorageInfo();
            
            // 获取当前剩余空间
            fetch('/api/fs/info')
                .then(response => {
                    if (!response.ok) throw new Error('HTTP ' + response.status);
                    return response.json();
                })
                .then(data => {
                    const fileSizeKB = Math.ceil(file.size / 1024);
                    const freeSpaceKB = data.free;
                    
                    if (fileSizeKB > freeSpaceKB) {
                        alert('存储空间不足！\\n\\n文件大小: ' + fileSizeKB + ' KB\\n剩余空间: ' + freeSpaceKB + ' KB\\n\\n请删除一些文件后再试。');
                        fileInput.value = '';
                        uploadBtn.disabled = true;
                    } else {
                        uploadBtn.disabled = false;
                    }
                })
                .catch(err => {
                    console.error('验证文件失败:', err);
                    uploadBtn.disabled = false;
                });
        }

        function showStatus(msg, type) {
            const statusDiv = document.getElementById('status');
            statusDiv.className = 'status ' + type;
            statusDiv.innerHTML = msg;
            statusDiv.style.display = 'block';
            setTimeout(() => statusDiv.style.display = 'none', 3000);
        }

        function loadList(path) {
            fetch('/list?path=' + encodeURIComponent(path))
                .then(response => {
                    if (!response.ok) {
                        throw new Error('HTTP ' + response.status);
                    }
                    return response.json();
                })
                .then(data => {
                    currentPath = path;
                    document.getElementById('currentPath').value = path;
                    renderList(data);
                })
                .catch(err => {
                    showStatus('加载失败: ' + err.message, 'error');
                });
        }

        function renderList(items) {
            const container = document.getElementById('fileListContainer');
            if (!items || items.length === 0) {
                container.innerHTML = '<div class="empty">文件夹为空</div>';
                return;
            }

            let html = '';
            items.forEach(item => {
                const isDir = item.isDirectory;
                const icon = isDir ? dirIcon : fileIcon;
                const sizeStr = isDir ? '<DIR>' : formatFileSize(item.size);
                // 双击目录时进入
                const ondblclick = isDir ? `ondblclick="navigate('${item.name}')"` : '';
                html += `
                    <div class="item ${isDir ? 'directory' : ''}" ${ondblclick}>
                        <span class="name">${icon} ${item.name}</span>
                        <span class="size">${sizeStr}</span>
                        <div>
                            ${isDir ? '' : '<button class="btn-sm" onclick="event.stopPropagation(); deleteItem(\'' + item.name + '\', false)">删除</button>'}
                            ${isDir ? '<button class="btn-sm" onclick="event.stopPropagation(); deleteItem(\'' + item.name + '\', true)">删除目录</button>' : ''}
                        </div>
                    </div>
                `;
            });
            container.innerHTML = html;
        }

        function formatFileSize(bytes) {
            if (bytes < 1024) return bytes + ' B';
            if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(2) + ' KB';
            return (bytes / (1024 * 1024)).toFixed(2) + ' MB';
        }

        function navigate(name) {
            let newPath = currentPath === '/' ? '/' + name : currentPath + '/' + name;
            // 规范化路径：去除多余的斜杠
            newPath = newPath.replace(/\/+/g, '/');
            loadList(newPath);
        }

        function goToParent() {
            if (currentPath === '/') return;
            let parts = currentPath.split('/');
            parts.pop();
            let parent = parts.join('/');
            if (parent === '') parent = '/';
            loadList(parent);
        }

        function createDir() {
            let dirName = prompt('请输入新文件夹名称:');
            if (!dirName) return;
            // 禁止包含路径分隔符
            if (dirName.includes('/') || dirName.includes('\\')) {
                alert('文件夹名称不能包含 / 或 \\');
                return;
            }
            let fullPath = currentPath === '/' ? '/' + dirName : currentPath + '/' + dirName;
            fullPath = fullPath.replace(/\/+/g, '/');

            fetch('/mkdir', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: 'path=' + encodeURIComponent(fullPath)
            })
            .then(response => {
                if (response.ok) {
                    showStatus('文件夹创建成功', 'success');
                    loadList(currentPath);
                } else {
                    showStatus('创建失败', 'error');
                }
            })
            .catch(err => showStatus('请求失败: ' + err.message, 'error'));
        }

        function uploadFile() {
            const fileInput = document.getElementById('file');
            const file = fileInput.files[0];
            if (!file) {
                alert('请选择文件');
                return;
            }

            const formData = new FormData();
            formData.append('file', file);
            formData.append('path', currentPath);  // 传递当前路径

            fetch('/upload', {
                method: 'POST',
                body: formData
            })
            .then(response => {
                if (response.ok) {
                    showStatus('上传成功', 'success');
                    fileInput.value = '';
                    loadList(currentPath);
                    loadStorageInfo();  // 刷新存储信息
                } else {
                    showStatus('上传失败', 'error');
                }
            })
            .catch(err => showStatus('上传出错: ' + err.message, 'error'));
        }

        function deleteItem(name, isDir) {
            let fullPath = currentPath === '/' ? '/' + name : currentPath + '/' + name;
            fullPath = fullPath.replace(/\/+/g, '/');
            if (!confirm(`确定删除 ${fullPath} 吗？`)) return;

            fetch('/delete', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: 'path=' + encodeURIComponent(fullPath)
            })
            .then(response => {
                if (response.ok) {
                    showStatus('删除成功', 'success');
                    loadList(currentPath);
                    loadStorageInfo();  // 刷新存储信息
                } else if (response.status === 409) {
                    alert('目录不为空，无法删除');
                } else {
                    showStatus('删除失败', 'error');
                }
            })
            .catch(err => showStatus('请求失败: ' + err.message, 'error'));
        }
    </script>
</body>
</html>
)rawliteral";

// 舵机管理页面HTML
const char servo_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>舵机管理</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            margin: 50px 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 600px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #2E86C1;
            margin-bottom: 30px;
        }
        .section {
            margin: 20px 0;
            padding: 20px;
            background-color: #F8F9F9;
            border-radius: 5px;
        }
        .section h2 {
            color: #2E86C1;
            font-size: 18px;
            margin-top: 0;
            margin-bottom: 20px;
        }
        .input-group {
            margin: 15px 0;
            text-align: left;
        }
        .input-group label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
            color: #333;
        }
        .input-group input {
            width: 100%;
            padding: 10px;
            border: 1px solid #ddd;
            border-radius: 5px;
            font-size: 16px;
            box-sizing: border-box;
        }
        .input-group .range-info {
            font-size: 12px;
            color: #666;
            margin-top: 5px;
        }
        .btn-group {
            display: flex;
            gap: 10px;
            margin: 20px 0;
        }
        .btn {
            flex: 1;
            padding: 12px 20px;
            color: white;
            background-color: #2E86C1;
            border: none;
            border-radius: 5px;
            font-size: 16px;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        .btn:hover {
            background-color: #1B4F72;
        }
        .btn:disabled {
            background-color: #ccc;
            cursor: not-allowed;
        }
        .btn-success {
            background-color: #27AE60;
        }
        .btn-success:hover {
            background-color: #1E8449;
        }
        .btn-danger {
            background-color: #E74C3C;
        }
        .btn-danger:hover {
            background-color: #C0392B;
        }
        .status {
            padding: 15px;
            margin: 20px 0;
            border-radius: 5px;
            display: none;
        }
        .status.success {
            background-color: #A9DFBF;
            color: #1D8348;
        }
        .status.error {
            background-color: #F1948A;
            color: #B03A2E;
        }
        .status.info {
            background-color: #D4E6F1;
            color: #1B4F72;
        }
        .back-link {
            display: block;
            margin-top: 20px;
            color: #2E86C1;
            text-decoration: none;
        }
        .back-link:hover {
            text-decoration: underline;
        }
        .servo-status {
            font-size: 18px;
            margin: 10px 0;
            padding: 10px;
            border-radius: 5px;
        }
        .servo-status.idle {
            background-color: #D5F5E3;
            color: #1D8348;
        }
        .servo-status.busy {
            background-color: #FADBD8;
            color: #B03A2E;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>舵机管理</h1>
        
        <!-- 配置区域 -->
        <div class="section">
            <h2>位置配置</h2>
            <div class="input-group">
                <label for="unlockPos">解锁位置 (0-4095)</label>
                <input type="number" id="unlockPos" min="0" max="4095" value="800">
                <div class="range-info">舵机解锁时的位置值</div>
            </div>
            <div class="input-group">
                <label for="lockPos">锁定位置 (0-4095)</label>
                <input type="number" id="lockPos" min="0" max="4095" value="1180">
                <div class="range-info">舵机锁定时的位置值</div>
            </div>
            <div class="btn-group">
                <button class="btn btn-success" onclick="saveConfig()">保存配置</button>
                <button class="btn" onclick="loadConfig()">重新加载</button>
            </div>
        </div>
        
        <!-- 控制区域 -->
        <div class="section">
            <h2>动作控制</h2>
            <div class="servo-status idle" id="servoStatus">状态: 空闲</div>
            <div class="btn-group">
                <button class="btn btn-success" onclick="unlock()" id="unlockBtn">解锁</button>
                <button class="btn btn-danger" onclick="lock()" id="lockBtn">锁定</button>
            </div>
        </div>
        
        <div id="status" class="status info"></div>
        
        <a href="/" class="back-link">← 返回主页</a>
    </div>
    
    <script>
        let actionInProgress = false;
        
        // 页面加载时获取配置
        window.onload = function() {
            loadConfig();
        };
        
        function showStatus(msg, type) {
            const statusDiv = document.getElementById('status');
            statusDiv.className = 'status ' + type;
            statusDiv.innerHTML = msg;
            statusDiv.style.display = 'block';
            setTimeout(() => statusDiv.style.display = 'none', 3000);
        }
        
        function updateServoStatus(busy) {
            const statusDiv = document.getElementById('servoStatus');
            const unlockBtn = document.getElementById('unlockBtn');
            const lockBtn = document.getElementById('lockBtn');
            
            if (busy) {
                statusDiv.className = 'servo-status busy';
                statusDiv.innerHTML = '状态: 执行中...';
                unlockBtn.disabled = true;
                lockBtn.disabled = true;
            } else {
                statusDiv.className = 'servo-status idle';
                statusDiv.innerHTML = '状态: 空闲';
                unlockBtn.disabled = false;
                lockBtn.disabled = false;
            }
        }
        
        // 加载配置
        function loadConfig() {
            fetch('/api/servo/config')
                .then(response => {
                    if (!response.ok) throw new Error('HTTP ' + response.status);
                    return response.json();
                })
                .then(data => {
                    document.getElementById('unlockPos').value = data.unlock;
                    document.getElementById('lockPos').value = data.lock;
                    showStatus('配置加载成功', 'success');
                })
                .catch(err => showStatus('加载配置失败: ' + err.message, 'error'));
        }
        
        // 保存配置
        function saveConfig() {
            const unlock = parseInt(document.getElementById('unlockPos').value);
            const lock = parseInt(document.getElementById('lockPos').value);
            
            if (isNaN(unlock) || isNaN(lock)) {
                showStatus('请输入有效的位置值', 'error');
                return;
            }
            
            if (unlock < 0 || unlock > 4095 || lock < 0 || lock > 4095) {
                showStatus('位置值必须在0-4095范围内', 'error');
                return;
            }
            
            fetch('/api/servo/config', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ unlock: unlock, lock: lock })
            })
            .then(response => {
                if (response.ok) {
                    showStatus('配置保存成功', 'success');
                } else {
                    showStatus('保存配置失败', 'error');
                }
            })
            .catch(err => showStatus('请求失败: ' + err.message, 'error'));
        }
        
        // 解锁
        function unlock() {
            if (actionInProgress) return;
            actionInProgress = true;
            updateServoStatus(true);
            
            fetch('/api/servo/unlock', { method: 'POST' })
                .then(response => {
                    if (response.ok) {
                        showStatus('解锁指令已发送', 'success');
                    } else {
                        showStatus('解锁失败', 'error');
                    }
                })
                .catch(err => showStatus('请求失败: ' + err.message, 'error'))
                .finally(() => {
                    setTimeout(() => {
                        actionInProgress = false;
                        updateServoStatus(false);
                    }, 1500);
                });
        }
        
        // 锁定
        function lock() {
            if (actionInProgress) return;
            actionInProgress = true;
            updateServoStatus(true);
            
            fetch('/api/servo/lock', { method: 'POST' })
                .then(response => {
                    if (response.ok) {
                        showStatus('锁定指令已发送', 'success');
                    } else {
                        showStatus('锁定失败', 'error');
                    }
                })
                .catch(err => showStatus('请求失败: ' + err.message, 'error'))
                .finally(() => {
                    setTimeout(() => {
                        actionInProgress = false;
                        updateServoStatus(false);
                    }, 1500);
                });
        }
    </script>
</body>
</html>
)rawliteral";

// 静态变量用于文件上传处理
static File uploadFile;
static String uploadPath;
static uint64_t uploadFileSize;

// 初始化Web服务器
void initWebServer()
{
    if (server != nullptr)
    {
        // 服务器已存在，不需要重新初始化
        return;
    }

    Serial.println("\n\n=== 初始化Web服务器 ===");

    // 检查WiFi连接状态
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("正在连接WiFi: ");
        Serial.println(ssid);
        WiFi.begin(ssid, password);

        // 等待连接
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 40)
        {
            delay(500);
            Serial.print(".");
            attempts++;
        }

        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.println("\n✅ WiFi连接成功!");
            Serial.print("IP地址: ");
            Serial.println(WiFi.localIP());
        }
        else
        {
            Serial.println("\n❌ WiFi连接失败");
            return;
        }
    }
    else
    {
        Serial.print("WiFi已连接，IP地址: ");
        Serial.println(WiFi.localIP());
    }

    // 创建Web服务器实例
    server = new AsyncWebServer(80);
    webserverMode = WS_MODE_RUNNING;

    // 主页路由
    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request)
               {
        String html = String(index_html);
        if (webserverMode == WS_MODE_RUNNING) {
            html.replace("%STATUS_CLASS%", "active");
            html.replace("%STATUS_TEXT%", "运行中");
        } else {
            html.replace("%STATUS_CLASS%", "");
            html.replace("%STATUS_TEXT%", "已关闭");
        }
        // 添加WiFi名称
        html.replace("<span id=\"wifi\"></span>", WiFi.SSID());
        request->send_P(200, "text/html; charset=utf-8", html.c_str()); });

    // OTA页面路由
    server->on("/ota", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send_P(200, "text/html; charset=utf-8", ota_html); });

    // OTA上传处理
    server->on("/update", HTTP_POST, [](AsyncWebServerRequest *request)
               {
            // 上传完成后的回调
            if (Update.hasError()) {
                AsyncWebServerResponse *response = request->beginResponse(500, "text/plain", "Update failed");
                request->send(response);
            } else {
                AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Update success");
                request->send(response);
                // 延迟重启，让响应发送完成
                delay(1000);
                ESP.restart();
            } }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
               {
            // 上传过程中的回调
            handleOTAUpload(request, filename, index, data, len, final); });

    // 文件管理页面路由
    server->on("/files", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send_P(200, "text/html; charset=utf-8", files_html); });

    // ========== 文件管理API ==========

    // 获取文件系统空间信息
    server->on("/api/fs/info", HTTP_GET, [](AsyncWebServerRequest *request)
               {
        uint64_t totalBytes = LittleFS.totalBytes();
        uint64_t usedBytes = LittleFS.usedBytes();
        uint64_t freeBytes = totalBytes - usedBytes;

        // 转换为 KB
        uint32_t totalKB = totalBytes / 1024;
        uint32_t usedKB = usedBytes / 1024;
        uint32_t freeKB = freeBytes / 1024;
        uint32_t freePercent = totalBytes > 0 ? (freeBytes * 100) / totalBytes : 0;

        DynamicJsonDocument doc(256);
        doc["total"] = totalKB;
        doc["used"] = usedKB;
        doc["free"] = freeKB;
        doc["freePercent"] = freePercent;

        String response;
        serializeJsonPretty(doc, response);
        request->send(200, "application/json", response); });

    // 获取目录内容（支持path参数）
    server->on("/list", HTTP_GET, [](AsyncWebServerRequest *request)
               {
        String path = "/";
        if (request->hasParam("path")) {
            path = request->getParam("path")->value();
            // 路径安全处理：确保以/开头，去除多余的斜杠
            if (!path.startsWith("/")) path = "/" + path;
            while (path.indexOf("//") != -1) path.replace("//", "/");
        }

        File dir = LittleFS.open(path);
        if (!dir || !dir.isDirectory()) {
            request->send(404, "application/json", "{\"error\":\"Directory not found\"}");
            return;
        }

        // 使用 ArduinoJson 构建响应
        DynamicJsonDocument doc(4096);
        JsonArray array = doc.to<JsonArray>();

        File entry = dir.openNextFile();
        while (entry) {
            JsonObject obj = array.createNestedObject();
            obj["name"] = String(entry.name());
            obj["isDirectory"] = entry.isDirectory();
            if (!entry.isDirectory()) {
                obj["size"] = entry.size();
            }
            entry = dir.openNextFile();
        }
        dir.close();

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response); });

    // 创建目录
    server->on("/mkdir", HTTP_POST, [](AsyncWebServerRequest *request)
               {
        if (!request->hasParam("path", true)) {
            request->send(400, "text/plain", "Missing path");
            return;
        }
        String path = request->getParam("path", true)->value();
        // 路径安全处理
        if (!path.startsWith("/")) path = "/" + path;
        while (path.indexOf("//") != -1) path.replace("//", "/");

        if (LittleFS.mkdir(path)) {
            request->send(200, "text/plain", "OK");
        } else {
            request->send(500, "text/plain", "Failed to create directory");
        } });

    // 删除文件或空目录
    server->on("/delete", HTTP_POST, [](AsyncWebServerRequest *request)
               {
        if (!request->hasParam("path", true)) {
            request->send(400, "text/plain", "Missing path");
            return;
        }
        String path = request->getParam("path", true)->value();
        if (!path.startsWith("/")) path = "/" + path;
        while (path.indexOf("//") != -1) path.replace("//", "/");

        File f = LittleFS.open(path);
        if (!f) {
            request->send(404, "text/plain", "Not found");
            return;
        }

        if (f.isDirectory()) {
            // 检查目录是否为空
            File dir = LittleFS.open(path);
            if (dir.openNextFile()) {
                dir.close();
                request->send(409, "text/plain", "Directory not empty");
                return;
            }
            dir.close();
            if (LittleFS.rmdir(path)) {
                request->send(200, "text/plain", "OK");
            } else {
                request->send(500, "text/plain", "Failed to remove directory");
            }
        } else {
            if (LittleFS.remove(path)) {
                request->send(200, "text/plain", "OK");
            } else {
                request->send(500, "text/plain", "Failed to delete file");
            }
        }
        f.close(); });

    // 文件上传（支持path参数指定目标目录）
    server->on("/upload", HTTP_POST, 
               [](AsyncWebServerRequest *request) {
                   // 上传完成后的回调
                   if (uploadFile && uploadFileSize > 0) {
                       uploadFile.close();
                       Serial.printf("文件上传完成: %s (大小: %u KB)\n", request->getParam("path", true) ? request->getParam("path", true)->value().c_str() : "/", uploadFileSize / 1024);
                       uploadFileSize = 0;
                   }
                   request->send(200, "text/plain", "OK");
               },
               [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
               { handleFileUpload(request, filename, index, data, len, final); });

    // ========== 舵机管理API ==========

    // 舵机管理页面
    server->on("/servo", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send_P(200, "text/html; charset=utf-8", servo_html); });

    // 获取舵机配置
    server->on("/api/servo/config", HTTP_GET, [](AsyncWebServerRequest *request)
               {
        uint16_t unlock, lock;
        getServoConfig(unlock, lock);

        DynamicJsonDocument doc(128);
        doc["unlock"] = unlock;
        doc["lock"] = lock;

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
               });

    // 保存舵机配置（POST）
    server->on("/api/servo/config", HTTP_POST,
        [](AsyncWebServerRequest *request) {
            // 请求完成后的回调 - 发送成功响应
            request->send(200, "application/json", "{\"status\":\"success\"}");
        },
        NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            // 处理请求体
            static String requestBody = "";
            if (index == 0) {
                requestBody = "";
            }
            requestBody += String((char *)data, len);

            // 当接收完整请求体时处理
            if (index + len == total) {
                DynamicJsonDocument doc(128);
                DeserializationError error = deserializeJson(doc, requestBody);

                if (error) {
                    Serial.println("JSON解析错误");
                    return;
                }

                if (!doc.containsKey("unlock") || !doc.containsKey("lock")) {
                    Serial.println("缺少参数");
                    return;
                }

                uint16_t unlock = doc["unlock"];
                uint16_t lock = doc["lock"];

                saveServoConfig(unlock, lock);
                Serial.printf("舵机配置已保存 - 解锁: %d, 锁定: %d\n", unlock, lock);
            }
        });

    // 执行解锁动作
    server->on("/api/servo/unlock", HTTP_POST, [](AsyncWebServerRequest *request)
               {
        if (isservobusy)
        {
            request->send(409, "application/json", "{\"error\":\"Servo is busy\"}");
            return;
        }

        executeUnlock();
        request->send(200, "application/json", "{\"status\":\"success\"}");
               });

    // 执行锁定动作
    server->on("/api/servo/lock", HTTP_POST, [](AsyncWebServerRequest *request)
               {
        if (isservobusy)
        {
            request->send(409, "application/json", "{\"error\":\"Servo is busy\"}");
            return;
        }

        executeLock();
        request->send(200, "application/json", "{\"status\":\"success\"}");
               });

    // 重启系统路由
    server->on("/restart", HTTP_POST, [](AsyncWebServerRequest *request)
               {
        request->send(200, "text/plain", "Restarting");
        Serial.println("🔄 收到重启请求");
        delay(500);
        ESP.restart(); });

    // 启动服务器
    server->begin();
    Serial.println("🌐 Web服务器已启动");
    Serial.print("主页访问地址: http://");
    Serial.println(WiFi.localIP());
    Serial.print("OTA升级地址: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/ota");
    Serial.println("===========================");
}

// 检查Web服务器是否运行
bool isWebServerRunning()
{
    return (webserverMode == WS_MODE_RUNNING) && (server != nullptr);
}

// 停止Web服务器
void stopWebServer()
{
    if (server != nullptr)
    {
        Serial.println("🛑 停止Web服务器");
        server->end();
        delete server;
        server = nullptr;
        webserverMode = WS_MODE_OFF;
        Serial.println("Web服务器已停止");
    }
}

// 处理OTA上传
void handleOTAUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
    if (!index)
    {
        Serial.println("开始OTA上传");
        Serial.printf("文件名: %s\n", filename.c_str());
        Serial.printf("文件大小: %u 字节\n", request->contentLength());

        // 开始OTA更新
        if (!Update.begin(request->contentLength()))
        {
            Update.printError(Serial);
            return;
        }

        // 设置OTA完成后重启
        Update.onProgress([](size_t progress, size_t total)
                          { Serial.printf("OTA进度: %u%%\n", (progress * 100) / total); });
    }

    // 写入数据
    if (Update.write(data, len) != len)
    {
        Update.printError(Serial);
        return;
    }

    // 上传完成
    if (final)
    {
        if (Update.end(true))
        {
            Serial.println("OTA上传成功，准备重启");
        }
        else
        {
            Update.printError(Serial);
        }
    }
}

// 处理文件上传（支持目标路径，包含空间验证和回滚）
void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
    if (!index)
    {
        // 首次调用，获取目标路径参数
        if (request->hasParam("path", true))
        {
            uploadPath = request->getParam("path", true)->value();
        }
        else
        {
            uploadPath = "/";
        }
        // 确保路径以 / 结尾
        if (!uploadPath.endsWith("/"))
            uploadPath += "/";

        String fullPath = uploadPath + filename;
        // 路径规范化
        while (fullPath.indexOf("//") != -1)
            fullPath.replace("//", "/");

        Serial.println("开始文件上传");
        Serial.printf("目标路径: %s\n", fullPath.c_str());
        Serial.printf("文件大小: %u 字节\n", request->contentLength());

        // 检查剩余空间
        uint64_t totalBytes = LittleFS.totalBytes();
        uint64_t usedBytes = LittleFS.usedBytes();
        uint64_t freeBytes = totalBytes - usedBytes;
        uint64_t fileSize = request->contentLength();

        Serial.printf("存储空间: 总计 %u KB, 已用 %u KB, 剩余 %u KB\n", 
                     totalBytes / 1024, usedBytes / 1024, freeBytes / 1024);

        // 空间不足，拒绝上传
        if (fileSize > freeBytes)
        {
            Serial.printf("空间不足！需要 %u KB，但只有 %u KB 可用\n", fileSize / 1024, freeBytes / 1024);
            Serial.println("拒绝上传");
            return;
        }

        // 打开文件准备写入
        uploadFile = LittleFS.open(fullPath, "w");
        if (!uploadFile)
        {
            Serial.println("无法创建文件");
            return;
        }

        uploadFileSize = 0;
    }

    if (uploadFile)
    {
        // 写入数据
        size_t written = uploadFile.write(data, len);
        if (written != len)
        {
            Serial.printf("写入失败！期望 %u 字节，实际写入 %u 字节\n", len, written);
            // 写入失败，回滚：关闭文件并删除
            uploadFile.close();
            String fullPath = uploadPath + filename;
            while (fullPath.indexOf("//") != -1)
                fullPath.replace("//", "/");
            LittleFS.remove(fullPath);
            Serial.println("已回滚：删除不完整的文件");
            return;
        }
        uploadFileSize += len;
    }

    if (final)
    {
        if (uploadFile)
        {
            uploadFile.close();
            Serial.printf("文件上传完成: %s (大小: %u KB)\n", filename.c_str(), uploadFileSize / 1024);
        }
        uploadPath = "";
    }
}
