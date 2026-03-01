#ifndef HTML_OTA_H
#define HTML_OTA_H

#include <pgmspace.h>

// OTA上传页面HTML（优化版 - UI/UX Pro Max设计系统）
const char ota_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>OTA 固件升级</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap');

        :root {
            /* 配色系统 - 浅色模式 */
            --color-primary: #0891B2;
            --color-primary-hover: #0E7490;
            --color-primary-light: #ECFEFF;
            --color-secondary: #22D3EE;
            --color-success: #22C55E;
            --color-success-hover: #16A34A;
            --color-danger: #EF4444;
            --color-danger-hover: #DC2626;
            --color-warning: #F59E0B;
            
            /* 文本颜色 */
            --color-text: #164E63;
            --color-text-secondary: #0E7490;
            --color-text-muted: #475569;
            --color-text-light: #94A3B8;
            
            /* 背景颜色 */
            --color-bg-page: linear-gradient(135deg, #ECFEFF 0%, #F0FDFA 100%);
            --color-bg-card: rgba(255, 255, 255, 0.85);
            --color-bg-elevated: #F8FAFC;
            --color-bg-hover: rgba(8, 145, 178, 0.04);
            
            /* 边框颜色 */
            --color-border: rgba(8, 145, 178, 0.15);
            --color-border-hover: rgba(8, 145, 178, 0.25);
            
            /* 阴影系统 */
            --shadow-sm: 0 1px 2px rgba(0, 0, 0, 0.04);
            --shadow-md: 0 4px 12px rgba(0, 0, 0, 0.08);
            --shadow-lg: 0 8px 24px rgba(0, 0, 0, 0.12);
            
            /* 圆角 */
            --radius-sm: 6px;
            --radius-md: 10px;
            --radius-lg: 16px;
            
            /* 间距 - 8px网格系统 */
            --spacing-xs: 4px;
            --spacing-sm: 8px;
            --spacing-md: 16px;
            --spacing-lg: 24px;
            --spacing-xl: 32px;
            --spacing-2xl: 48px;
            
            /* 过渡 */
            --transition-fast: all 0.15s ease;
            --transition-base: all 0.25s ease;
            --transition-smooth: all 0.35s cubic-bezier(0.4, 0, 0.2, 1);
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
            text-align: center;
            min-height: 100vh;
            background: var(--color-bg-page);
            color: var(--color-text);
            line-height: 1.6;
            padding: var(--spacing-xl) var(--spacing-md);
            -webkit-font-smoothing: antialiased;
            -moz-osx-font-smoothing: grayscale;
        }

        .container {
            max-width: 640px;
            margin: 0 auto;
            background: var(--color-bg-card);
            backdrop-filter: blur(20px);
            -webkit-backdrop-filter: blur(20px);
            padding: var(--spacing-xl);
            border-radius: var(--radius-lg);
            box-shadow: var(--shadow-lg);
            border: 1px solid var(--color-border);
        }

        h1 {
            color: var(--color-primary);
            font-size: 28px;
            font-weight: 700;
            margin: 0 0 var(--spacing-lg) 0;
            letter-spacing: -0.02em;
            line-height: 1.2;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: var(--spacing-sm);
        }

        h1 svg {
            width: 32px;
            height: 32px;
        }

        /* 上传区域 */
        .upload-box {
            border: 2px dashed var(--color-primary);
            padding: var(--spacing-2xl) var(--spacing-lg);
            margin: var(--spacing-lg) 0;
            border-radius: var(--radius-md);
            background: linear-gradient(135deg, var(--color-primary-light) 0%, white 100%);
            transition: var(--transition-base);
            cursor: pointer;
            position: relative;
            overflow: hidden;
            text-align: center;
            user-select: none;
        }

        .upload-box:hover {
            border-color: var(--color-primary-hover);
            transform: translateY(-2px);
            box-shadow: var(--shadow-md);
        }

        .upload-box.drag-over {
            border-color: var(--color-success);
            border-style: solid;
            background: linear-gradient(135deg, #DCFCE7 0%, white 100%);
            transform: scale(1.02);
            box-shadow: var(--shadow-lg);
        }

        .upload-box.drag-over::after {
            content: '释放以上传';
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            color: var(--color-success-hover);
            font-weight: 700;
            font-size: 18px;
            pointer-events: none;
            z-index: 2;
        }

        .upload-box-content {
            position: relative;
            z-index: 1;
        }

        .upload-box-content svg {
            width: 64px;
            height: 64px;
            color: var(--color-primary);
            margin-bottom: var(--spacing-lg);
            transition: var(--transition-base);
        }

        .upload-box:hover .upload-box-content svg {
            transform: scale(1.1);
            color: var(--color-primary-hover);
        }

        .upload-box-content h3 {
            color: var(--color-text);
            font-size: 18px;
            font-weight: 600;
            margin: 0 0 var(--spacing-sm) 0;
        }

        .upload-box-content p {
            color: var(--color-text-muted);
            font-size: 14px;
            margin: 0;
        }

        .upload-box-content .file-info {
            margin-top: var(--spacing-md);
            padding: var(--spacing-md);
            background: var(--color-bg-elevated);
            border-radius: var(--radius-sm);
            color: var(--color-primary);
            font-weight: 600;
            font-size: 14px;
            display: none;
            text-align: center;
            border: 1px solid var(--color-border);
        }

        .upload-box-content .file-info strong {
            display: block;
            margin-bottom: var(--spacing-xs);
            font-size: 15px;
        }

        .upload-box-content .file-info .file-size {
            color: var(--color-text-muted);
            font-weight: 500;
            font-size: 13px;
        }

        /* 进度条 */
        .progress-container {
            width: 100%;
            height: 32px;
            background: #E2E8F0;
            border-radius: 16px;
            overflow: hidden;
            margin: var(--spacing-lg) 0;
            display: none;
            box-shadow: inset 0 2px 4px rgba(0, 0, 0, 0.1);
        }

        .progress-bar {
            width: 0%;
            height: 100%;
            background: linear-gradient(90deg, var(--color-success) 0%, var(--color-success-hover) 100%);
            border-radius: 16px;
            display: flex;
            align-items: center;
            justify-content: center;
            color: white;
            font-weight: 700;
            font-size: 14px;
            transition: width 0.3s ease;
            position: relative;
        }

        .progress-bar::after {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.3), transparent);
            animation: shimmer 1.5s infinite;
        }

        @keyframes shimmer {
            0% { transform: translateX(-100%); }
            100% { transform: translateX(100%); }
        }

        /* 状态消息 */
        .status {
            padding: 16px 20px;
            margin: var(--spacing-md) 0;
            border-radius: var(--radius-md);
            display: none;
            font-size: 14px;
            font-weight: 500;
            animation: slideIn 0.3s ease;
        }

        @keyframes slideIn {
            from {
                opacity: 0;
                transform: translateY(-10px);
            }
            to {
                opacity: 1;
                transform: translateY(0);
            }
        }

        .status.success {
            background: linear-gradient(135deg, #DCFCE7 0%, #BBF7D0 100%);
            color: #166534;
            border-left: 4px solid var(--color-success);
        }

        .status.error {
            background: linear-gradient(135deg, #FEE2E2 0%, #FECACA 100%);
            color: #991B1B;
            border-left: 4px solid var(--color-danger);
        }

        .status.info {
            background: linear-gradient(135deg, var(--color-primary-light) 0%, white 100%);
            color: var(--color-primary-hover);
            border-left: 4px solid var(--color-primary);
        }

        .status svg {
            width: 20px;
            height: 20px;
            margin-right: var(--spacing-sm);
            display: inline-block;
            vertical-align: middle;
        }

        /* 返回链接 */
        .back-link {
            display: inline-flex;
            align-items: center;
            margin-top: var(--spacing-lg);
            color: var(--color-primary);
            text-decoration: none;
            font-weight: 500;
            font-size: 14px;
            transition: var(--transition-base);
            padding: var(--spacing-sm) var(--spacing-md);
            border-radius: var(--radius-sm);
        }

        .back-link:hover {
            color: var(--color-primary-hover);
            background: var(--color-bg-hover);
        }

        .back-link svg {
            width: 16px;
            height: 16px;
            margin-right: var(--spacing-sm);
            transition: var(--transition-base);
        }

        .back-link:hover svg {
            transform: translateX(-3px);
        }

        /* 响应式设计 */
        @media (max-width: 768px) {
            body {
                padding: var(--spacing-lg) var(--spacing-md);
            }

            .container {
                padding: var(--spacing-lg);
            }

            h1 {
                font-size: 24px;
            }

            .upload-box {
                padding: var(--spacing-lg);
            }
        }

        @media (max-width: 480px) {
            body {
                padding: var(--spacing-md);
            }

            .container {
                padding: var(--spacing-md);
            }

            h1 {
                font-size: 20px;
            }

            .upload-box {
                padding: var(--spacing-md);
            }

            .upload-box-content svg {
                width: 48px;
                height: 48px;
            }

            .upload-box-content h3 {
                font-size: 16px;
            }
        }

        /* 减少动画偏好 */
        @media (prefers-reduced-motion: reduce) {
            * {
                animation-duration: 0.01ms !important;
                animation-iteration-count: 1 !important;
                transition-duration: 0.01ms !important;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                <rect x="4" y="4" width="16" height="16" rx="2" ry="2"></rect>
                <rect x="9" y="9" width="6" height="6"></rect>
                <line x1="9" y1="1" x2="9" y2="4"></line>
                <line x1="15" y1="1" x2="15" y2="4"></line>
                <line x1="9" y1="20" x2="9" y2="23"></line>
                <line x1="15" y1="20" x2="15" y2="23"></line>
                <line x1="20" y1="9" x2="23" y2="9"></line>
                <line x1="20" y1="14" x2="23" y2="14"></line>
                <line x1="1" y1="9" x2="4" y2="9"></line>
                <line x1="1" y1="14" x2="4" y2="14"></line>
            </svg>
            OTA 固件升级
        </h1>
        
        <div class="upload-box" id="uploadBox" onclick="document.getElementById('file').click()">
            <div class="upload-box-content" id="uploadContent">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round">
                    <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"></path>
                    <polyline points="17 8 12 3 7 8"></polyline>
                    <line x1="12" y1="3" x2="12" y2="15"></line>
                </svg>
                <h3>点击或拖拽上传</h3>
                <p>支持 .bin 格式的固件文件</p>
                <div class="file-info" id="fileInfo">
                    <strong id="fileName"></strong>
                    <span class="file-size" id="fileSize"></span>
                </div>
            </div>
            <input type="file" id="file" accept=".bin" style="display: none;" aria-label="选择固件文件">
        </div>
        
        <div class="progress-container" id="progressContainer">
            <div class="progress-bar" id="progressBar">0%</div>
        </div>
        
        <div class="status info" id="statusInfo">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                <circle cx="12" cy="12" r="10"></circle>
                <line x1="12" y1="16" x2="12" y2="12"></line>
                <line x1="12" y1="8" x2="12.01" y2="8"></line>
            </svg>
            准备就绪
        </div>
        
        <div class="status success" id="statusSuccess">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"></path>
                <polyline points="22 4 12 14.01 9 11.01"></polyline>
            </svg>
            <strong>上传成功！</strong><br>
            设备将自动重启...
        </div>
        
        <div class="status error" id="statusError">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                <circle cx="12" cy="12" r="10"></circle>
                <line x1="15" y1="9" x2="9" y2="15"></line>
                <line x1="9" y1="9" x2="15" y2="15"></line>
            </svg>
            <strong>上传失败！</strong><br>
            <span id="errorMsg"></span>
        </div>
        
        <a href="/" class="back-link">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                <line x1="19" y1="12" x2="5" y2="12"></line>
                <polyline points="12 19 5 12 12 5"></polyline>
            </svg>
            返回主页
        </a>
    </div>
    
    <script>
        let uploadInProgress = false;
        let selectedFile = null;
        
        const uploadBox = document.getElementById('uploadBox');
        const fileInput = document.getElementById('file');
        const uploadContent = document.getElementById('uploadContent');
        const fileInfo = document.getElementById('fileInfo');
        const fileName = document.getElementById('fileName');
        const fileSize = document.getElementById('fileSize');
        
        // 初始化
        fileInput.addEventListener('change', handleFileSelect);
        
        // 点击上传
        function handleFileSelect(e) {
            const file = e.target.files[0];
            if (file) {
                selectedFile = file;
                displayFileInfo(file);
                uploadFile();
            }
        }
        
        // 显示文件信息
        function displayFileInfo(file) {
            uploadContent.style.display = 'none';
            fileInfo.style.display = 'block';
            fileName.textContent = file.name;
            fileSize.textContent = formatFileSize(file.size);
        }
        
        // 格式化文件大小
        function formatFileSize(bytes) {
            if (bytes < 1024) return bytes + ' B';
            if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(2) + ' KB';
            if (bytes < 1024 * 1024 * 1024) return (bytes / (1024 * 1024)).toFixed(2) + ' MB';
            return (bytes / (1024 * 1024 * 1024)).toFixed(2) + ' GB';
        }
        
        // 拖拽上传
        uploadBox.addEventListener('dragover', function(e) {
            e.preventDefault();
            e.stopPropagation();
            uploadBox.classList.add('drag-over');
        });
        
        uploadBox.addEventListener('dragleave', function(e) {
            e.preventDefault();
            e.stopPropagation();
            uploadBox.classList.remove('drag-over');
        });
        
        uploadBox.addEventListener('drop', function(e) {
            e.preventDefault();
            e.stopPropagation();
            uploadBox.classList.remove('drag-over');
            
            const files = e.dataTransfer.files;
            if (files.length > 0) {
                const file = files[0];
                if (file.name.endsWith('.bin')) {
                    selectedFile = file;
                    displayFileInfo(file);
                    uploadFile();
                } else {
                    showStatus('error', '请选择 .bin 格式的固件文件');
                }
            }
        });
        
        // 上传文件
        function uploadFile() {
            const file = selectedFile || fileInput.files[0];
            
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
            
            showStatus('info', '正在上传，请稍候...');
            document.getElementById('progressContainer').style.display = 'block';
            uploadBox.style.pointerEvents = 'none';
            uploadBox.style.opacity = '0.6';
            
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
                    resetUploadBox();
                }
                uploadInProgress = false;
            };
            
            xhr.onerror = function() {
                showStatus('error', '网络错误，请重试');
                resetUploadBox();
                uploadInProgress = false;
            };
            
            xhr.open('POST', '/update', true);
            xhr.send(formData);
        }
        
        // 重置上传区域
        function resetUploadBox() {
            uploadBox.style.pointerEvents = '';
            uploadBox.style.opacity = '';
            uploadContent.style.display = '';
            fileInfo.style.display = 'none';
            fileInput.value = '';
            selectedFile = null;
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

#endif // HTML_OTA_H