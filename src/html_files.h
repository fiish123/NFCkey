#ifndef HTML_FILES_H
#define HTML_FILES_H

#include <pgmspace.h>

// 文件管理页面HTML（优化版 - UI/UX Pro Max设计系统）
const char files_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>文件管理 - LittleFS</title>
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
            --color-bg-active: rgba(8, 145, 178, 0.08);
            
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
            max-width: 840px;
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
        }

        /* 路径栏 */
        .path-bar {
            display: flex;
            flex-wrap: wrap;
            align-items: center;
            gap: var(--spacing-sm);
            margin: var(--spacing-lg) 0;
            padding: var(--spacing-md);
            background: linear-gradient(135deg, var(--color-bg-elevated) 0%, white 100%);
            border-radius: var(--radius-md);
            border: 1px solid var(--color-border);
            box-shadow: var(--shadow-sm);
        }

        .path-bar span {
            color: var(--color-text-muted);
            font-weight: 500;
            font-size: 14px;
            white-space: nowrap;
        }

        .path-bar input {
            flex: 1;
            min-width: 200px;
            padding: 10px 14px;
            border: 1px solid var(--color-border);
            border-radius: var(--radius-sm);
            background: var(--color-bg-card);
            color: var(--color-text);
            font-family: inherit;
            font-size: 14px;
            transition: var(--transition-base);
        }

        .path-bar input:focus {
            outline: none;
            border-color: var(--color-primary);
            box-shadow: 0 0 0 3px var(--color-primary-light);
        }

        /* 按钮基础样式 */
        .btn {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            padding: 10px 20px;
            color: white;
            background: linear-gradient(135deg, var(--color-primary) 0%, var(--color-primary-hover) 100%);
            border: none;
            border-radius: var(--radius-sm);
            font-size: 14px;
            font-weight: 500;
            cursor: pointer;
            transition: var(--transition-smooth);
            box-shadow: var(--shadow-sm);
            min-height: 40px;
            white-space: nowrap;
        }

        .btn:hover {
            transform: translateY(-2px);
            box-shadow: var(--shadow-md);
        }

        .btn:active {
            transform: translateY(0);
        }

        .btn:focus-visible {
            outline: 3px solid var(--color-primary-light);
            outline-offset: 2px;
        }

        /* 优化后的删除按钮样式 */
        .btn-sm {
            padding: 10px 16px;
            font-size: 13px;
            background: linear-gradient(135deg, var(--color-danger) 0%, var(--color-danger-hover) 100%);
            box-shadow: var(--shadow-sm);
            min-height: 36px;
            display: inline-flex;
            align-items: center;
            gap: 6px;
            position: relative;
            overflow: hidden;
            border: none;
            border-radius: var(--radius-sm);
            color: white;
            font-weight: 500;
            cursor: pointer;
            transition: var(--transition-smooth);
        }

        .btn-sm::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.2), transparent);
            transition: var(--transition-smooth);
        }

        .btn-sm:hover::before {
            left: 100%;
        }

        .btn-sm:hover {
            background: linear-gradient(135deg, var(--color-danger-hover) 0%, #B91C1C 100%);
            transform: translateY(-2px);
            box-shadow: var(--shadow-md), 0 0 15px rgba(239, 68, 68, 0.3);
        }

        .btn-sm:active {
            transform: translateY(0);
            box-shadow: var(--shadow-sm);
        }

        .btn-sm svg {
            width: 14px;
            height: 14px;
            transition: var(--transition-base);
        }

        .btn-sm:hover svg {
            transform: scale(1.1) rotate(5deg);
        }

        .btn-sm:focus-visible {
            outline: 3px solid rgba(239, 68, 68, 0.2);
            outline-offset: 2px;
        }

        /* 下载按钮样式 - 蓝色主题 */
        .btn-sm.btn-download {
            background: linear-gradient(135deg, #3B82F6 0%, #2563EB 100%);
        }

        .btn-sm.btn-download::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.2), transparent);
            transition: var(--transition-smooth);
        }

        .btn-sm.btn-download:hover::before {
            left: 100%;
        }

        .btn-sm.btn-download:hover {
            background: linear-gradient(135deg, #2563EB 0%, #1D4ED8 100%);
            transform: translateY(-2px);
            box-shadow: var(--shadow-md), 0 0 15px rgba(59, 130, 246, 0.3);
        }

        .btn-sm.btn-download:active {
            transform: translateY(0);
            box-shadow: var(--shadow-sm);
        }

        .btn-sm.btn-download svg {
            width: 14px;
            height: 14px;
            transition: var(--transition-base);
        }

        .btn-sm.btn-download:hover svg {
            transform: scale(1.1) translateY(2px);
        }

        .btn-sm.btn-download:focus-visible {
            outline: 3px solid rgba(59, 130, 246, 0.2);
            outline-offset: 2px;
        }

        /* 存储信息卡片 */
        .storage-info {
            margin: var(--spacing-lg) 0;
            padding: var(--spacing-lg);
            background: linear-gradient(135deg, var(--color-bg-elevated) 0%, white 100%);
            border-radius: var(--radius-md);
            border: 1px solid var(--color-border);
            text-align: left;
            box-shadow: var(--shadow-sm);
        }

        .storage-info .info-text {
            margin-bottom: var(--spacing-md);
            font-size: 14px;
            color: var(--color-text);
            display: flex;
            justify-content: space-between;
            flex-wrap: wrap;
            gap: var(--spacing-sm);
        }

        .storage-info .info-text span {
            font-weight: 600;
        }

        .storage-info .progress-container {
            width: 100%;
            height: 24px;
            background: #E2E8F0;
            border-radius: 12px;
            overflow: hidden;
            box-shadow: inset 0 1px 3px rgba(0, 0, 0, 0.1);
        }

        .storage-info .progress-bar {
            height: 100%;
            background: linear-gradient(90deg, var(--color-success) 0%, var(--color-success-hover) 100%);
            width: 0%;
            transition: width 0.5s ease;
            border-radius: 12px;
            position: relative;
        }

        .storage-info .progress-bar::after {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.3), transparent);
            animation: shimmer 2s infinite;
        }

        @keyframes shimmer {
            0% { transform: translateX(-100%); }
            100% { transform: translateX(100%); }
        }

        .storage-info .progress-bar.warning {
            background: linear-gradient(90deg, var(--color-warning) 0%, #D97706 100%);
        }

        .storage-info .progress-bar.danger {
            background: linear-gradient(90deg, var(--color-danger) 0%, var(--color-danger-hover) 100%);
        }

        .storage-info .progress-text {
            margin-top: var(--spacing-sm);
            font-size: 13px;
            color: var(--color-text-muted);
            text-align: right;
        }

        /* 上传区域 */
        .upload-section {
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

        .upload-section:hover {
            border-color: var(--color-primary-hover);
            background: linear-gradient(135deg, rgba(8, 145, 178, 0.08) 0%, white 100%);
            transform: translateY(-2px);
            box-shadow: var(--shadow-md);
        }

        .upload-section.drag-over {
            border-color: var(--color-success);
            border-style: solid;
            background: linear-gradient(135deg, #DCFCE7 0%, white 100%);
            transform: scale(1.02);
            box-shadow: var(--shadow-lg);
        }

        .upload-section.drag-over::after {
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

        .upload-content {
            position: relative;
            z-index: 1;
        }

        .upload-content svg {
            width: 48px;
            height: 48px;
            color: var(--color-primary);
            margin-bottom: var(--spacing-md);
            transition: var(--transition-base);
        }

        .upload-section:hover .upload-content svg {
            transform: scale(1.1);
            color: var(--color-primary-hover);
        }

        .upload-content h3 {
            color: var(--color-text);
            font-size: 16px;
            font-weight: 600;
            margin: 0 0 var(--spacing-xs) 0;
        }

        .upload-content p {
            margin: 0;
            color: var(--color-text-muted);
            font-size: 13px;
        }

        .file-info {
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

        .file-info strong {
            display: block;
            margin-bottom: var(--spacing-xs);
            font-size: 15px;
            word-break: break-all;
        }

        .file-info .file-size {
            color: var(--color-text-muted);
            font-weight: 500;
            font-size: 13px;
        }

        .upload-section input[type="file"] {
            display: none;
        }

        /* 状态消息 */
        .status {
            padding: 14px 18px;
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

        /* 文件列表 */
        .file-list {
            margin-top: var(--spacing-lg);
            text-align: left;
        }

        .item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            gap: var(--spacing-md);
            padding: var(--spacing-md);
            margin: var(--spacing-xs) 0;
            background: linear-gradient(135deg, var(--color-bg-elevated) 0%, white 100%);
            border-radius: var(--radius-sm);
            border-left: 3px solid var(--color-primary);
            cursor: pointer;
            transition: var(--transition-base);
            box-shadow: var(--shadow-sm);
        }

        .item:hover {
            background: var(--color-bg-hover);
            transform: translateX(4px);
            box-shadow: var(--shadow-md);
        }

        .item:active {
            background: var(--color-bg-active);
        }

        .item.directory {
            border-left-color: var(--color-success);
            font-weight: 600;
        }

        .item .name {
            flex: 1;
            display: flex;
            align-items: center;
            gap: var(--spacing-sm);
            font-size: 14px;
        }

        .item .icon {
            flex-shrink: 0;
            transition: var(--transition-base);
        }

        .item:hover .icon {
            transform: scale(1.1);
        }

        .item .size {
            margin-right: var(--spacing-md);
            color: var(--color-text-muted);
            font-size: 13px;
            font-family: 'Monaco', 'Consolas', monospace;
        }

        .item .actions {
            display: flex;
            gap: var(--spacing-sm);
        }

        .empty {
            text-align: center;
            color: var(--color-text-muted);
            padding: var(--spacing-xl) var(--spacing-md);
            font-size: 14px;
            background: var(--color-bg-elevated);
            border-radius: var(--radius-md);
            border: 2px dashed var(--color-border);
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

            .path-bar {
                flex-direction: column;
                align-items: stretch;
            }

            .path-bar input {
                width: 100%;
            }

            .path-bar .btn {
                width: 100%;
            }

            .item {
                flex-wrap: wrap;
                padding: var(--spacing-sm);
            }

            .item .size {
                width: 100%;
                margin: var(--spacing-xs) 0 0 0;
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

            .upload-section {
                padding: var(--spacing-lg);
            }

            .upload-content svg {
                width: 40px;
                height: 40px;
            }

            .upload-content h3 {
                font-size: 14px;
            }

            .upload-content p {
                font-size: 12px;
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
        <h1>LittleFS 文件管理</h1>

        <div class="path-bar">
            <span>当前路径:</span>
            <input type="text" id="currentPath" value="/" readonly aria-label="当前路径">
            <button class="btn" onclick="goToParent()" aria-label="上级目录">上级目录</button>
            <button class="btn" onclick="createDir()" aria-label="新建文件夹">新建文件夹</button>
        </div>

        <div class="storage-info">
            <div class="info-text">
                <span>已用: <span id="usedStorage">0</span> KB</span>
                <span>总计: <span id="totalStorage">0</span> KB</span>
                <span>剩余: <span id="freeStoragePercent">0</span>%</span>
            </div>
            <div class="progress-container">
                <div class="progress-bar" id="storageProgressBar"></div>
            </div>
            <div class="progress-text">剩余空间: <span id="freeStorage">0</span> KB</div>
        </div>

        <div class="upload-section" id="uploadSection" onclick="document.getElementById('file').click()">
            <div class="upload-content" id="uploadContent">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round">
                    <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"></path>
                    <polyline points="17 8 12 3 7 8"></polyline>
                    <line x1="12" y1="3" x2="12" y2="15"></line>
                </svg>
                <h3>点击或拖拽上传</h3>
                <p>支持任意格式的文件</p>
                <div class="file-info" id="fileInfo">
                    <strong id="fileName"></strong>
                    <span class="file-size" id="fileSize"></span>
                </div>
            </div>
            <input type="file" id="file" style="display: none;" aria-label="选择文件">
        </div>

        <div id="status" class="status info"></div>

        <div class="file-list">
            <div id="fileListContainer"></div>
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
        let currentPath = '/';
        let selectedFile = null;

        window.onload = function() {
            loadList(currentPath);
            loadStorageInfo();
            initDragAndDrop();
        };

        // SVG图标
        const dirIcon = `<svg class="icon" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#22C55E" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M22 19a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h5l2 3h9a2 2 0 0 1 2 2z"></path></svg>`;
        const fileIcon = `<svg class="icon" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#0891B2" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M13 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V9z"></path><polyline points="13 2 13 9 20 9"></polyline></svg>`;
        const deleteIcon = `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="3 6 5 6 21 6"></polyline><path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path></svg>`;
        const downloadIcon = `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"></path><polyline points="7 10 12 15 17 10"></polyline><line x1="12" y1="15" x2="12" y2="3"></line></svg>`;

        // 初始化拖拽上传
        function initDragAndDrop() {
            const uploadSection = document.getElementById('uploadSection');
            const fileInput = document.getElementById('file');
            
            fileInput.addEventListener('change', handleFileSelect);

            uploadSection.addEventListener('dragover', function(e) {
                e.preventDefault();
                e.stopPropagation();
                uploadSection.classList.add('drag-over');
            });

            uploadSection.addEventListener('dragleave', function(e) {
                e.preventDefault();
                e.stopPropagation();
                uploadSection.classList.remove('drag-over');
            });

            uploadSection.addEventListener('drop', function(e) {
                e.preventDefault();
                e.stopPropagation();
                uploadSection.classList.remove('drag-over');
                
                const files = e.dataTransfer.files;
                if (files.length > 0) {
                    selectedFile = files[0];
                    displayFileInfo(selectedFile);
                    validateAndUpload(selectedFile);
                }
            });
        }

        // 处理文件选择
        function handleFileSelect(e) {
            const file = e.target.files[0];
            if (file) {
                selectedFile = file;
                displayFileInfo(file);
                validateAndUpload(file);
            }
        }

        // 显示文件信息
        function displayFileInfo(file) {
            document.getElementById('uploadContent').style.display = 'none';
            document.getElementById('fileInfo').style.display = 'block';
            document.getElementById('fileName').textContent = file.name;
            document.getElementById('fileSize').textContent = formatFileSize(file.size);
        }

        // 重置上传区域
        function resetUploadArea() {
            document.getElementById('uploadContent').style.display = '';
            document.getElementById('fileInfo').style.display = 'none';
            document.getElementById('file').value = '';
            selectedFile = null;
        }

        // 验证并上传
        function validateAndUpload(file) {
            loadStorageInfo();
            
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
                        resetUploadArea();
                    } else {
                        uploadFile(file);
                    }
                })
                .catch(err => {
                    console.error('验证文件失败:', err);
                    showStatus('验证失败: ' + err.message, 'error');
                });
        }

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
                const ondblclick = isDir ? `ondblclick="navigate('${item.name}')"` : '';
                html += `
                    <div class="item ${isDir ? 'directory' : ''}" ${ondblclick} role="button" tabindex="0" aria-label="${item.name}">
                        <span class="name">${icon} ${item.name}</span>
                        <span class="size">${sizeStr}</span>
                        <div class="actions">
                            ${isDir ? '' : '<button class="btn-sm btn-download" onclick="event.stopPropagation(); downloadFile(\'' + item.name + '\')" aria-label="下载文件">' + downloadIcon + ' 下载</button>'}
                            ${isDir ? '' : '<button class="btn-sm" onclick="event.stopPropagation(); deleteItem(\'' + item.name + '\', false)" aria-label="删除文件">' + deleteIcon + ' 删除</button>'}
                            ${isDir ? '<button class="btn-sm" onclick="event.stopPropagation(); deleteItem(\'' + item.name + '\', true)" aria-label="删除目录">' + deleteIcon + ' 删除目录</button>' : ''}
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

        function uploadFile(file) {
            const fileToUpload = file || selectedFile || document.getElementById('file').files[0];
            if (!fileToUpload) {
                alert('请选择文件');
                return;
            }

            const formData = new FormData();
            formData.append('file', fileToUpload);
            formData.append('path', currentPath);

            fetch('/upload', {
                method: 'POST',
                body: formData
            })
            .then(response => {
                if (response.ok) {
                    showStatus('上传成功', 'success');
                    resetUploadArea();
                    loadList(currentPath);
                    loadStorageInfo();
                } else {
                    showStatus('上传失败', 'error');
                }
            })
            .catch(err => showStatus('上传出错: ' + err.message, 'error'));
        }

        function downloadFile(name) {
            let fullPath = currentPath === '/' ? '/' + name : currentPath + '/' + name;
            fullPath = fullPath.replace(/\/+/g, '/');
            
            // 使用window.open打开下载链接
            const downloadUrl = '/download?path=' + encodeURIComponent(fullPath);
            window.open(downloadUrl, '_blank');
            showStatus('开始下载: ' + name, 'info');
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
                    loadStorageInfo();
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

#endif // HTML_FILES_H