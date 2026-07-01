// 文件管理页面逻辑模块
(function() {
    let currentPath = '/';
    let selectedFile = null;
    let fileHashInfo = null;
    let systemInfoPromise = null;

    // SVG图标
    const dirIcon = `<svg class="icon" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#22C55E" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M22 19a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h5l2 3h9a2 2 0 0 1 2 2z"></path></svg>`;
    const fileIcon = `<svg class="icon" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="#0891B2" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M13 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V9z"></path><polyline points="13 2 13 9 20 9"></polyline></svg>`;
    const deleteIcon = `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="3 6 5 6 21 6"></polyline><path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path></svg>`;
    const downloadIcon = `<svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"></path><polyline points="7 10 12 15 17 10"></polyline><line x1="12" y1="15" x2="12" y2="3"></line></svg>`;

    // 初始化
    function init() {
        loadList(currentPath);
        loadStorageInfo();
        loadSystemInfo();
        initUploadInteractions();

        // WebSocket 重连后自动刷新存储与文件列表（去抖：2s 内的重复事件忽略）
        let lastWsRefreshTs = 0;
        document.addEventListener('ws-connected', function() {
            const now = Date.now();
            if (now - lastWsRefreshTs < 2000) return;
            lastWsRefreshTs = now;
            loadStorageInfo();
            loadList(currentPath);
        });
    }

    function loadSystemInfo() {
        if (systemInfoPromise) {
            return systemInfoPromise;
        }

        systemInfoPromise = fetch('/api/system/info')
            .then(response => {
                if (!response.ok) throw new Error('HTTP ' + response.status);
                return response.json();
            })
            .then(response => {
                if (response.success && response.data) {
                    fileHashInfo = response.data;
                }
                return fileHashInfo;
            })
            .catch(err => {
                console.error('加载系统信息失败:', err);
                fileHashInfo = null;
                throw err;
            })
            .finally(() => {
                systemInfoPromise = null;
            });

        return systemInfoPromise;
    }

    // 初始化拖拽上传
    function initUploadInteractions() {
        const uploadSection = document.getElementById('uploadSection');
        const uploadArea = document.getElementById('uploadArea');
        const fileInput = document.getElementById('file');
        const uploadContent = document.getElementById('uploadContent');
        const fileInfo = document.getElementById('fileInfo');
        const fileName = document.getElementById('fileName');
        const fileSize = document.getElementById('fileSize');
        
        if (!uploadSection || !uploadArea || !fileInput) return;
        
        bindFilePickerTrigger(uploadArea, fileInput);
        initDragAndDrop(uploadArea, fileInput, function(file) {
            if (file) {
                selectedFile = file;
                displayFileInfo(uploadContent, fileInfo, fileName, fileSize, file);
                validateAndUpload(file);
            }
        });
    }

    // 重置上传区域
    function resetSelectedUploadArea() {
        const uploadContent = document.getElementById('uploadContent');
        const fileInfo = document.getElementById('fileInfo');
        const fileInput = document.getElementById('file');

        resetUploadArea(uploadContent, fileInfo, fileInput);
        selectedFile = null;
    }

    // 验证并上传
    function validateAndUpload(file) {
        uploadFile(file);
    }

    // 加载存储信息
    function loadStorageInfo() {
        fetch('/api/filesystem')
            .then(response => {
                if (!response.ok) throw new Error('HTTP ' + response.status);
                return response.json();
            })
            .then(response => {
                if (!response.success || !response.data) {
                    console.error('加载存储信息失败: ' + (response.message || '未知错误'));
                    return;
                }
                const data = response.data;
                document.getElementById('usedStorage').textContent = data.used;
                document.getElementById('totalStorage').textContent = data.total;
                document.getElementById('freeStorage').textContent = data.free;
                document.getElementById('freeStoragePercent').textContent = data.freePercent;
                
                const progressBar = document.getElementById('storageProgressBar');
                const usedPercent = 100 - data.freePercent;
                progressBar.style.width = usedPercent + '%';
                progressBar.setAttribute('aria-valuenow', String(usedPercent));
                
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
    
    function loadList(path) {
        fetch('/api/files?path=' + encodeURIComponent(path))
            .then(response => {
                if (!response.ok) {
                    throw new Error('HTTP ' + response.status);
                }
                return response.json();
            })
            .then(response => {
                if (!response.success || !response.data) {
                    showToast('加载失败: ' + (response.message || '未知错误'), 'error');
                    return;
                }
                currentPath = path;
                const currentPathElement = document.getElementById('currentPath');
                const pathChipElement = currentPathElement && currentPathElement.closest('.path-chip');

                if (currentPathElement) {
                    currentPathElement.textContent = path;
                }

                if (pathChipElement) {
                    pathChipElement.title = path;
                }
                renderList(response.data);
            })
            .catch(err => {
                showToast('加载失败: ' + err.message, 'error');
            });
    }
    
    function renderList(items) {
        const container = document.getElementById('fileListContainer');
        if (!items || items.length === 0) {
            container.innerHTML = '<div class="empty">文件夹为空</div>';
            return;
        }

        // 分离文件夹和文件
        const directories = [];
        const files = [];
        
        items.forEach(item => {
            if (item.isDirectory) {
                directories.push(item);
            } else {
                files.push(item);
            }
        });

        // 分别排序：按名称字母顺序
        directories.sort((a, b) => a.name.localeCompare(b.name));
        files.sort((a, b) => a.name.localeCompare(b.name));

        // 合并：文件夹在前，文件在后
        const sortedItems = [...directories, ...files];

        let html = '';
        sortedItems.forEach((item, index) => {
            const isDir = item.isDirectory;
            const icon = isDir ? dirIcon : fileIcon;
            const sizeStr = isDir ? '<DIR>' : formatFileSize(item.size);
            const escapedName = item.name.replace(/'/g, "\\'").replace(/"/g, '"');
            const onclick = isDir ? `onclick="navigate('${escapedName}')"` : '';
            const delayStyle = `style="animation-delay: ${index * 0.05}s;"`;
            html += `
                <div class="item ${isDir ? 'directory' : ''}" ${delayStyle} ${onclick} role="button" tabindex="0" aria-label="${escapedName}">
                    <span class="name">${icon} ${item.name}</span>
                    <span class="size">${sizeStr}</span>
                    <div class="actions">
                        ${isDir ? '' : '<button class="btn-sm btn-download" onclick="event.stopPropagation(); downloadFile(\'' + escapedName + '\')" aria-label="下载文件">' + downloadIcon + ' 下载</button>'}
                        ${isDir ? '' : '<button class="btn-sm" onclick="event.stopPropagation(); deleteItem(\'' + escapedName + '\', false)" aria-label="删除文件">' + deleteIcon + ' 删除</button>'}
                        ${isDir ? '<button class="btn-sm" onclick="event.stopPropagation(); deleteItem(\'' + escapedName + '\', true)" aria-label="删除目录">' + deleteIcon + ' 删除目录</button>' : ''}
                    </div>
                </div>
            `;
        });
        container.innerHTML = html;
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

    async function createDir() {
        const dirName = await showPrompt('', {
            title: '新建文件夹',
            placeholder: '文件夹名称',
            type: 'success',
            confirmText: '创建',
            cancelText: '取消'
        });
        
        if (!dirName) return;
        
        if (dirName.includes('/') || dirName.includes('\\')) {
            showToast('文件夹名称不能包含 / 或 \\', 'error');
            return;
        }
        
        let fullPath = currentPath === '/' ? '/' + dirName : currentPath + '/' + dirName;
        fullPath = fullPath.replace(/\/+/g, '/');

        fetch('/api/directories', {
            method: 'POST',
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: 'path=' + encodeURIComponent(fullPath)
        })
        .then(response => response.json())
        .then(response => {
            if (response.success) {
                showToast('文件夹创建成功', 'success');
                loadList(currentPath);
            } else {
                showToast('创建失败: ' + (response.message || '未知错误'), 'error');
            }
        })
        .catch(err => showToast('请求失败: ' + err.message, 'error'));
    }

    async function uploadFile(file) {
        const fileToUpload = file || selectedFile || document.getElementById('file').files[0];
        if (!fileToUpload) {
            showToast('请选择文件', 'warning');
            return;
        }

        let hashHeaders = {};
        try {
            if (!fileHashInfo) {
                await loadSystemInfo();
            }

            hashHeaders = await buildFileHashHeaders(fileToUpload, {
                algorithm: fileHashInfo && fileHashInfo.fileHashAlgorithm,
                headerName: fileHashInfo && fileHashInfo.fileHashHeader,
                required: fileHashInfo ? fileHashInfo.fileHashRequired !== false : true,
                label: '文件'
            });
        } catch (err) {
            showToast(err.message || '文件哈希计算失败', 'error');
            resetSelectedUploadArea();
            return;
        }

        const formData = new FormData();
        formData.append('file', fileToUpload);
        
        // 显示进度条
        const uploadProgress = document.getElementById('uploadProgress');
        const uploadProgressBar = document.getElementById('uploadProgressBar');
        const uploadProgressText = document.getElementById('uploadProgressText');
        if (uploadProgress) uploadProgress.style.display = 'block';
        if (uploadProgressBar) uploadProgressBar.style.width = '0%';
        if (uploadProgressText) uploadProgressText.textContent = '0%';

        // 防重复提交：上传期间禁用上传触发区域
        const uploadArea = document.getElementById('uploadArea');
        setButtonLoading(uploadArea, true);

        // 使用 XMLHttpRequest 支持上传进度
        const xhr = new XMLHttpRequest();
        xhr.open('POST', '/api/files?path=' + encodeURIComponent(currentPath), true);
        Object.entries(hashHeaders).forEach(([headerName, headerValue]) => {
            xhr.setRequestHeader(headerName, headerValue);
        });

        // 上传进度监听
        xhr.upload.onprogress = function(e) {
            if (e.lengthComputable) {
                const percentComplete = Math.round((e.loaded / e.total) * 100);
                if (uploadProgressBar) uploadProgressBar.style.width = percentComplete + '%';
                if (uploadProgressText) uploadProgressText.textContent = percentComplete + '%';
            }
        };

        // 上传完成
        xhr.onload = function() {
            setButtonLoading(uploadArea, false);
            if (xhr.status === 200) {
                showToast('上传成功', 'success');
                resetSelectedUploadArea();
                loadList(currentPath);
                loadStorageInfo();
            } else {
                let errorMsg = 'HTTP ' + xhr.status;
                try {
                    if (xhr.responseText) {
                        const jsonData = JSON.parse(xhr.responseText);
                        errorMsg = jsonData.message || jsonData.data?.error || jsonData.data?.reason || errorMsg;
                    }
                } catch (e) {
                    console.error('解析上传错误失败:', e);
                }
                showToast('上传失败: ' + errorMsg, 'error');
                resetSelectedUploadArea();
                // 隐藏进度条
                if (uploadProgress) uploadProgress.style.display = 'none';
            }
        };

        // 上传错误
        xhr.onerror = function() {
            setButtonLoading(uploadArea, false);
            showToast('上传出错: 网络错误', 'error');
            resetSelectedUploadArea();
            // 隐藏进度条
            if (uploadProgress) uploadProgress.style.display = 'none';
        };

        xhr.send(formData);
    }

    function downloadFile(name) {
        let fullPath = currentPath === '/' ? '/' + name : currentPath + '/' + name;
        fullPath = fullPath.replace(/\/+/g, '/');
        
        // 使用window.open打开下载链接
        const downloadUrl = '/api/files/download?path=' + encodeURIComponent(fullPath);
        window.open(downloadUrl, '_blank');
        showToast('开始下载: ' + name, 'info');
    }

    async function deleteItem(name, isDir) {
        let fullPath = currentPath === '/' ? '/' + name : currentPath + '/' + name;
        fullPath = fullPath.replace(/\/+/g, '/');
        const confirmed = await showConfirm(`确定删除 ${fullPath} 吗？`, {
            type: 'danger',
            title: '确认删除'
        });
        if (!confirmed) return;

        fetch('/api/files?path=' + encodeURIComponent(fullPath), {
            method: 'DELETE'
        })
        .then(response => response.json().then(data => ({ status: response.status, data })))
        .then(({ status, data }) => {
            if (data.success) {
                showToast('删除成功', 'success');
                loadList(currentPath);
                loadStorageInfo();
            } else if (status === 409) {
                showToast('目录不为空，无法删除', 'error');
            } else {
                showToast('删除失败: ' + (data.message || '未知错误'), 'error');
            }
        })
        .catch(err => showToast('请求失败: ' + err.message, 'error'));
    }

    // 暴露给全局
    window.goToParent = goToParent;
    window.createDir = createDir;
    window.navigate = navigate;
    window.downloadFile = downloadFile;
    window.deleteItem = deleteItem;

    // 页面加载完成后初始化
    window.addEventListener('DOMContentLoaded', init);
})();
