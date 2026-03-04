// OTA升级页面逻辑模块
(function() {
    let selectedFile = null;
    
    // 初始化
    function init() {
        loadSystemInfo();
        initFileUpload();
    }
    
    function loadSystemInfo() {
        fetch('/api/system/info')
            .then(response => {
                if (!response.ok) throw new Error('HTTP ' + response.status);
                return response.json();
            })
            .then(response => {
                if (response.success && response.data) {
                    const data = response.data;
                    document.getElementById('chipModel').textContent = data.chipModel || '未知';
                    document.getElementById('chipId').textContent = data.chipId || '未知';
                    document.getElementById('currentVersion').textContent = data.version || '未知';
                } else {
                    console.error('获取系统信息失败: ' + (response.message || '未知错误'));
                    document.getElementById('chipModel').textContent = '获取失败';
                    document.getElementById('chipId').textContent = '-';
                    document.getElementById('currentVersion').textContent = '-';
                }
            })
            .catch(err => {
                console.error('获取系统信息失败:', err);
                document.getElementById('chipModel').textContent = '获取失败';
                document.getElementById('chipId').textContent = '-';
                document.getElementById('currentVersion').textContent = '-';
            });
    }
    
    function initFileUpload() {
        const uploadBox = document.getElementById('uploadBox');
        const fileInput = document.getElementById('file');
        
        if (!uploadBox || !fileInput) return;
        
        // 点击上传区域触发文件选择
        uploadBox.addEventListener('click', function(e) {
            // 防止冒泡，避免重复触发
            if (e.target !== fileInput) {
                fileInput.click();
            }
        });
        
        fileInput.addEventListener('change', function(e) {
            const file = e.target.files[0];
            if (file) {
                selectedFile = file;
                displayFileInfo(file);
                validateAndUpload(file);
            }
        });
        
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
                selectedFile = files[0];
                displayFileInfo(files[0]);
                validateAndUpload(files[0]);
            }
        });
    }
    
    function displayFileInfo(file) {
        const uploadBoxContent = document.getElementById('uploadBoxContent');
        const fileInfo = document.getElementById('fileInfo');
        const fileName = document.getElementById('fileName');
        const fileSize = document.getElementById('fileSize');
        
        if (uploadBoxContent) uploadBoxContent.style.display = 'none';
        if (fileInfo) {
            fileInfo.style.display = 'block';
            if (fileName) fileName.textContent = file.name;
            if (fileSize) fileSize.textContent = formatFileSize(file.size);
        }
    }
    
    function resetUploadArea() {
        const uploadBoxContent = document.getElementById('uploadBoxContent');
        const fileInfo = document.getElementById('fileInfo');
        const fileInput = document.getElementById('file');
        
        if (uploadBoxContent) uploadBoxContent.style.display = '';
        if (fileInfo) fileInfo.style.display = 'none';
        if (fileInput) fileInput.value = '';
        selectedFile = null;
    }
    
    function validateAndUpload(file) {
        // 验证文件扩展名
        if (!file.name.endsWith('.bin')) {
            showToast('请选择 .bin 格式的固件文件', 'error');
            resetUploadArea();
            return;
        }
        
        // 验证文件大小 (ESP32-C3 OTA分区通常限制为2MB)
        const MAX_FILE_SIZE = 2 * 1024 * 1024; // 2MB
        if (file.size > MAX_FILE_SIZE) {
            showToast('文件过大！最大支持2MB', 'error');
            resetUploadArea();
            return;
        }
        
        // 开始上传
        uploadFile(file);
    }
    
    function uploadFile(file) {
        const fileToUpload = file || selectedFile || document.getElementById('file').files[0];
        if (!fileToUpload) {
            showToast('请选择文件', 'warning');
            return;
        }
        
        // 显示进度条
        document.getElementById('progressContainer').style.display = 'block';
        updateProgress(0);
        showToast('正在上传固件...', 'info');
        
        const formData = new FormData();
        formData.append('file', fileToUpload);
        
        const xhr = new XMLHttpRequest();
        
        xhr.upload.onprogress = function(e) {
            if (e.lengthComputable) {
                const percent = Math.round((e.loaded / e.total) * 100);
                updateProgress(percent);
            }
        };
        
        xhr.onload = function() {
            if (xhr.status === 200) {
                showToast('固件上传成功！设备即将重启...', 'success');
                document.getElementById('uploadBox').style.display = 'none';
            } else {
                // 尝试解析后端返回的错误信息
                let errorMsg = 'HTTP ' + xhr.status;
                try {
                    const responseText = xhr.responseText;
                    if (responseText) {
                        // 尝试解析JSON格式
                        if (responseText.startsWith('{') || responseText.startsWith('[')) {
                            const jsonData = JSON.parse(responseText);
                            // ApiResponse格式: {success, code, message, data}
                            errorMsg = jsonData.message || jsonData.data?.error || jsonData.data?.reason || errorMsg;
                        } else {
                            // 直接使用文本响应
                            errorMsg = responseText;
                        }
                    }
                } catch (e) {
                    console.error('解析错误信息失败:', e);
                }
                showToast('上传失败: ' + errorMsg, 'error');
                resetUploadArea();
                document.getElementById('progressContainer').style.display = 'none';
            }
        };

        xhr.onerror = function() {
            showToast('上传失败: 网络错误', 'error');
            resetUploadArea();
            document.getElementById('progressContainer').style.display = 'none';
        };
        
        xhr.open('POST', '/api/system/firmware', true);
        xhr.send(formData);
    }
    
    function updateProgress(percent) {
        const progressBar = document.getElementById('progressBar');
        progressBar.style.width = percent + '%';
        progressBar.textContent = percent + '%';
    }

    // 页面加载完成后初始化
    window.addEventListener('DOMContentLoaded', init);
})();