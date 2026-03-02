// 公共JavaScript函数库

// 浮窗通知系统
const Toast = (function() {
    // 创建或获取容器
    function getContainer() {
        let container = document.querySelector('.toast-container');
        if (!container) {
            container = document.createElement('div');
            container.className = 'toast-container';
            document.body.appendChild(container);
        }
        return container;
    }

    // SVG 图标
    const icons = {
        success: `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"></path><polyline points="22 4 12 14.01 9 11.01"></polyline></svg>`,
        error: `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"></circle><line x1="15" y1="9" x2="9" y2="15"></line><line x1="9" y1="9" x2="15" y2="15"></line></svg>`,
        info: `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"></circle><line x1="12" y1="16" x2="12" y2="12"></line><line x1="12" y1="8" x2="12.01" y2="8"></line></svg>`,
        warning: `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"></path><line x1="12" y1="9" x2="12" y2="13"></line><line x1="12" y1="17" x2="12.01" y2="17"></line></svg>`
    };

    // 显示浮窗
    function show(message, type = 'info', options = {}) {
        const container = getContainer();
        const toast = document.createElement('div');
        toast.className = `toast ${type}`;
        
        const duration = options.duration !== undefined ? options.duration : 3000;
        const title = options.title || '';
        const showClose = options.showClose !== false;
        
        let html = `<div class="toast-icon">${icons[type] || icons.info}</div>`;
        html += `<div class="toast-content">`;
        if (title) {
            html += `<span class="toast-title">${escapeHtml(title)}</span>`;
        }
        html += `<span class="toast-message">${escapeHtml(message)}</span></div>`;
        
        if (showClose) {
            html += `<button class="toast-close" aria-label="关闭"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><line x1="18" y1="6" x2="6" y2="18"></line><line x1="6" y1="6" x2="18" y2="18"></line></svg></button>`;
        }
        
        if (duration > 0) {
            const progress = document.createElement('div');
            progress.className = 'toast-progress';
            progress.style.animationDuration = `${duration}ms`;
            setTimeout(() => {
                if (toast.parentNode) {
                    progress.remove();
                }
            }, 0);
            html = html + progress.outerHTML;
        }
        
        toast.innerHTML = html;
        container.appendChild(toast);
        
        // 关闭按钮事件
        const closeBtn = toast.querySelector('.toast-close');
        if (closeBtn) {
            closeBtn.addEventListener('click', () => dismiss(toast));
        }
        
        // 自动关闭
        let timer = null;
        if (duration > 0) {
            timer = setTimeout(() => dismiss(toast), duration);
        }
        
        // 鼠标悬停暂停自动关闭
        toast.addEventListener('mouseenter', () => {
            if (timer) clearTimeout(timer);
        });
        toast.addEventListener('mouseleave', () => {
            if (duration > 0) {
                timer = setTimeout(() => dismiss(toast), duration);
            }
        });
        
        return toast;
    }
    
    // 关闭浮窗
    function dismiss(toast) {
        if (!toast || !toast.parentNode) return;
        toast.classList.add('removing');
        setTimeout(() => {
            if (toast.parentNode) {
                toast.parentNode.removeChild(toast);
            }
        }, 300);
    }
    
    // 关闭所有浮窗
    function dismissAll() {
        const container = document.querySelector('.toast-container');
        if (!container) return;
        const toasts = container.querySelectorAll('.toast');
        toasts.forEach((toast, index) => {
            setTimeout(() => dismiss(toast), index * 100);
        });
    }
    
    // HTML 转义
    function escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
    
    return {
        show: show,
        success: (msg, opts) => show(msg, 'success', opts),
        error: (msg, opts) => show(msg, 'error', opts),
        info: (msg, opts) => show(msg, 'info', opts),
        warning: (msg, opts) => show(msg, 'warning', opts),
        dismiss: dismiss,
        dismissAll: dismissAll
    };
})();

// 确认对话框系统
const Confirm = (function() {
    // SVG 图标
    const icons = {
        danger: `<svg viewBox="0 0 24 24" fill="none" stroke="#EF4444" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"></path><line x1="12" y1="9" x2="12" y2="13"></line><line x1="12" y1="17" x2="12.01" y2="17"></line></svg>`,
        warning: `<svg viewBox="0 0 24 24" fill="none" stroke="#F59E0B" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"></circle><line x1="12" y1="8" x2="12" y2="12"></line><line x1="12" y1="16" x2="12.01" y2="16"></line></svg>`,
        info: `<svg viewBox="0 0 24 24" fill="none" stroke="#0891B2" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"></circle><line x1="12" y1="16" x2="12" y2="12"></line><line x1="12" y1="8" x2="12.01" y2="8"></line></svg>`
    };
    
    function show(message, options = {}) {
        return new Promise((resolve) => {
            const type = options.type || 'danger';
            const title = options.title || '确认操作';
            const confirmText = options.confirmText || '确定';
            const cancelText = options.cancelText || '取消';
            
            // 创建遮罩层
            const overlay = document.createElement('div');
            overlay.className = 'confirm-overlay';
            
            // 创建对话框
            const dialog = document.createElement('div');
            dialog.className = `confirm-dialog ${type}`;
            
            dialog.innerHTML = `
                <div class="confirm-header">
                    <div class="confirm-icon">${icons[type] || icons.info}</div>
                    <div class="confirm-title">${title}</div>
                </div>
                <div class="confirm-body">${message}</div>
                <div class="confirm-footer">
                    <button class="confirm-btn confirm-btn-cancel">${cancelText}</button>
                    <button class="confirm-btn confirm-btn-confirm ${type}">${confirmText}</button>
                </div>
            `;
            
            overlay.appendChild(dialog);
            document.body.appendChild(overlay);
            
            // 按钮事件
            const cancelBtn = dialog.querySelector('.confirm-btn-cancel');
            const confirmBtn = dialog.querySelector('.confirm-btn-confirm');
            
            cancelBtn.addEventListener('click', () => {
                cleanup();
                resolve(false);
            });
            
            confirmBtn.addEventListener('click', () => {
                cleanup();
                resolve(true);
            });
            
            // 点击遮罩层关闭
            overlay.addEventListener('click', (e) => {
                if (e.target === overlay && !options.backdrop) {
                    cleanup();
                    resolve(false);
                }
            });
            
            // ESC 键关闭
            const handleEsc = (e) => {
                if (e.key === 'Escape') {
                    cleanup();
                    resolve(false);
                }
            };
            document.addEventListener('keydown', handleEsc);
            
            function cleanup() {
                overlay.classList.add('removing');
                dialog.classList.add('removing');
                setTimeout(() => {
                    if (overlay.parentNode) {
                        overlay.parentNode.removeChild(overlay);
                    }
                    document.removeEventListener('keydown', handleEsc);
                }, 200);
            }
        });
    }
    
    return {
        show: show
    };
})();

// 显示浮窗通知（便捷方法）
function showToast(msg, type = 'info', options = {}) {
    return Toast.show(msg, type, options);
}

// 输入对话框系统
const Prompt = (function() {
    // SVG 图标
    const icons = {
        info: `<svg viewBox="0 0 24 24" fill="none" stroke="#0891B2" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"></circle><line x1="12" y1="16" x2="12" y2="12"></line><line x1="12" y1="8" x2="12.01" y2="8"></line></svg>`,
        success: `<svg viewBox="0 0 24 24" fill="none" stroke="#22C55E" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"></path><polyline points="22 4 12 14.01 9 11.01"></polyline></svg>`
    };
    
    function show(message, options = {}) {
        return new Promise((resolve) => {
            const type = options.type || 'info';
            const title = options.title || '输入';
            const confirmText = options.confirmText || '确定';
            const cancelText = options.cancelText || '取消';
            const placeholder = options.placeholder || '';
            const defaultValue = options.defaultValue || '';
            const maxLength = options.maxLength || 255;
            
            // 创建遮罩层
            const overlay = document.createElement('div');
            overlay.className = 'confirm-overlay';
            
            // 创建对话框
            const dialog = document.createElement('div');
            dialog.className = `confirm-dialog prompt-dialog ${type}`;
            
            dialog.innerHTML = `
                <div class="confirm-header">
                    <div class="confirm-icon">${icons[type] || icons.info}</div>
                    <div class="confirm-title">${title}</div>
                </div>
                <div class="confirm-body">
                    <div class="prompt-message">${message}</div>
                    <div class="prompt-input-wrapper">
                        <input type="text" class="prompt-input" placeholder="${escapeHtml(placeholder)}" maxlength="${maxLength}" value="${escapeHtml(defaultValue)}">
                    </div>
                </div>
                <div class="confirm-footer">
                    <button class="confirm-btn confirm-btn-cancel">${cancelText}</button>
                    <button class="confirm-btn confirm-btn-confirm ${type}">${confirmText}</button>
                </div>
            `;
            
            overlay.appendChild(dialog);
            document.body.appendChild(overlay);
            
            // 获取输入框
            const input = dialog.querySelector('.prompt-input');
            
            // 聚焦输入框
            setTimeout(() => {
                input.focus();
                if (defaultValue) {
                    input.select();
                }
            }, 100);
            
            // 按钮事件
            const cancelBtn = dialog.querySelector('.confirm-btn-cancel');
            const confirmBtn = dialog.querySelector('.confirm-btn-confirm');
            
            cancelBtn.addEventListener('click', () => {
                cleanup();
                resolve(null);
            });
            
            confirmBtn.addEventListener('click', () => {
                const value = input.value.trim();
                cleanup();
                resolve(value);
            });
            
            // 回车键确认
            input.addEventListener('keydown', (e) => {
                if (e.key === 'Enter') {
                    const value = input.value.trim();
                    cleanup();
                    resolve(value);
                }
            });
            
            // 点击遮罩层关闭
            overlay.addEventListener('click', (e) => {
                if (e.target === overlay && !options.backdrop) {
                    cleanup();
                    resolve(null);
                }
            });
            
            // ESC 键关闭
            const handleEsc = (e) => {
                if (e.key === 'Escape') {
                    cleanup();
                    resolve(null);
                }
            };
            document.addEventListener('keydown', handleEsc);
            
            function cleanup() {
                overlay.classList.add('removing');
                dialog.classList.add('removing');
                setTimeout(() => {
                    if (overlay.parentNode) {
                        overlay.parentNode.removeChild(overlay);
                    }
                    document.removeEventListener('keydown', handleEsc);
                }, 200);
            }
            
            // HTML 转义函数
            function escapeHtml(text) {
                const div = document.createElement('div');
                div.textContent = text;
                return div.innerHTML;
            }
        });
    }
    
    return {
        show: show
    };
})();

// 显示浮窗通知（便捷方法）
function showToast(msg, type = 'info', options = {}) {
    return Toast.show(msg, type, options);
}

// 显示确认对话框（便捷方法）
function showConfirm(msg, options = {}) {
    return Confirm.show(msg, options);
}

// 显示输入对话框（便捷方法）
function showPrompt(msg, options = {}) {
    return Prompt.show(msg, options);
}

// 格式化文件大小
function formatFileSize(bytes) {
    if (bytes < 1024) return bytes + ' B';
    if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(2) + ' KB';
    if (bytes < 1024 * 1024 * 1024) return (bytes / (1024 * 1024)).toFixed(2) + ' MB';
    return (bytes / (1024 * 1024 * 1024)).toFixed(2) + ' GB';
}

// 初始化拖拽上传
// uploadElement: 上传区域元素
// fileInput: 文件输入元素
// callback: 文件选择后的回调函数(file)
function initDragAndDrop(uploadElement, fileInput, callback) {
    if (!uploadElement || !fileInput) return;
    
    fileInput.addEventListener('change', function(e) {
        const file = e.target.files[0];
        if (file && callback) {
            callback(file);
        }
    });
    
    uploadElement.addEventListener('dragover', function(e) {
        e.preventDefault();
        e.stopPropagation();
        uploadElement.classList.add('drag-over');
    });
    
    uploadElement.addEventListener('dragleave', function(e) {
        e.preventDefault();
        e.stopPropagation();
        uploadElement.classList.remove('drag-over');
    });
    
    uploadElement.addEventListener('drop', function(e) {
        e.preventDefault();
        e.stopPropagation();
        uploadElement.classList.remove('drag-over');
        
        const files = e.dataTransfer.files;
        if (files.length > 0 && callback) {
            callback(files[0]);
        }
    });
}

// 显示文件信息
// contentElement: 内容元素
// fileInfoElement: 文件信息元素
// fileNameElement: 文件名元素
// fileSizeElement: 文件大小元素
// file: 文件对象
function displayFileInfo(contentElement, fileInfoElement, fileNameElement, fileSizeElement, file) {
    if (contentElement) contentElement.style.display = 'none';
    if (fileInfoElement) {
        fileInfoElement.style.display = 'block';
        if (fileNameElement) fileNameElement.textContent = file.name;
        if (fileSizeElement) fileSizeElement.textContent = formatFileSize(file.size);
    }
}

// 重置上传区域
// contentElement: 内容元素
// fileInfoElement: 文件信息元素
// fileInput: 文件输入元素
function resetUploadArea(contentElement, fileInfoElement, fileInput) {
    if (contentElement) contentElement.style.display = '';
    if (fileInfoElement) fileInfoElement.style.display = 'none';
    if (fileInput) fileInput.value = '';
}

// 初始化进度条动画
function initProgressAnimation() {
    const progressBars = document.querySelectorAll('.progress-bar::after');
    // CSS动画已定义，这里可以添加额外的逻辑
}

// 通用fetch错误处理
function handleFetchError(err) {
    console.error('请求失败:', err);
    return '请求失败: ' + (err.message || err);
}

// ==================== 页面过渡效果 ====================

// 页面切换处理
(function initPageTransitions() {
    // 注意：页面容器现在通过HTML中的data-animate属性直接控制动画
    // 不需要JS动态初始化，避免闪烁问题
    
    // 拦截所有内部链接的点击事件
    document.addEventListener('click', function(e) {
        const link = e.target.closest('a');
        
        // 只处理内部链接（以/开头或相对路径）
        if (!link) return;
        const href = link.getAttribute('href');
        if (!href || href.startsWith('#') || href.startsWith('javascript:') || href.startsWith('http')) return;
        
        // 如果是下载链接，不应用过渡效果
        if (link.hasAttribute('download') || href.startsWith('/download?')) return;
        
        // 阻止默认跳转
        e.preventDefault();
        
        // 添加淡出动画
        const containers = document.querySelectorAll('.container');
        containers.forEach(container => {
            container.classList.remove('page-container');
            container.classList.add('page-fade-out');
        });
        
        // 动画完成后跳转
        setTimeout(function() {
            window.location.href = href;
        }, 150);
    });
})();