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

// ==================== WebSocket 日志系统 ====================

// 全局变量
let ws = null;
let wsReconnectTimer = null;
let wsReconnectAttempts = 0;
const WS_MAX_RECONNECT_ATTEMPTS = 10;
const WS_RECONNECT_DELAY = 3000;

// 当前过滤级别：0=全部, 1=ERROR, 2=WARN+, 3=INFO+, 4=DEBUG+, 5=VERBOSE+
let currentFilterLevel = 3;

// 存储所有日志条目（用于过滤）
let allLogs = [];

// 存储未读日志
let unreadLogs = [];

// ==================== WebSocket 请求管理系统 ====================

// 请求回调映射：{ requestId: { resolve, reject, timeout, action } }
let wsRequestCallbacks = {};
// 请求ID计数器
let wsRequestIdCounter = 1;

// 事件监听器映射：{ action: [callback1, callback2, ...] }
let wsEventListeners = {};

/**
 * 发送 WebSocket 请求
 * @param {string} action - 动作类型
 * @param {object} data - 请求数据
 * @param {number} timeout - 超时时间（毫秒），默认 30000
 * @returns {Promise} 返回响应数据的 Promise
 */
function sendWsRequest(action, data = {}, timeout = 30000) {
    return new Promise((resolve, reject) => {
        // 检查 WebSocket 连接
        if (!ws || ws.readyState !== WebSocket.OPEN) {
            reject(new Error('WebSocket 未连接'));
            return;
        }

        const requestId = wsRequestIdCounter++;

        // 注册回调
        wsRequestCallbacks[requestId] = {
            resolve,
            reject,
            timeout,
            action
        };

        // 设置超时
        const timer = setTimeout(() => {
            if (wsRequestCallbacks[requestId]) {
                delete wsRequestCallbacks[requestId];
                reject(new Error(`请求超时 (${timeout}ms)`));
            }
        }, timeout);

        wsRequestCallbacks[requestId].timeoutTimer = timer;

        // 发送请求
        const message = {
            action,
            requestId,
            ...data
        };

        try {
            ws.send(JSON.stringify(message));
        } catch (e) {
            clearTimeout(timer);
            delete wsRequestCallbacks[requestId];
            reject(new Error('发送请求失败: ' + e.message));
        }
    });
}

/**
 * 注册事件监听器
 * @param {string} action - 动作类型
 * @param {function} callback - 回调函数
 */
function onWsEvent(action, callback) {
    if (!wsEventListeners[action]) {
        wsEventListeners[action] = [];
    }
    wsEventListeners[action].push(callback);
}

/**
 * 移除事件监听器
 * @param {string} action - 动作类型
 * @param {function} callback - 回调函数
 */
function offWsEvent(action, callback) {
    if (wsEventListeners[action]) {
        wsEventListeners[action] = wsEventListeners[action].filter(cb => cb !== callback);
    }
}

// ==================== 日志浮窗系统 ====================

// 浮窗状态
let logWindowVisible = false;
let logWindowMinimized = false;
let isDragging = false;
let dragOffset = { x: 0, y: 0 };

// 初始化日志浮窗
function initLogWindow() {
    // 从 localStorage 读取偏好
    const savedState = localStorage.getItem('logWindowState');
    if (savedState) {
        try {
            const state = JSON.parse(savedState);
            if (state.visible) {
                showLogWindow();
                if (state.minimized) {
                    minimizeLogWindow();
                }
                // 恢复位置
                if (state.position) {
                    const logWindow = document.getElementById('log-float-window');
                    if (logWindow) {
                        logWindow.style.right = 'auto';
                        logWindow.style.bottom = 'auto';
                        logWindow.style.left = state.position.x + 'px';
                        logWindow.style.top = state.position.y + 'px';
                    }
                }
            }
        } catch (e) {
            console.error('读取日志窗口状态失败:', e);
        }
    }

    // 初始化拖拽
    initLogWindowDrag();

    // 初始化键盘快捷键
    initLogKeyboardShortcuts();
}

// 显示/隐藏日志浮窗
function toggleLogWindow() {
    const logWindow = document.getElementById('log-float-window');
    const toggleBtn = document.getElementById('log-toggle-btn');
    
    if (!logWindow) return;

    if (logWindowVisible) {
        // 隐藏浮窗
        logWindow.classList.add('closing');
        setTimeout(() => {
            logWindow.style.display = 'none';
            logWindow.classList.remove('closing');
            toggleBtn.style.display = 'flex';
        }, 200);
        logWindowVisible = false;
    } else {
        // 显示浮窗
        logWindow.style.display = 'flex';
        logWindowVisible = true;
        toggleBtn.style.display = 'none';
        
        // 清空未读日志
        unreadLogs = [];
        updateLogBadge();
    }

    // 保存状态
    saveLogWindowState();
}

// 显示日志浮窗
function showLogWindow() {
    const logWindow = document.getElementById('log-float-window');
    const toggleBtn = document.getElementById('log-toggle-btn');
    
    if (!logWindow || logWindowVisible) return;

    logWindow.style.display = 'flex';
    logWindowVisible = true;
    toggleBtn.style.display = 'none';
    
    // 清空未读日志
    unreadLogs = [];
    updateLogBadge();

    // 自动渲染所有日志
    renderLogs();

    // 保存状态
    saveLogWindowState();
}

// 最小化/展开日志浮窗
function minimizeLogWindow() {
    const logWindow = document.getElementById('log-float-window');
    const minimizeBtn = document.querySelector('.log-float-minimize-btn');
    
    if (!logWindow) return;

    logWindowMinimized = !logWindowMinimized;
    
    if (logWindowMinimized) {
        logWindow.classList.add('minimized');
        if (minimizeBtn) {
            minimizeBtn.innerHTML = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="18 15 12 9 6 15"></polyline></svg>`;
        }
    } else {
        logWindow.classList.remove('minimized');
        if (minimizeBtn) {
            minimizeBtn.innerHTML = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><line x1="5" y1="12" x2="19" y2="12"></line></svg>`;
        }
    }

    // 保存状态
    saveLogWindowState();
}

// 获取未读日志数量（符合当前过滤级别）
function getUnreadLogCount() {
    return unreadLogs.filter(log => shouldShowLog(log.level, currentFilterLevel)).length;
}

// 更新新日志徽章
function updateLogBadge() {
    const badge = document.getElementById('log-toggle-badge');
    if (!badge) return;

    const count = getUnreadLogCount();
    if (count > 0) {
        badge.textContent = count > 99 ? '99+' : count;
        badge.style.display = 'flex';
    } else {
        badge.style.display = 'none';
    }
}

// 初始化拖拽功能
function initLogWindowDrag() {
    const logWindow = document.getElementById('log-float-window');
    const header = document.getElementById('log-float-header');
    
    if (!logWindow || !header) return;

    header.addEventListener('mousedown', startDrag);
    header.addEventListener('touchstart', startDrag, { passive: false });
}

function startDrag(e) {
    const logWindow = document.getElementById('log-float-window');
    if (!logWindow) return;

    // 如果是最小化状态，不允许拖拽
    if (logWindowMinimized) return;

    e.preventDefault();
    isDragging = true;

    // 获取触点位置（支持鼠标和触摸）
    const clientX = e.type === 'touchstart' ? e.touches[0].clientX : e.clientX;
    const clientY = e.type === 'touchstart' ? e.touches[0].clientY : e.clientY;

    // 获取窗口当前位置
    const rect = logWindow.getBoundingClientRect();
    dragOffset.x = clientX - rect.left;
    dragOffset.y = clientY - rect.top;

    // 添加移动和结束事件监听
    document.addEventListener('mousemove', drag);
    document.addEventListener('mouseup', stopDrag);
    document.addEventListener('touchmove', drag, { passive: false });
    document.addEventListener('touchend', stopDrag);

    // 改变光标
    document.body.style.cursor = 'move';
}

function drag(e) {
    if (!isDragging) return;

    e.preventDefault();

    const logWindow = document.getElementById('log-float-window');
    if (!logWindow) return;

    // 获取触点位置
    const clientX = e.type === 'touchmove' ? e.touches[0].clientX : e.clientX;
    const clientY = e.type === 'touchmove' ? e.touches[0].clientY : e.clientY;

    // 计算新位置
    let newX = clientX - dragOffset.x;
    let newY = clientY - dragOffset.y;

    // 限制在视口内
    const maxX = window.innerWidth - logWindow.offsetWidth;
    const maxY = window.innerHeight - logWindow.offsetHeight;

    newX = Math.max(0, Math.min(newX, maxX));
    newY = Math.max(0, Math.min(newY, maxY));

    // 清除 right 和 bottom，使用 left 和 top
    logWindow.style.right = 'auto';
    logWindow.style.bottom = 'auto';
    logWindow.style.left = newX + 'px';
    logWindow.style.top = newY + 'px';
}

function stopDrag() {
    if (!isDragging) return;

    isDragging = false;
    document.body.style.cursor = '';

    // 移除事件监听
    document.removeEventListener('mousemove', drag);
    document.removeEventListener('mouseup', stopDrag);
    document.removeEventListener('touchmove', drag);
    document.removeEventListener('touchend', stopDrag);

    // 保存位置
    saveLogWindowState();
}

// 初始化键盘快捷键
function initLogKeyboardShortcuts() {
    document.addEventListener('keydown', function(e) {
        // 按 'L' 键切换日志窗口（当没有聚焦在输入框时）
        if (e.key === 'l' || e.key === 'L') {
            const activeElement = document.activeElement;
            const isInputFocused = activeElement && 
                (activeElement.tagName === 'INPUT' || 
                 activeElement.tagName === 'TEXTAREA' || 
                 activeElement.isContentEditable);

            if (!isInputFocused) {
                e.preventDefault();
                toggleLogWindow();
            }
        }

        // 按 Escape 键关闭日志窗口
        if (e.key === 'Escape' && logWindowVisible) {
            toggleLogWindow();
        }
    });
}

// 保存日志窗口状态
function saveLogWindowState() {
    const logWindow = document.getElementById('log-float-window');
    if (!logWindow) return;

    const rect = logWindow.getBoundingClientRect();
    
    const state = {
        visible: logWindowVisible,
        minimized: logWindowMinimized,
        position: {
            x: rect.left,
            y: rect.top
        }
    };

    localStorage.setItem('logWindowState', JSON.stringify(state));
}

// 页面加载时初始化日志浮窗
document.addEventListener('DOMContentLoaded', function() {
    initLogLevel();  // 初始化日志等级
    initLogWindow();
});

// 初始化 WebSocket 连接
function initWebSocket() {
    // 获取当前页面的协议（ws 或 wss）
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    
    try {
        ws = new WebSocket(wsUrl);
        
        ws.onopen = function() {
            console.log('WebSocket 连接成功');
            wsReconnectAttempts = 0;
            updateConnectionStatus(true);
            // 触发自定义事件，通知其他模块 WebSocket 已连接
            document.dispatchEvent(new CustomEvent('ws-connected'));
        };
        
        ws.onmessage = function(event) {
            try {
                const data = JSON.parse(event.data);
                
                // 处理请求响应（带 requestId）
                if (data.requestId && wsRequestCallbacks[data.requestId]) {
                    const callback = wsRequestCallbacks[data.requestId];
                    clearTimeout(callback.timeoutTimer);
                    delete wsRequestCallbacks[data.requestId];

                    if (data.success) {
                        callback.resolve(data.data || null);
                    } else {
                        callback.reject(new Error(data.data?.message || '请求失败'));
                    }
                    return;
                }
                
                // 处理日志数据（兼容原有日志系统）
                if (Array.isArray(data)) {
                    // 历史日志数组
                    data.forEach(log => addLogEntry(log));
                    return;
                }

                // 单条日志（有 level 字段）
                if (data.level !== undefined) {
                    addLogEntry(data);
                    return;
                }

                // 处理事件通知（带 action 字段）
                if (data.action) {
                    const action = data.action;
                    if (wsEventListeners[action]) {
                        wsEventListeners[action].forEach(callback => {
                            try {
                                callback(data);
                            } catch (e) {
                                console.error('事件回调执行失败:', e);
                            }
                        });
                    }
                }
            } catch (e) {
                console.error('解析 WebSocket 消息失败:', e);
            }
        };
        
        ws.onerror = function(error) {
            console.error('WebSocket 错误:', error);
            updateConnectionStatus(false);
        };
        
        ws.onclose = function(event) {
            console.log('WebSocket 连接关闭:', event.code, event.reason);
            updateConnectionStatus(false);
            
            // 尝试重连
            if (wsReconnectAttempts < WS_MAX_RECONNECT_ATTEMPTS) {
                wsReconnectAttempts++;
                console.log(`尝试重连 (${wsReconnectAttempts}/${WS_MAX_RECONNECT_ATTEMPTS})...`);
                
                wsReconnectTimer = setTimeout(() => {
                    initWebSocket();
                }, WS_RECONNECT_DELAY);
            }
        };
        
    } catch (e) {
        console.error('WebSocket 初始化失败:', e);
    }
}

// 关闭 WebSocket 连接
function closeWebSocket() {
    if (wsReconnectTimer) {
        clearTimeout(wsReconnectTimer);
        wsReconnectTimer = null;
    }
    
    if (ws) {
        ws.close();
        ws = null;
    }
    
    wsReconnectAttempts = WS_MAX_RECONNECT_ATTEMPTS; // 防止自动重连
}

// 判断日志是否应该显示
function shouldShowLog(logLevel, filterLevel) {
    // filterLevel: 0=全部, 1=ERROR, 2=WARN+, 3=INFO+, 4=DEBUG+, 5=VERBOSE+
    // logLevel: 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG, 4=VERBOSE
    
    // 0=全部 或 5=VERBOSE+，显示所有日志
    if (filterLevel === 0 || filterLevel === 5) {
        return true;
    }
    
    // filterLevel=1(ERROR) -> 只显示 level 0 (ERROR)
    // filterLevel=2(WARN+) -> 显示 level 0-1 (ERROR + WARN)
    // filterLevel=3(INFO+) -> 显示 level 0-2 (ERROR + WARN + INFO)
    // filterLevel=4(DEBUG+) -> 显示 level 0-3 (ERROR + WARN + INFO + DEBUG)
    return logLevel <= filterLevel - 1;
}

// 添加日志条目
function addLogEntry(logData) {
    // 存储到数组
    allLogs.push(logData);
    
    // 限制日志数量（最多保留 500 条）
    if (allLogs.length > 500) {
        allLogs.shift();
    }
    
    // 存储未读日志
    unreadLogs.push(logData);
    if (unreadLogs.length > 500) {
        unreadLogs.shift();
    }
    
    // 如果浮窗隐藏，更新徽章
    if (!logWindowVisible) {
        updateLogBadge();
    }
    
    // 根据过滤级别决定是否显示
    if (shouldShowLog(logData.level, currentFilterLevel) && logWindowVisible && !logWindowMinimized) {
        renderLogEntry(logData);
    }
}

// 渲染单条日志
function renderLogEntry(logData) {
    const logOutput = document.getElementById('log-output');
    if (!logOutput) return;
    
    // 创建日志元素
    const entry = document.createElement('div');
    entry.className = `log-entry ${getLogClass(logData.level)}`;
    
    // 格式化时间戳
    const time = formatTimestamp(logData.timestamp);
    
    entry.innerHTML = `
        <span class="log-time">${time}</span>
        <span class="log-tag">${logData.tag}</span>
        <span class="log-message">${escapeHtml(logData.message)}</span>
    `;
    
    logOutput.appendChild(entry);
    
    // 自动滚动到底部
    scrollToBottom();
}

// 重新渲染所有日志（用于过滤级别变化）
function renderLogs() {
    const logOutput = document.getElementById('log-output');
    if (!logOutput) return;
    
    // 清空当前显示
    logOutput.innerHTML = '';
    
    // 根据当前过滤级别重新渲染
    allLogs.forEach(logData => {
        if (shouldShowLog(logData.level, currentFilterLevel)) {
            renderLogEntry(logData);
        }
    });
    
    // 滚动到底部
    scrollToBottom();
}

// 获取日志级别的 CSS 类名
function getLogClass(level) {
    switch (level) {
        case 0: return 'error';
        case 1: return 'warn';
        case 2: return 'info';
        case 3: return 'debug';
        case 4: return 'verbose';
        default: return 'info';
    }
}

// 格式化时间戳
function formatTimestamp(timestamp) {
    if (!timestamp) return '';
    
    const date = new Date(timestamp);
    const hours = String(date.getHours()).padStart(2, '0');
    const minutes = String(date.getMinutes()).padStart(2, '0');
    const seconds = String(date.getSeconds()).padStart(2, '0');
    
    return `${hours}:${minutes}:${seconds}`;
}

// 滚动到日志底部
function scrollToBottom() {
    const logOutput = document.getElementById('log-output');
    if (logOutput) {
        logOutput.scrollTop = logOutput.scrollHeight;
    }
}

// 初始化日志等级（从 localStorage 读取）
function initLogLevel() {
    const savedLevel = localStorage.getItem('logFilterLevel');
    if (savedLevel !== null) {
        currentFilterLevel = parseInt(savedLevel);
    }
    
    // 更新下拉框选中状态
    const select = document.getElementById('log-filter');
    if (select) {
        select.value = currentFilterLevel;
    }
}

// 设置日志过滤级别
function setLogFilter(level) {
    currentFilterLevel = parseInt(level);
    console.log('设置日志过滤级别:', currentFilterLevel);
    
    // 保存到 localStorage
    localStorage.setItem('logFilterLevel', currentFilterLevel);
    
    // 更新下拉框选中状态
    const select = document.getElementById('log-filter');
    if (select) {
        select.value = currentFilterLevel;
    }
    
    // 重新渲染日志
    renderLogs();
    
    // 更新徽章
    updateLogBadge();
}

// 清空日志
function clearLogs() {
    const logOutput = document.getElementById('log-output');
    if (logOutput) {
        logOutput.innerHTML = '';
    }
    
    // 清空存储的日志
    allLogs = [];
    
    console.log('日志已清空');
}

// 更新连接状态（可选：在 UI 上显示连接状态）
function updateConnectionStatus(connected) {
    // 可以在日志容器上添加连接状态指示
    // 这里只是示例，可以根据需要实现
    const logContainer = document.querySelector('.log-container');
    if (logContainer) {
        if (connected) {
            logContainer.style.borderColor = 'var(--color-success)';
        } else {
            logContainer.style.borderColor = 'var(--color-border)';
        }
    }
}

// HTML 转义
function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

// 页面加载时初始化 WebSocket
document.addEventListener('DOMContentLoaded', function() {
    // 延迟初始化，等待 DOM 完全加载
    setTimeout(() => {
        initWebSocket();
    }, 500);
});

// 页面卸载时关闭 WebSocket
window.addEventListener('beforeunload', function() {
    closeWebSocket();
});