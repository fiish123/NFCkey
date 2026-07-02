// 公共JavaScript函数库

// ===== ICONS — 全局唯一 SVG 图标来源 =====
// 所有 stroke=currentColor；颜色由各表面的 CSS 按变体设定。
// 取代此前 Toast/Confirm/Prompt/LogWindow 各自内联的重复 SVG 字符串。
const ICONS = {
    // 状态图标（圆形/三角几何）
    success:     '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"></path><polyline points="22 4 12 14.01 9 11.01"></polyline></svg>',
    error:       '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"></circle><line x1="15" y1="9" x2="9" y2="15"></line><line x1="9" y1="9" x2="15" y2="15"></line></svg>',
    info:        '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"></circle><line x1="12" y1="16" x2="12" y2="12"></line><line x1="12" y1="8" x2="12.01" y2="8"></line></svg>',
    warning:     '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"></path><line x1="12" y1="9" x2="12" y2="13"></line><line x1="12" y1="17" x2="12.01" y2="17"></line></svg>',
    alertCircle: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="10"></circle><line x1="12" y1="8" x2="12" y2="12"></line><line x1="12" y1="16" x2="12.01" y2="16"></line></svg>',
    close:       '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><line x1="18" y1="6" x2="6" y2="18"></line><line x1="6" y1="6" x2="18" y2="18"></line></svg>',
    // 日志浮窗 UI 图标（装饰性，aria-hidden）
    logToggle:   '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"></path><polyline points="14 2 14 8 20 8"></polyline><line x1="16" y1="13" x2="8" y2="13"></line><line x1="16" y1="17" x2="8" y2="17"></line><polyline points="10 9 9 9 8 9"></polyline></svg>',
    logTitle:    '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"></path><polyline points="14 2 14 8 20 8"></polyline></svg>',
    minimize:    '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><line x1="5" y1="12" x2="19" y2="12"></line></svg>',
    restore:     '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="18 15 12 9 6 15"></polyline></svg>'
};

// ===== _Dialog — 共享对话框基座 =====
// Confirm 与 Prompt 同为 .confirm-overlay 上的模态对话框：共用
// overlay 建拆 / ESC / 遮罩点击关闭 / 焦点回退 / removing→detach 淡出序列。
// Toast 仅复用 fadeRemove（逐条 toast 的淡出拆除）。
const _Dialog = {
    // 单节点淡出拆除：加 .removing 触发 CSS 动画，延时后移除。
    fadeRemove(node, delay = 200) {
        if (!node || !node.parentNode) return;
        node.classList.add('removing');
        setTimeout(() => { if (node.parentNode) node.parentNode.removeChild(node); }, delay);
    },

    // 把 dialog 挂载到新建的 .confirm-overlay；接管 ESC + 遮罩点击；
    // close(result) 触发一次性淡出拆除并把焦点还给 trigger，再经 onResult
    // 把 result 回传给调用方（Promise resolve）。dismissValue 是 ESC/遮罩
    // 关闭时回传的值（Confirm=false，Prompt=null，保持各自契约）。
    // 返回 { overlay, close } — 按钮处理器自行决定传给 close 的值。
    modal({ dialog, onResult, allowBackdropClose = true, dismissValue = false, trigger }) {
        const overlay = document.createElement('div');
        overlay.className = 'confirm-overlay';
        overlay.appendChild(dialog);
        document.body.appendChild(overlay);

        const restoreFocusTo = trigger || document.activeElement;
        let done = false;

        function close(result) {
            if (done) return;
            done = true;
            document.removeEventListener('keydown', onKeydown);
            overlay.removeEventListener('click', onBackdrop);
            overlay.classList.add('removing');
            dialog.classList.add('removing');
            if (restoreFocusTo && typeof restoreFocusTo.focus === 'function') {
                restoreFocusTo.focus();
            }
            setTimeout(() => { if (overlay.parentNode) overlay.parentNode.removeChild(overlay); }, 200);
            if (typeof onResult === 'function') onResult(result);
        }

        function onKeydown(e) { if (e.key === 'Escape') close(dismissValue); }
        function onBackdrop(e) {
            if (allowBackdropClose && e.target === overlay) close(dismissValue);
        }

        document.addEventListener('keydown', onKeydown);
        overlay.addEventListener('click', onBackdrop);
        return { overlay, close };
    }
};

// 浮窗通知系统
const Toast = (function() {
    // 创建或获取容器
    function getContainer() {
        let container = document.querySelector('.toast-container');
        if (!container) {
            container = document.createElement('div');
            container.className = 'toast-container';
            container.setAttribute('aria-live', 'polite');
            document.body.appendChild(container);
        }
        return container;
    }

    // 显示浮窗
    function show(message, type = 'info', options = {}) {
        const container = getContainer();
        const toast = document.createElement('div');
        toast.className = `toast ${type}`;
        if (type === 'error' || type === 'warning') toast.setAttribute('role', 'alert');

        const duration = options.duration !== undefined ? options.duration : 3000;
        const title = options.title || '';
        const showClose = options.showClose !== false;

        let html = `<div class="toast-icon">${ICONS[type] || ICONS.info}</div>`;
        html += `<div class="toast-content">`;
        if (title) {
            html += `<span class="toast-title">${escapeHtml(title)}</span>`;
        }
        html += `<span class="toast-message">${escapeHtml(message)}</span></div>`;

        if (showClose) {
            html += `<button class="toast-close" aria-label="关闭">${ICONS.close}</button>`;
        }

        if (duration > 0) {
            html += '<div class="toast-progress"></div>';
        }

        toast.innerHTML = html;
        container.appendChild(toast);
        // 可见堆叠上限 4 条，FIFO 关闭（走 removing 动画）
        const stack = container.querySelectorAll('.toast:not(.removing)');
        for (let i = 0; i < stack.length - 4; i++) dismiss(stack[i]);

        // 关闭按钮事件
        const closeBtn = toast.querySelector('.toast-close');
        if (closeBtn) {
            closeBtn.addEventListener('click', () => dismiss(toast));
        }

        // 自动关闭 + 悬停暂停（追踪剩余时间，避免 mouseleave 重置寿命导致脱节）
        let timer = null;
        let remaining = duration;
        let lastResume = Date.now();
        if (duration > 0) {
            timer = setTimeout(() => dismiss(toast), duration);
        }

        toast.addEventListener('mouseenter', () => {
            if (timer) {
                clearTimeout(timer);
                timer = null;
                remaining = Math.max(0, remaining - (Date.now() - lastResume));
                if (remaining <= 0) { dismiss(toast); return; }
            }
        });
        toast.addEventListener('mouseleave', () => {
            if (duration > 0 && !timer && remaining > 0) {
                lastResume = Date.now();
                timer = setTimeout(() => dismiss(toast), remaining);
            }
        });

        return toast;
    }

    // 关闭浮窗（共享淡出拆除序列）
    function dismiss(toast) {
        _Dialog.fadeRemove(toast, 300);
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
    // 变体 → 图标（颜色由 .confirm-dialog.<type> .confirm-icon 的 CSS 设定）
    const typeIcon = {
        danger: ICONS.warning,       // 三角警示
        warning: ICONS.alertCircle,  // 圆形警示
        info: ICONS.info
    };

    function show(message, options = {}) {
        return new Promise((resolve) => {
            const type = options.type || 'danger';
            const title = options.title || '确认操作';
            const confirmText = options.confirmText || '确定';
            const cancelText = options.cancelText || '取消';

            // 创建对话框（overlay/ESC/遮罩/焦点/淡出 由 _Dialog.modal 接管）
            const dialog = document.createElement('div');
            dialog.className = `confirm-dialog ${type}`;
            dialog.innerHTML = `
                <div class="confirm-header">
                    <div class="confirm-icon">${typeIcon[type] || typeIcon.info}</div>
                    <div class="confirm-title">${title}</div>
                </div>
                <div class="confirm-body">${message}</div>
                <div class="confirm-footer">
                    <button class="confirm-btn confirm-btn-cancel">${cancelText}</button>
                    <button class="confirm-btn confirm-btn-confirm ${type}">${confirmText}</button>
                </div>
            `;

            const { close } = _Dialog.modal({
                dialog,
                onResult: resolve,
                allowBackdropClose: !options.backdrop
            });

            // 默认聚焦取消按钮（安全默认：不意外触发破坏性操作）
            const cancelBtn = dialog.querySelector('.confirm-btn-cancel');
            const confirmBtn = dialog.querySelector('.confirm-btn-confirm');
            setTimeout(() => { if (cancelBtn) cancelBtn.focus(); }, 0);

            cancelBtn.addEventListener('click', () => close(false));
            confirmBtn.addEventListener('click', () => close(true));
        });
    }

    return {
        show: show
    };
})();

// 输入对话框系统
const Prompt = (function() {
    // 变体 → 图标
    const typeIcon = {
        info: ICONS.info,
        success: ICONS.success
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

            // 创建对话框（overlay/ESC/遮罩/焦点/淡出 由 _Dialog.modal 接管）
            const dialog = document.createElement('div');
            dialog.className = `confirm-dialog prompt-dialog ${type}`;
            dialog.innerHTML = `
                <div class="confirm-header">
                    <div class="confirm-icon">${typeIcon[type] || typeIcon.info}</div>
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

            // dismissValue=null 保持 Prompt 的原有契约（取消/ESC/遮罩 → null）
            const { close } = _Dialog.modal({
                dialog,
                onResult: resolve,
                allowBackdropClose: !options.backdrop,
                dismissValue: null
            });

            const input = dialog.querySelector('.prompt-input');

            // 聚焦输入框（有默认值则全选）
            setTimeout(() => {
                input.focus();
                if (defaultValue) {
                    input.select();
                }
            }, 0);

            const cancelBtn = dialog.querySelector('.confirm-btn-cancel');
            const confirmBtn = dialog.querySelector('.confirm-btn-confirm');

            cancelBtn.addEventListener('click', () => close(null));
            confirmBtn.addEventListener('click', () => close(input.value.trim()));

            // 回车键确认
            input.addEventListener('keydown', (e) => {
                if (e.key === 'Enter') close(input.value.trim());
            });
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

function waitForWebSocketReady(timeout = 15000) {
    return new Promise((resolve, reject) => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            resolve();
            return;
        }

        let settled = false;
        let timeoutId = null;

        const cleanup = () => {
            if (timeoutId) {
                clearTimeout(timeoutId);
            }
            document.removeEventListener('ws-connected', handleConnected);
            document.removeEventListener('ws-state-changed', handleStateChanged);
        };

        const settle = (callback, value) => {
            if (settled) {
                return;
            }

            settled = true;
            cleanup();
            callback(value);
        };

        const handleConnected = () => {
            settle(resolve);
        };

        const handleStateChanged = (event) => {
            const detail = event.detail || {};
            if (detail.state === WS_STATE.DISCONNECTED) {
                settle(reject, new Error(detail.reason || 'WebSocket 连接失败'));
            }
        };

        timeoutId = setTimeout(() => {
            settle(reject, new Error('等待 WebSocket 连接超时'));
        }, timeout);

        document.addEventListener('ws-connected', handleConnected);
        document.addEventListener('ws-state-changed', handleStateChanged);
    });
}

function bindFilePickerTrigger(triggerElement, fileInput) {
    if (!triggerElement || !fileInput) return;

    triggerElement.addEventListener('click', function(e) {
        if (e.target !== fileInput) {
            fileInput.click();
        }
    });

    triggerElement.addEventListener('keydown', function(e) {
        if (e.key === 'Enter' || e.key === ' ') {
            e.preventDefault();
            fileInput.click();
        }
    });
}

// 格式化文件大小
function formatFileSize(bytes) {
    if (bytes < 1024) return bytes + ' B';
    if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(2) + ' KB';
    if (bytes < 1024 * 1024 * 1024) return (bytes / (1024 * 1024)).toFixed(2) + ' MB';
    return (bytes / (1024 * 1024 * 1024)).toFixed(2) + ' GB';
}

function bytesToHex(bytes) {
    return Array.from(bytes, byte => byte.toString(16).padStart(2, '0')).join('');
}

function rightRotate(value, amount) {
    return (value >>> amount) | (value << (32 - amount));
}

function computeSha256Fallback(buffer) {
    const words = [];
    const bytes = new Uint8Array(buffer);
    const bitLength = bytes.length * 8;
    const hash = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
    ];
    const k = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
    ];

    for (let i = 0; i < bytes.length; i++) {
        words[i >> 2] = (words[i >> 2] || 0) | (bytes[i] << (24 - (i % 4) * 8));
    }

    words[bytes.length >> 2] = (words[bytes.length >> 2] || 0) | (0x80 << (24 - (bytes.length % 4) * 8));
    words[(((bytes.length + 8) >> 6) << 4) + 14] = Math.floor(bitLength / 0x100000000);
    words[(((bytes.length + 8) >> 6) << 4) + 15] = bitLength >>> 0;

    const schedule = new Array(64);

    for (let offset = 0; offset < words.length; offset += 16) {
        for (let i = 0; i < 16; i++) {
            schedule[i] = words[offset + i] | 0;
        }

        for (let i = 16; i < 64; i++) {
            const s0 = rightRotate(schedule[i - 15], 7) ^ rightRotate(schedule[i - 15], 18) ^ (schedule[i - 15] >>> 3);
            const s1 = rightRotate(schedule[i - 2], 17) ^ rightRotate(schedule[i - 2], 19) ^ (schedule[i - 2] >>> 10);
            schedule[i] = (((schedule[i - 16] + s0) | 0) + ((schedule[i - 7] + s1) | 0)) | 0;
        }

        let a = hash[0];
        let b = hash[1];
        let c = hash[2];
        let d = hash[3];
        let e = hash[4];
        let f = hash[5];
        let g = hash[6];
        let h = hash[7];

        for (let i = 0; i < 64; i++) {
            const sum1 = rightRotate(e, 6) ^ rightRotate(e, 11) ^ rightRotate(e, 25);
            const choice = (e & f) ^ (~e & g);
            const temp1 = (((((h + sum1) | 0) + choice) | 0) + ((k[i] + schedule[i]) | 0)) | 0;
            const sum0 = rightRotate(a, 2) ^ rightRotate(a, 13) ^ rightRotate(a, 22);
            const majority = (a & b) ^ (a & c) ^ (b & c);
            const temp2 = (sum0 + majority) | 0;

            h = g;
            g = f;
            f = e;
            e = (d + temp1) | 0;
            d = c;
            c = b;
            b = a;
            a = (temp1 + temp2) | 0;
        }

        hash[0] = (hash[0] + a) | 0;
        hash[1] = (hash[1] + b) | 0;
        hash[2] = (hash[2] + c) | 0;
        hash[3] = (hash[3] + d) | 0;
        hash[4] = (hash[4] + e) | 0;
        hash[5] = (hash[5] + f) | 0;
        hash[6] = (hash[6] + g) | 0;
        hash[7] = (hash[7] + h) | 0;
    }

    const digest = new Uint8Array(32);
    for (let i = 0; i < hash.length; i++) {
        digest[i * 4] = (hash[i] >>> 24) & 0xff;
        digest[i * 4 + 1] = (hash[i] >>> 16) & 0xff;
        digest[i * 4 + 2] = (hash[i] >>> 8) & 0xff;
        digest[i * 4 + 3] = hash[i] & 0xff;
    }

    return bytesToHex(digest);
}

async function computeFileSha256(file) {
    const buffer = await file.arrayBuffer();

    return computeBufferSha256(buffer);
}

async function computeBufferSha256(input) {
    let buffer = input;

    if (input instanceof Uint8Array) {
        buffer = input.byteOffset === 0 && input.byteLength === input.buffer.byteLength
            ? input.buffer
            : input.buffer.slice(input.byteOffset, input.byteOffset + input.byteLength);
    }

    if (window.crypto && window.crypto.subtle) {
        const digest = await window.crypto.subtle.digest('SHA-256', buffer);
        return bytesToHex(new Uint8Array(digest));
    }

    return computeSha256Fallback(buffer);
}

async function buildFileHashHeaders(file, options = {}) {
    const headers = {};
    const algorithm = options.algorithm;
    const headerName = options.headerName;
    const required = options.required === true;
    const label = options.label || '文件';

    if (!algorithm || !headerName) {
        return headers;
    }

    if (algorithm !== 'sha256') {
        if (required) {
            throw new Error('设备使用了当前页面不支持的文件校验算法');
        }
        return headers;
    }

    showToast(`正在校验${label}哈希...`, 'info');
    headers[headerName] = await computeFileSha256(file);
    return headers;
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
    return categorizeError(err);
}

// 错误归类（仅生成中文文案，无类层级）：网络断开 / 请求超时 / 服务器错误
function categorizeError(err) {
    const m = String((err && err.message) || err || '').toLowerCase();
    return /超时|timeout/.test(m) ? '请求超时，请稍后重试'
        : /未连接|断开|network|unreachable|failed to fetch/.test(m) ? '网络已断开，请检查连接'
        : '服务器错误，请稍后重试';
}

function setButtonLoading(btn, loading) {
    if (!btn) return;
    btn.classList.toggle('btn-loading', !!loading);
    btn.disabled = !!loading;
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
let wsReconnectEnabled = true;
let wsActiveConnectionId = 0;
let wsInitialized = false; // 重连按钮可见性判定
// 指数退避：1s→30s 倍率 2 + 最多 1s 抖动；无 attempt 上限
const WS_RECONNECT_MIN_DELAY = 1000;
const WS_RECONNECT_MAX_DELAY = 30000;
const WS_RECONNECT_FACTOR = 2;
const LOG_REPLAY_STORAGE_KEY = 'logLastId';
const LOG_SESSION_STORAGE_KEY = 'logSessionId';
const LOG_MAX_ENTRIES = 500;
const LOG_DEDUPE_WINDOW = 1000;
const LOG_LEVEL_LABELS = ['ERROR', 'WARN', 'INFO', 'DEBUG', 'VERBOSE'];

// 当前过滤级别：0=全部, 1=ERROR, 2=WARN+, 3=INFO+, 4=DEBUG+, 5=VERBOSE+
let currentFilterLevel = 3;

const WS_STATE = {
    DISCONNECTED: 'disconnected',
    CONNECTING: 'connecting',
    CONNECTED: 'connected',
    RECONNECTING: 'reconnecting'
};

let wsConnectionState = {
    state: WS_STATE.DISCONNECTED,
    connected: false,
    attempt: 0,
    reason: ''
};

// 存储所有日志条目（用于过滤）
let allLogs = [];

// 存储未读日志
let unreadLogs = [];

let recentLogIds = [];
let seenLogIds = new Set();
let latestLogId = 0;
let activeLogSessionId = null;
let latestDeviceLogTimestamp = null;
let latestBrowserLogTimestamp = null;
let timestampAnchorSessionId = null;
let lastRenderedSessionId = null;

const LOG_HISTORY_STORAGE_KEY = 'logHistorySnapshot';

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
            reject(new Error(categorizeError({ message: 'WebSocket 未连接' })));
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
                reject(new Error(categorizeError({ message: '请求超时' })));
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
            reject(new Error(categorizeError(e)));
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

function rejectPendingWsRequests(message) {
    const error = new Error(message);

    Object.keys(wsRequestCallbacks).forEach((requestId) => {
        const callback = wsRequestCallbacks[requestId];
        if (!callback) {
            return;
        }

        clearTimeout(callback.timeoutTimer);
        callback.reject(error);
        delete wsRequestCallbacks[requestId];
    });
}

// ==================== 日志浮窗系统 ====================

// 浮窗状态
let logWindowVisible = false;
let logWindowMinimized = false;
let isDragging = false;
let dragOffset = { x: 0, y: 0 };

function ensureLogWindowMarkup() {
    if (document.getElementById('log-toggle-btn') && document.getElementById('log-float-window')) {
        return;
    }

    const toggleBtn = document.createElement('div');
    toggleBtn.innerHTML = `
        <button type="button" id="log-toggle-btn" class="log-toggle-btn" onclick="toggleLogWindow()" aria-label="显示/隐藏日志" aria-controls="log-float-window" aria-expanded="false">
            ${ICONS.logToggle}
            <span class="log-toggle-badge" id="log-toggle-badge" style="display: none;"></span>
        </button>
    `;
    document.body.appendChild(toggleBtn.firstElementChild);

    const logWindow = document.createElement('div');
    logWindow.innerHTML = `
        <div id="log-float-window" class="log-float-window" style="display: none;" role="dialog" aria-labelledby="log-float-title">
            <div class="log-float-header" id="log-float-header">
                <div class="log-float-title">
                    ${ICONS.logTitle}
                    <span id="log-float-title">系统日志</span>
                </div>
                <div class="log-float-controls">
                    <button type="button" class="log-float-minimize-btn" onclick="minimizeLogWindow()" aria-label="最小化" title="最小化">
                        ${ICONS.minimize}
                    </button>
                    <button type="button" class="log-float-close-btn" onclick="toggleLogWindow()" aria-label="关闭" title="关闭">
                        ${ICONS.close}
                    </button>
                </div>
            </div>
            <div class="log-float-body" id="log-float-body">
                <div class="log-float-toolbar" role="toolbar" aria-label="日志工具栏">
                    <select id="log-filter" onchange="setLogFilter(this.value)" aria-label="日志级别过滤">
                        <option value="0">全部</option>
                        <option value="1">ERROR</option>
                        <option value="2">WARN+</option>
                        <option value="3">INFO+</option>
                        <option value="4">DEBUG+</option>
                        <option value="5">VERBOSE+</option>
                    </select>
                    <button type="button" onclick="clearLogs()" aria-label="清空日志">清空</button>
                </div>
                <div id="log-output" class="log-float-output" role="log" aria-live="polite" aria-atomic="false"></div>
            </div>
        </div>
    `;
    document.body.appendChild(logWindow.firstElementChild);
}

// 初始化日志浮窗
function initLogWindow() {
    ensureLogWindowMarkup();

    const filterSelect = document.getElementById('log-filter');
    if (filterSelect) {
        filterSelect.value = String(currentFilterLevel);
    }

    restorePersistedLogHistory();

    ensureLogWindowEnhancements();

    // 从 localStorage 读取偏好
    const savedState = localStorage.getItem('logWindowState');
    if (savedState) {
        try {
            const state = JSON.parse(savedState);
            if (state.visible) {
                showLogWindow({ preserveUnread: !!state.minimized });
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

    updateLogBadge();

    // 初始化拖拽
    initLogWindowDrag();

    // 初始化键盘快捷键
    initLogKeyboardShortcuts();

    renderConnectionStatus();
    updateLogMeta();
    normalizeLogWindowPosition();
}

function ensureLogWindowEnhancements() {
    const title = document.querySelector('.log-float-title');
    if (title && !document.getElementById('log-header-unread')) {
        const unread = document.createElement('span');
        unread.id = 'log-header-unread';
        unread.className = 'log-header-unread';
        unread.style.display = 'none';
        title.appendChild(unread);
    }

    const toolbar = document.querySelector('.log-float-toolbar');
    if (toolbar && !document.getElementById('log-meta')) {
        const meta = document.createElement('div');
        meta.id = 'log-meta';
        meta.className = 'log-meta';
        meta.innerHTML = [
            '<span class="log-meta-chip" id="log-meta-state"></span>',
            '<span class="log-meta-chip" id="log-meta-total"></span>',
            '<span class="log-meta-chip" id="log-meta-unread"></span>',
            '<button type="button" id="ws-retry-btn" class="btn btn-secondary btn-sm ws-retry-btn" onclick="retryWebSocketConnection()" style="display:none;" aria-label="手动重连 WebSocket">重连</button>'
        ].join('');
        toolbar.insertBefore(meta, toolbar.firstChild);
    }

    const toggleBtn = document.getElementById('log-toggle-btn');
    if (toggleBtn && !toggleBtn.querySelector('.log-toggle-status-ring')) {
        const ring = document.createElement('span');
        ring.className = 'log-toggle-status-ring';
        ring.setAttribute('aria-hidden', 'true');
        toggleBtn.appendChild(ring);
    }
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
            toggleBtn.setAttribute('aria-expanded', 'false');
        }, 200);
        logWindowVisible = false;
    } else {
        // 显示浮窗
        logWindow.style.display = 'flex';
        logWindowVisible = true;
        toggleBtn.style.display = 'none';
        toggleBtn.setAttribute('aria-expanded', 'true');
        
        // 清空未读日志
        unreadLogs = [];
        persistLogHistory();
        updateLogBadge();
        renderLogs();
    }

    // 保存状态
    saveLogWindowState();
}

// 显示日志浮窗
function showLogWindow(options = {}) {
    const logWindow = document.getElementById('log-float-window');
    const toggleBtn = document.getElementById('log-toggle-btn');
    const preserveUnread = !!options.preserveUnread;
    
    if (!logWindow || logWindowVisible) return;

    logWindow.style.display = 'flex';
    logWindowVisible = true;
    toggleBtn.style.display = 'none';
    toggleBtn.setAttribute('aria-expanded', 'true');
    
    if (!preserveUnread) {
        unreadLogs = [];
        persistLogHistory();
        updateLogBadge();
    }

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
            minimizeBtn.innerHTML = ICONS.restore;
        }
    } else {
        logWindow.classList.remove('minimized');
        unreadLogs = [];
        persistLogHistory();
        updateLogBadge();
        if (minimizeBtn) {
            minimizeBtn.innerHTML = ICONS.minimize;
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

    updateLogMeta();
}

function getConnectionStatusLabel() {
    switch (wsConnectionState.state) {
        case WS_STATE.CONNECTED:
            return '已连接';
        case WS_STATE.RECONNECTING:
            return `重连中 #${wsConnectionState.attempt}`;
        case WS_STATE.DISCONNECTED:
            return '未连接';
        case WS_STATE.CONNECTING:
        default:
            return '连接中';
    }
}

function getVisibleLogCount() {
    return allLogs.filter(log => shouldShowLog(log.level, currentFilterLevel)).length;
}

function updateLogMeta() {
    const stateElement = document.getElementById('log-meta-state');
    const totalElement = document.getElementById('log-meta-total');
    const unreadElement = document.getElementById('log-meta-unread');
    const headerUnreadElement = document.getElementById('log-header-unread');
    const retryBtn = document.getElementById('ws-retry-btn');
    const unreadCount = getUnreadLogCount();

    if (stateElement) {
        stateElement.textContent = getConnectionStatusLabel();
        stateElement.className = `log-meta-chip connection-${wsConnectionState.state}`;
    }
    if (retryBtn) {
        const s = wsConnectionState.state;
        const showRetry = wsInitialized && (s === WS_STATE.DISCONNECTED || s === WS_STATE.RECONNECTING);
        retryBtn.style.display = showRetry ? '' : 'none';
    }

    if (totalElement) {
        totalElement.textContent = `${getVisibleLogCount()}/${allLogs.length} 条日志`;
    }

    if (unreadElement) {
        unreadElement.textContent = `未读 ${unreadCount}`;
    }

    if (headerUnreadElement) {
        if (unreadCount > 0) {
            headerUnreadElement.textContent = `未读 ${unreadCount}`;
            headerUnreadElement.style.display = 'inline-flex';
        } else {
            headerUnreadElement.style.display = 'none';
        }
    }
}

function normalizeLogWindowPosition() {
    const logWindow = document.getElementById('log-float-window');
    if (!logWindow) return;

    if (window.innerWidth <= 768) {
        logWindow.style.left = '';
        logWindow.style.top = '';
        logWindow.style.right = '';
        logWindow.style.bottom = '';
        return;
    }

    const rect = logWindow.getBoundingClientRect();
    const maxX = Math.max(0, window.innerWidth - logWindow.offsetWidth);
    const maxY = Math.max(0, window.innerHeight - logWindow.offsetHeight);
    const nextLeft = Math.max(0, Math.min(rect.left, maxX));
    const nextTop = Math.max(0, Math.min(rect.top, maxY));

    logWindow.style.right = 'auto';
    logWindow.style.bottom = 'auto';
    logWindow.style.left = `${nextLeft}px`;
    logWindow.style.top = `${nextTop}px`;
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

window.addEventListener('resize', normalizeLogWindowPosition);

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
function clearWsReconnectTimer() {
    if (wsReconnectTimer) {
        clearTimeout(wsReconnectTimer);
        wsReconnectTimer = null;
    }
}

function cleanupWsSocket(socket, shouldClose = false) {
    if (!socket) {
        return;
    }

    socket.onopen = null;
    socket.onmessage = null;
    socket.onerror = null;
    socket.onclose = null;

    if (shouldClose && socket.readyState !== WebSocket.CLOSED && socket.readyState !== WebSocket.CLOSING) {
        try {
            socket.close();
        } catch (error) {
            console.warn('关闭旧 WebSocket 失败:', error);
        }
    }
}

function scheduleWsReconnect(reason = '') {
    if (!wsReconnectEnabled || wsReconnectTimer) {
        return;
    }

    wsReconnectAttempts += 1;
    const base = Math.min(WS_RECONNECT_MAX_DELAY,
        WS_RECONNECT_MIN_DELAY * Math.pow(WS_RECONNECT_FACTOR, wsReconnectAttempts - 1));
    const delay = Math.round(base + Math.random() * WS_RECONNECT_MIN_DELAY);
    console.log(`尝试重连 #${wsReconnectAttempts}，${delay}ms 后重试...`);
    updateConnectionStatus(false, {
        state: WS_STATE.RECONNECTING,
        attempt: wsReconnectAttempts,
        reason
    });

    wsReconnectTimer = setTimeout(() => {
        wsReconnectTimer = null;
        initWebSocket();
    }, delay);
}

function initWebSocket() {
    // 获取当前页面的协议（ws 或 wss）
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    const previousSocket = ws;
    const connectionId = ++wsActiveConnectionId;
    wsInitialized = true;

    clearWsReconnectTimer();
    wsReconnectEnabled = true;
    cleanupWsSocket(previousSocket, true);

    updateConnectionStatus(false, {
        state: wsReconnectAttempts > 0 ? WS_STATE.RECONNECTING : WS_STATE.CONNECTING,
        attempt: wsReconnectAttempts,
        reason: ''
    });
    
    try {
        const socket = new WebSocket(wsUrl);
        ws = socket;
        
        socket.onopen = function() {
            if (socket !== ws || connectionId !== wsActiveConnectionId) {
                cleanupWsSocket(socket, true);
                return;
            }

            console.log('WebSocket 连接成功');
            wsReconnectAttempts = 0;
            updateConnectionStatus(true, {
                state: WS_STATE.CONNECTED,
                attempt: 0,
                reason: ''
            });
            requestLogReplay();
            // 触发自定义事件，通知其他模块 WebSocket 已连接
            document.dispatchEvent(new CustomEvent('ws-connected'));
        };
        
        socket.onmessage = function(event) {
            if (socket !== ws || connectionId !== wsActiveConnectionId) {
                return;
            }

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
                    data.forEach((log) => {
                        addLogEntry(log);
                    });
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
        
        socket.onerror = function(error) {
            if (socket !== ws || connectionId !== wsActiveConnectionId) {
                return;
            }

            console.error('WebSocket 错误:', error);
            if (wsConnectionState.state === WS_STATE.CONNECTING) {
                updateConnectionStatus(false, {
                    state: WS_STATE.CONNECTING,
                    attempt: wsReconnectAttempts,
                    reason: '连接异常'
                });
            }
        };
        
        socket.onclose = function(event) {
            if (socket === ws) {
                ws = null;
            }

            if (connectionId !== wsActiveConnectionId) {
                return;
            }

            console.log('WebSocket 连接关闭:', event.code, event.reason);
            rejectPendingWsRequests(`WebSocket 已断开 (${event.code || 'unknown'})`);
            
            // 自动重连（指数退避，无限重试）
            if (wsReconnectEnabled) {
                scheduleWsReconnect(event.reason || `连接关闭 (${event.code || 'unknown'})`);
            } else {
                updateConnectionStatus(false, {
                    state: WS_STATE.DISCONNECTED,
                    attempt: wsReconnectAttempts,
                    reason: event.reason || `连接关闭 (${event.code || 'unknown'})`
                });
            }
        };
        
    } catch (e) {
        console.error('WebSocket 初始化失败:', e);
        updateConnectionStatus(false, {
            state: WS_STATE.DISCONNECTED,
            attempt: wsReconnectAttempts,
            reason: e.message || '初始化失败'
        });
    }
}

// 关闭 WebSocket 连接
function closeWebSocket() {
    wsReconnectEnabled = false;
    clearWsReconnectTimer();
    
    if (ws) {
        const currentSocket = ws;
        ws = null;
        cleanupWsSocket(currentSocket, true);
    }
    
    wsReconnectAttempts = 0;
    rejectPendingWsRequests('WebSocket 已手动关闭');
    updateConnectionStatus(false, {
        state: WS_STATE.DISCONNECTED,
        attempt: 0,
        reason: '连接已关闭'
    });
}

// 手动重连（log-meta 中"重连"按钮触发）
function retryWebSocketConnection() {
    if (ws && ws.readyState === WebSocket.OPEN) return;
    wsReconnectAttempts = 0;
    initWebSocket();
}

function getStoredLastLogId() {
    const rawValue = sessionStorage.getItem(LOG_REPLAY_STORAGE_KEY);
    if (rawValue === null) {
        return null;
    }

    const parsedValue = Number.parseInt(rawValue, 10);
    if (!Number.isFinite(parsedValue) || parsedValue <= 0) {
        sessionStorage.removeItem(LOG_REPLAY_STORAGE_KEY);
        return null;
    }

    latestLogId = Math.max(latestLogId, parsedValue);
    return parsedValue;
}

function getStoredLogSessionId() {
    const rawValue = sessionStorage.getItem(LOG_SESSION_STORAGE_KEY);
    if (rawValue === null) {
        return null;
    }

    const parsedValue = Number.parseInt(rawValue, 10);
    if (!Number.isFinite(parsedValue) || parsedValue <= 0) {
        sessionStorage.removeItem(LOG_SESSION_STORAGE_KEY);
        return null;
    }

    return parsedValue;
}

function storeLastLogId(logId) {
    if (!Number.isFinite(logId) || logId <= 0) {
        return;
    }

    latestLogId = logId;
    sessionStorage.setItem(LOG_REPLAY_STORAGE_KEY, String(latestLogId));
}

function storeLogSessionId(sessionId) {
    if (!Number.isFinite(sessionId) || sessionId <= 0) {
        activeLogSessionId = null;
        sessionStorage.removeItem(LOG_SESSION_STORAGE_KEY);
        return;
    }

    activeLogSessionId = sessionId;
    sessionStorage.setItem(LOG_SESSION_STORAGE_KEY, String(sessionId));
}

function resetReplayCursor() {
    latestLogId = 0;
    recentLogIds = [];
    seenLogIds = new Set();
    sessionStorage.removeItem(LOG_REPLAY_STORAGE_KEY);
}

function resetLogTimestampAnchor(sessionId = null) {
    timestampAnchorSessionId = sessionId;
    latestDeviceLogTimestamp = null;
    latestBrowserLogTimestamp = null;
}

function normalizeLogSessionId(sessionId) {
    if (sessionId === undefined || sessionId === null) {
        return null;
    }

    const parsedValue = Number(sessionId);
    if (!Number.isFinite(parsedValue) || parsedValue <= 0) {
        return null;
    }

    return parsedValue;
}

function getLogIdentity(logData) {
    const logId = normalizeLogId(logData);
    if (logId === null) {
        return null;
    }

    const logSessionId = normalizeLogSessionId(logData?.sessionId);
    return logSessionId !== null ? `${logSessionId}:${logId}` : String(logId);
}

function normalizeLogSession(logData, fallbackSessionId = activeLogSessionId) {
    if (!logData || typeof logData !== 'object') {
        return null;
    }

    const normalizedSessionId = normalizeLogSessionId(logData.sessionId);
    if (normalizedSessionId !== null) {
        logData.sessionId = normalizedSessionId;
        return normalizedSessionId;
    }

    if (fallbackSessionId !== null) {
        logData.sessionId = fallbackSessionId;
        return fallbackSessionId;
    }

    delete logData.sessionId;
    return null;
}

function persistLogHistory() {
    const snapshot = {
        sessionId: activeLogSessionId,
        logs: allLogs.slice(-LOG_MAX_ENTRIES),
        unreadLogs: unreadLogs.slice(-LOG_MAX_ENTRIES)
    };

    try {
        sessionStorage.setItem(LOG_HISTORY_STORAGE_KEY, JSON.stringify(snapshot));
    } catch (error) {
        console.warn('保存日志历史失败:', error);
    }
}

function restorePersistedLogHistory() {
    const rawValue = sessionStorage.getItem(LOG_HISTORY_STORAGE_KEY);
    if (!rawValue) {
        return;
    }

    try {
        const snapshot = JSON.parse(rawValue);
        const restoredLogs = Array.isArray(snapshot?.logs) ? snapshot.logs.slice(-LOG_MAX_ENTRIES) : [];
        const restoredUnreadLogs = Array.isArray(snapshot?.unreadLogs) ? snapshot.unreadLogs.slice(-LOG_MAX_ENTRIES) : [];
        const restoredSessionId = normalizeLogSessionId(snapshot?.sessionId) ?? getStoredLogSessionId();

        allLogs = restoredLogs;
        unreadLogs = restoredUnreadLogs;
        storeLogSessionId(restoredSessionId);
        resetLogTimestampAnchor();

        allLogs.forEach((logData) => {
            normalizeLogSession(logData, restoredSessionId);
            logData.browserTimestamp = resolveLogBrowserTimestamp(logData);
        });

        unreadLogs.forEach((logData) => {
            normalizeLogSession(logData, restoredSessionId);
            logData.browserTimestamp = resolveLogBrowserTimestamp(logData);
        });

        resetReplayCursor();
        allLogs.forEach((logData) => {
            rememberLogId(logData);
        });
    } catch (error) {
        console.warn('恢复日志历史失败:', error);
        sessionStorage.removeItem(LOG_HISTORY_STORAGE_KEY);
        allLogs = [];
        unreadLogs = [];
        storeLogSessionId(null);
        resetReplayCursor();
        resetLogTimestampAnchor();
    }
}

function normalizeLogId(logData) {
    if (!logData || logData.id === undefined || logData.id === null) {
        return null;
    }

    const parsedValue = Number.parseInt(logData.id, 10);
    if (!Number.isFinite(parsedValue) || parsedValue <= 0) {
        return null;
    }

    return parsedValue;
}

function rememberLogId(logData) {
    const logId = normalizeLogId(logData);
    if (logId === null) {
        return;
    }

    const logIdentity = getLogIdentity(logData);
    if (logIdentity === null || seenLogIds.has(logIdentity)) {
        return;
    }

    seenLogIds.add(logIdentity);
    recentLogIds.push(logIdentity);

    if (recentLogIds.length > LOG_DEDUPE_WINDOW) {
        const removedLogIdentity = recentLogIds.shift();
        seenLogIds.delete(removedLogIdentity);
    }

    if (normalizeLogSessionId(logData.sessionId) === activeLogSessionId) {
        storeLastLogId(logId);
    }
}

async function requestLogReplay() {
    const lastLogId = getStoredLastLogId();
    const storedSessionId = getStoredLogSessionId();
    const previousSessionId = activeLogSessionId ?? storedSessionId;
    const replayRequest = {};

    if (lastLogId !== null && previousSessionId !== null) {
        replayRequest.lastLogId = lastLogId;
        replayRequest.sessionId = previousSessionId;
    }

    try {
        const data = await sendWsRequest('log/replay', replayRequest, 10000);
        const replayLogs = data && Array.isArray(data.logs) ? data.logs : [];
        const replaySessionId = normalizeLogSessionId(data?.sessionId);
        const sessionChanged = replaySessionId !== null && previousSessionId !== null && replaySessionId !== previousSessionId;
        const hasCachedLogs = allLogs.length > 0 || unreadLogs.length > 0;

        if (sessionChanged) {
            const shouldClearLogs = !hasCachedLogs || await showConfirm(
                '检测到设备日志已重新开始，是否清空上次缓存的日志记录？<br><br>清空后将只保留本次启动后的日志。',
                {
                    type: 'warning',
                    title: '检测到设备已重启',
                    confirmText: '清空旧日志',
                    cancelText: '保留旧日志',
                    backdrop: true
                }
            );

            if (shouldClearLogs) {
                clearLogs({ preserveReplayCursor: false, preserveSession: false });
            } else {
                resetReplayCursor();
                resetLogTimestampAnchor();
                storeLogSessionId(replaySessionId);
            }
        } else if (replaySessionId !== null && activeLogSessionId === null) {
            storeLogSessionId(replaySessionId);
        }

        if (data && data.mode === 'fallback' && typeof data.requestedLastLogId === 'number' && typeof data.latestAvailableLogId === 'number' && data.requestedLastLogId > data.latestAvailableLogId) {
            resetReplayCursor();
        }

        replayLogs.forEach((log) => {
            if (replaySessionId !== null && normalizeLogSessionId(log.sessionId) === null) {
                log.sessionId = replaySessionId;
            }
            addLogEntry(log);
        });

        if (replaySessionId !== null) {
            storeLogSessionId(replaySessionId);
        }

        if (data && data.mode === 'fallback' && replayLogs.length > 0) {
            console.info('日志回放降级为最近窗口:', {
                requestedLastLogId: data.requestedLastLogId,
                oldestAvailableLogId: data.oldestAvailableLogId,
                latestAvailableLogId: data.latestAvailableLogId,
                sessionId: replaySessionId
            });
        }
    } catch (error) {
        console.error('请求日志回放失败:', error);
    }
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
    const fallbackSessionId = activeLogSessionId ?? getStoredLogSessionId();
    const logSessionId = normalizeLogSession(logData, fallbackSessionId);
    const logIdentity = getLogIdentity(logData);

    if (logIdentity !== null && seenLogIds.has(logIdentity)) {
        return;
    }

    if (logSessionId !== null && logSessionId !== activeLogSessionId) {
        storeLogSessionId(logSessionId);
        resetLogTimestampAnchor(logSessionId);
    }

    rememberLogId(logData);

    logData.browserTimestamp = resolveLogBrowserTimestamp(logData);

    // 存储到数组
    allLogs.push(logData);
    
    // 限制日志数量（最多保留 500 条）
    if (allLogs.length > LOG_MAX_ENTRIES) {
        allLogs.shift();
    }
    
    // 存储未读日志
    if (!logWindowVisible || logWindowMinimized) {
        unreadLogs.push(logData);
        if (unreadLogs.length > LOG_MAX_ENTRIES) {
            unreadLogs.shift();
        }
    }

    persistLogHistory();
    
    // 如果浮窗隐藏，更新徽章
    if (!logWindowVisible || logWindowMinimized) {
        updateLogBadge();
    }
    
    // 根据过滤级别决定是否显示
    if (shouldShowLog(logData.level, currentFilterLevel) && logWindowVisible && !logWindowMinimized) {
        renderLogEntry(logData);
    }

    updateLogMeta();
}

// 渲染单条日志
function renderLogEntry(logData) {
    const logOutput = document.getElementById('log-output');
    if (!logOutput) return;
    const shouldStickToBottom = isNearLogBottom(logOutput);
    const currentSessionId = normalizeLogSessionId(logData.sessionId);

    if (shouldRenderSessionDivider(lastRenderedSessionId, currentSessionId)) {
        const divider = createSessionDivider();
        logOutput.appendChild(divider);
    }

    // 创建日志元素
    const entry = document.createElement('div');
    entry.className = `log-entry ${getLogClass(logData.level)}`;

    entry.innerHTML = buildLogEntryMarkup(logData);

    logOutput.appendChild(entry);

    if (currentSessionId !== null) {
        lastRenderedSessionId = currentSessionId;
    }

    // 自动滚动到底部
    if (shouldStickToBottom) {
        scrollToBottom();
    }
}

function createSessionDivider() {
    const divider = document.createElement('div');
    divider.className = 'log-session-divider';
    divider.innerHTML = `
        <div class="divider-label">当前启动日志</div>
    `;
    return divider;
}

function shouldRenderSessionDivider(previousSessionId, currentSessionId) {
    return currentSessionId !== null && previousSessionId !== null && currentSessionId !== previousSessionId && activeLogSessionId !== null && currentSessionId === activeLogSessionId;
}

// 重新渲染所有日志（用于过滤级别变化）
function renderLogs() {
    const logOutput = document.getElementById('log-output');
    if (!logOutput) return;

    // 清空当前显示
    logOutput.innerHTML = '';
    lastRenderedSessionId = null;

    // 根据当前过滤级别重新渲染
    allLogs.forEach(logData => {
        if (shouldShowLog(logData.level, currentFilterLevel)) {
            const currentSessionId = normalizeLogSessionId(logData.sessionId);

            if (shouldRenderSessionDivider(lastRenderedSessionId, currentSessionId)) {
                const divider = createSessionDivider();
                logOutput.appendChild(divider);
            }

            const entry = document.createElement('div');
            entry.className = `log-entry ${getLogClass(logData.level)}`;

            entry.innerHTML = buildLogEntryMarkup(logData);

            logOutput.appendChild(entry);

            if (currentSessionId !== null) {
                lastRenderedSessionId = currentSessionId;
            }
        }
    });

    // 滚动到底部
    scrollToBottom();

    updateLogMeta();
}

function buildLogEntryMarkup(logData) {
    const time = formatTimestamp(logData.browserTimestamp);
    const levelLabel = getLogLevelLabel(logData.level);
    const normalizedTag = normalizeLogTag(logData.tag, levelLabel);
    const message = escapeHtml(logData.message || '');
    const tagMarkup = normalizedTag ? `<span class="log-tag">${escapeHtml(normalizedTag)}</span>` : '';

    return `
        <div class="log-entry-meta">
            <span class="log-level-badge">${levelLabel}</span>
            <span class="log-time">${time}</span>
            ${tagMarkup}
        </div>
        <div class="log-message">${message}</div>
    `;
}

function normalizeLogTag(tag, levelLabel) {
    const normalizedTag = String(tag || '').trim();

    if (!normalizedTag) {
        return 'SYSTEM';
    }

    if (normalizedTag.toUpperCase() === levelLabel) {
        return '';
    }

    return normalizedTag;
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

function normalizeLogTimestamp(timestamp) {
    if (timestamp === undefined || timestamp === null) {
        return null;
    }

    const parsedTimestamp = Number(timestamp);
    if (!Number.isFinite(parsedTimestamp) || parsedTimestamp < 0) {
        return null;
    }

    return parsedTimestamp;
}

function resolveLogBrowserTimestamp(logData) {
    const storedBrowserTimestamp = normalizeLogTimestamp(logData.browserTimestamp);
    if (storedBrowserTimestamp !== null) {
        return storedBrowserTimestamp;
    }

    const logSessionId = normalizeLogSession(logData, activeLogSessionId);
    const deviceTimestamp = normalizeLogTimestamp(logData.timestamp);
    const browserNow = Date.now();

    if (logSessionId !== null && timestampAnchorSessionId !== logSessionId) {
        resetLogTimestampAnchor(logSessionId);
    }

    if (deviceTimestamp === null) {
        return browserNow;
    }

    if (latestDeviceLogTimestamp === null || deviceTimestamp >= latestDeviceLogTimestamp) {
        latestDeviceLogTimestamp = deviceTimestamp;
        latestBrowserLogTimestamp = browserNow;
    }

    if (latestBrowserLogTimestamp === null || latestDeviceLogTimestamp === null) {
        return browserNow;
    }

    return latestBrowserLogTimestamp - (latestDeviceLogTimestamp - deviceTimestamp);
}

function getLogLevelLabel(level) {
    return LOG_LEVEL_LABELS[level] || 'INFO';
}

function isNearLogBottom(logOutput) {
    const threshold = 24;
    return logOutput.scrollHeight - logOutput.scrollTop - logOutput.clientHeight <= threshold;
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
function clearLogs(options = {}) {
    const preserveReplayCursor = options.preserveReplayCursor !== false;
    const preserveSession = options.preserveSession !== false;
    const logOutput = document.getElementById('log-output');
    if (logOutput) {
        logOutput.innerHTML = '';
    }
    
    // 清空存储的日志
    allLogs = [];
    unreadLogs = [];
    recentLogIds = [];
    seenLogIds = new Set();
    lastRenderedSessionId = null;
    sessionStorage.removeItem(LOG_HISTORY_STORAGE_KEY);

    if (!preserveSession) {
        storeLogSessionId(null);
    }

    resetLogTimestampAnchor(preserveSession ? activeLogSessionId : null);

    if (preserveReplayCursor && latestLogId > 0) {
        storeLastLogId(latestLogId);
    } else {
        latestLogId = 0;
        sessionStorage.removeItem(LOG_REPLAY_STORAGE_KEY);
    }
    
    console.log('日志已清空');

    updateLogBadge();
}

function updateConnectionStatus(connected, options = {}) {
    wsConnectionState = {
        state: options.state || (connected ? WS_STATE.CONNECTED : WS_STATE.DISCONNECTED),
        connected,
        attempt: options.attempt || 0,
        reason: options.reason || ''
    };

    document.dispatchEvent(new CustomEvent('ws-state-changed', {
        detail: { ...wsConnectionState }
    }));

    renderConnectionStatus();
    updateLogMeta();
}

function renderConnectionStatus() {
    ensureLogWindowEnhancements();

    const toggleBtn = document.getElementById('log-toggle-btn');
    const statusLabel = getConnectionStatusLabel();

    if (toggleBtn) {
        toggleBtn.dataset.wsState = wsConnectionState.state;
        toggleBtn.setAttribute('aria-label', `显示/隐藏日志，${statusLabel}`);
        if (wsConnectionState.reason) {
            toggleBtn.title = `${statusLabel} - ${wsConnectionState.reason}`;
        } else {
            toggleBtn.title = statusLabel;
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
