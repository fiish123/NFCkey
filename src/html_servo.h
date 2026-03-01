#ifndef HTML_SERVO_H
#define HTML_SERVO_H

#include <pgmspace.h>

// 舵机管理页面HTML（优化版 - UI/UX Pro Max设计系统）
const char servo_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>舵机管理</title>
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
        }

        /* 区域卡片 */
        .section {
            margin: var(--spacing-lg) 0;
            padding: var(--spacing-lg);
            background: linear-gradient(135deg, var(--color-bg-elevated) 0%, white 100%);
            border-radius: var(--radius-md);
            border: 1px solid var(--color-border);
            box-shadow: var(--shadow-sm);
        }

        .section h2 {
            color: var(--color-primary);
            font-size: 18px;
            font-weight: 600;
            margin: 0 0 var(--spacing-lg) 0;
            padding-bottom: var(--spacing-md);
            border-bottom: 2px solid var(--color-border);
            display: flex;
            align-items: center;
            gap: var(--spacing-sm);
        }

        .section h2 svg {
            width: 20px;
            height: 20px;
        }

        /* 输入组 */
        .input-group {
            margin: var(--spacing-md) 0;
            text-align: left;
        }

        .input-group label {
            display: block;
            margin-bottom: var(--spacing-sm);
            font-weight: 600;
            font-size: 14px;
            color: var(--color-text);
        }

        .input-group input {
            width: 100%;
            padding: 12px 16px;
            border: 1px solid var(--color-border);
            border-radius: var(--radius-sm);
            background: var(--color-bg-card);
            color: var(--color-text);
            font-family: inherit;
            font-size: 15px;
            transition: var(--transition-base);
        }

        .input-group input:focus {
            outline: none;
            border-color: var(--color-primary);
            box-shadow: 0 0 0 3px var(--color-primary-light);
        }

        .input-group .range-info {
            font-size: 13px;
            color: var(--color-text-muted);
            margin-top: var(--spacing-sm);
        }

        /* 按钮组 */
        .btn-group {
            display: flex;
            flex-wrap: wrap;
            gap: var(--spacing-md);
            margin: var(--spacing-lg) 0;
        }

        /* 按钮基础样式 */
        .btn {
            flex: 1;
            min-width: 140px;
            padding: 12px 24px;
            color: white;
            background: linear-gradient(135deg, var(--color-primary) 0%, var(--color-primary-hover) 100%);
            border: none;
            border-radius: var(--radius-sm);
            font-size: 15px;
            font-weight: 600;
            cursor: pointer;
            transition: var(--transition-smooth);
            box-shadow: var(--shadow-sm);
            min-height: 48px;
            position: relative;
            overflow: hidden;
        }

        .btn::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.2), transparent);
            transition: var(--transition-smooth);
        }

        .btn:hover::before {
            left: 100%;
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

        .btn:disabled {
            background: #CBD5E1;
            cursor: not-allowed;
            transform: none;
            box-shadow: none;
        }

        .btn:disabled::before {
            display: none;
        }

        .btn-success {
            background: linear-gradient(135deg, var(--color-success) 0%, var(--color-success-hover) 100%);
        }

        .btn-success:hover {
            box-shadow: var(--shadow-md), 0 0 20px rgba(34, 197, 94, 0.3);
        }

        .btn-danger {
            background: linear-gradient(135deg, var(--color-danger) 0%, var(--color-danger-hover) 100%);
        }

        .btn-danger:hover {
            box-shadow: var(--shadow-md), 0 0 20px rgba(239, 68, 68, 0.3);
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

        /* 舵机状态指示器 */
        .servo-status {
            font-size: 15px;
            font-weight: 600;
            margin: var(--spacing-md) 0;
            padding: var(--spacing-md);
            border-radius: var(--radius-sm);
            display: flex;
            align-items: center;
            justify-content: center;
            gap: var(--spacing-sm);
            transition: var(--transition-base);
        }

        .servo-status.idle {
            background: linear-gradient(135deg, #DCFCE7 0%, #BBF7D0 100%);
            color: #166534;
            border-left: 4px solid var(--color-success);
        }

        .servo-status.busy {
            background: linear-gradient(135deg, #FEE2E2 0%, #FECACA 100%);
            color: #991B1B;
            border-left: 4px solid var(--color-danger);
        }

        .servo-status .status-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: currentColor;
            animation: pulse 1.5s ease-in-out infinite;
        }

        @keyframes pulse {
            0%, 100% { opacity: 1; transform: scale(1); }
            50% { opacity: 0.5; transform: scale(0.8); }
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

            .btn-group {
                flex-direction: column;
            }

            .btn {
                width: 100%;
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

            .section {
                padding: var(--spacing-md);
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
        <h1>舵机管理</h1>
        
        <!-- 配置区域 -->
        <div class="section">
            <h2>
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                    <circle cx="12" cy="12" r="3"></circle>
                    <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z"></path>
                </svg>
                位置配置
            </h2>
            <div class="input-group">
                <label for="unlockPos">解锁位置 (0-4095)</label>
                <input type="number" id="unlockPos" min="0" max="4095" value="800" aria-label="解锁位置">
                <div class="range-info">舵机解锁时的位置值</div>
            </div>
            <div class="input-group">
                <label for="lockPos">锁定位置 (0-4095)</label>
                <input type="number" id="lockPos" min="0" max="4095" value="1180" aria-label="锁定位置">
                <div class="range-info">舵机锁定时的位置值</div>
            </div>
            <div class="btn-group">
                <button class="btn btn-success" onclick="saveConfig()" aria-label="保存配置">保存配置</button>
                <button class="btn" onclick="loadConfig()" aria-label="重新加载配置">重新加载</button>
            </div>
        </div>
        
        <!-- 控制区域 -->
        <div class="section">
            <h2>
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                    <polygon points="5 3 19 12 5 21 5 3"></polygon>
                </svg>
                动作控制
            </h2>
            <div class="servo-status idle" id="servoStatus">
                <span class="status-dot"></span>
                <span>状态: 空闲</span>
            </div>
            <div class="btn-group">
                <button class="btn btn-success" onclick="unlock()" id="unlockBtn" aria-label="解锁门锁">
                    <svg style="width: 18px; height: 18px; margin-right: 6px;" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                        <rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect>
                        <path d="M7 11V7a5 5 0 0 1 10 0v4"></path>
                    </svg>
                    解锁
                </button>
                <button class="btn btn-danger" onclick="lock()" id="lockBtn" aria-label="锁定门锁">
                    <svg style="width: 18px; height: 18px; margin-right: 6px;" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                        <rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect>
                        <path d="M7 11V7a5 5 0 0 1 9.9-1"></path>
                    </svg>
                    锁定
                </button>
            </div>
        </div>
        
        <div id="status" class="status info"></div>
        
        <a href="/" class="back-link">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                <line x1="19" y1="12" x2="5" y2="12"></line>
                <polyline points="12 19 5 12 12 5"></polyline>
            </svg>
            返回主页
        </a>
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
                statusDiv.innerHTML = '<span class="status-dot"></span><span>状态: 执行中...</span>';
                unlockBtn.disabled = true;
                lockBtn.disabled = true;
            } else {
                statusDiv.className = 'servo-status idle';
                statusDiv.innerHTML = '<span class="status-dot"></span><span>状态: 空闲</span>';
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

#endif // HTML_SERVO_H