#ifndef HTML_INDEX_H
#define HTML_INDEX_H

#include <pgmspace.h>

// 主页HTML（优化版 - UI/UX Pro Max设计系统）
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32-C3 NFC门禁系统</title>
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
            --shadow-glow: 0 0 20px rgba(8, 145, 178, 0.15);
            
            /* 圆角 */
            --radius-sm: 6px;
            --radius-md: 10px;
            --radius-lg: 16px;
            --radius-xl: 20px;
            
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

        /* 主容器 */
        .container {
            max-width: 640px;
            margin: 0 auto;
            background: var(--color-bg-card);
            backdrop-filter: blur(20px);
            -webkit-backdrop-filter: blur(20px);
            padding: var(--spacing-2xl) var(--spacing-xl);
            border-radius: var(--radius-xl);
            box-shadow: var(--shadow-lg);
            border: 1px solid var(--color-border);
        }

        /* 标题 */
        h1 {
            color: var(--color-primary);
            font-size: 32px;
            font-weight: 700;
            margin: 0 0 var(--spacing-lg) 0;
            letter-spacing: -0.02em;
            line-height: 1.2;
            text-shadow: 0 2px 8px rgba(8, 145, 178, 0.1);
        }

        /* 系统信息卡片 */
        .system-info {
            margin: var(--spacing-lg) 0;
            padding: var(--spacing-lg);
            background: linear-gradient(135deg, var(--color-bg-elevated) 0%, white 100%);
            border-radius: var(--radius-lg);
            border: 1px solid var(--color-border);
            font-size: 14px;
            color: var(--color-text-muted);
            text-align: left;
            box-shadow: var(--shadow-sm);
            transition: var(--transition-base);
        }

        .system-info:hover {
            box-shadow: var(--shadow-md);
            transform: translateY(-2px);
        }

        .system-info .info-row {
            display: flex;
            align-items: center;
            justify-content: space-between;
            padding: var(--spacing-sm) 0;
        }

        .system-info .info-row:not(:last-child) {
            border-bottom: 1px solid var(--color-border);
        }

        .system-info strong {
            color: var(--color-text);
            font-weight: 600;
        }

        /* 按钮组 */
        .btn-group {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: var(--spacing-md);
            margin: var(--spacing-xl) 0;
        }

        /* 按钮基础样式 */
        .btn {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            width: 100%;
            max-width: 320px;
            padding: 16px 32px;
            text-decoration: none;
            color: white;
            background: linear-gradient(135deg, var(--color-primary) 0%, var(--color-primary-hover) 100%);
            border-radius: var(--radius-lg);
            font-size: 15px;
            font-weight: 600;
            transition: var(--transition-smooth);
            cursor: pointer;
            border: none;
            box-shadow: var(--shadow-md);
            min-height: 52px;
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
            transform: translateY(-3px);
            box-shadow: var(--shadow-lg), var(--shadow-glow);
        }

        .btn:active {
            transform: translateY(-1px);
            box-shadow: var(--shadow-md);
        }

        .btn:focus-visible {
            outline: 3px solid var(--color-primary-light);
            outline-offset: 3px;
        }

        .btn svg {
            width: 20px;
            height: 20px;
            margin-right: var(--spacing-sm);
            transition: var(--transition-base);
        }

        .btn:hover svg {
            transform: scale(1.1);
        }

        /* 成功按钮 */
        .btn-success {
            background: linear-gradient(135deg, var(--color-success) 0%, var(--color-success-hover) 100%);
        }

        .btn-success:hover {
            box-shadow: var(--shadow-lg), 0 0 20px rgba(34, 197, 94, 0.3);
        }

        /* 危险按钮 */
        .btn-danger {
            background: linear-gradient(135deg, var(--color-danger) 0%, var(--color-danger-hover) 100%);
        }

        .btn-danger:hover {
            box-shadow: var(--shadow-lg), 0 0 20px rgba(239, 68, 68, 0.3);
        }

        /* 响应式设计 */
        @media (max-width: 768px) {
            body {
                padding: var(--spacing-lg) var(--spacing-md);
            }

            .container {
                padding: var(--spacing-xl) var(--spacing-lg);
                border-radius: var(--radius-lg);
            }

            h1 {
                font-size: 26px;
            }

            .btn {
                max-width: 100%;
                padding: 14px 24px;
                font-size: 14px;
            }
        }

        @media (max-width: 480px) {
            body {
                padding: var(--spacing-md);
            }

            .container {
                padding: var(--spacing-lg) var(--spacing-md);
            }

            h1 {
                font-size: 22px;
            }

            .system-info {
                font-size: 13px;
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
        <h1>NFC门禁</h1>
        
        <div class="system-info">
            <div class="info-row">
                <span><strong>IP：</strong></span>
                <span id="ip">-</span>
            </div>
            <div class="info-row">
                <span><strong>WiFi：</strong></span>
                <span id="wifi">-</span>
            </div>
        </div>

        <div class="btn-group">
            <a href="/files" class="btn btn-success" aria-label="文件管理">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                    <path d="M22 19a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h5l2 3h9a2 2 0 0 1 2 2z"></path>
                </svg>
                文件管理
            </a>
            <a href="/servo" class="btn" aria-label="舵机管理">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                    <circle cx="12" cy="12" r="3"></circle>
                    <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z"></path>
                </svg>
                舵机管理
            </a>
            <a href="/ota" class="btn" aria-label="OTA升级">
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
                OTA升级
            </a>
            <button onclick="restartSystem()" class="btn btn-danger" aria-label="重启系统">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                    <path d="M23 4v6h-6"></path>
                    <path d="M20.49 15a9 9 0 1 1-2.12-9.36L23 10"></path>
                </svg>
                重启系统
            </button>
        </div>
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
</body>
</html>
)rawliteral";

#endif // HTML_INDEX_H