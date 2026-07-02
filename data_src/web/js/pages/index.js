// 首页逻辑模块
(function() {
    // 状态磁贴内联图标（与 index.html 中的占位图标一致）
    const WIFI_ICON = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">'
        + '<path d="M5 12.55a11 11 0 0 1 14.08 0"></path>'
        + '<path d="M1.42 9a16 16 0 0 1 21.16 0"></path>'
        + '<path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>'
        + '<line x1="12" y1="20" x2="12.01" y2="20"></line>'
        + '</svg>';
    const BATTERY_ICON = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">'
        + '<rect x="2" y="7" width="16" height="10" rx="2"></rect>'
        + '<line x1="22" y1="11" x2="22" y2="13"></line>'
        + '</svg>';

    // 组装一块状态磁贴的 innerHTML
    function tileMarkup(icon, label, value, extra) {
        return '<div class="tile-icon">' + icon + '</div>'
            + '<div class="tile-body">'
            + '<div class="tile-label">' + label + '</div>'
            + '<div class="tile-value">' + value + '</div>'
            + (extra || '')
            + '</div>';
    }

    // 电压 → 电量百分比：3.4V 为临界下限(0%)，4.2V 视为满电(100%)
    function voltageToLevel(voltage) {
        const level = (voltage - 3.4) / (4.2 - 3.4) * 100;
        return Math.max(0, Math.min(100, Math.round(level)));
    }

    // 渲染 WiFi 磁贴（data: {mode, ssid, ip}）
    function renderWifi(data) {
        const el = document.getElementById('wifi');
        let stateClass, label, value, sub;
        if (data && data.mode === 'AP') {
            stateClass = 'state-warning';
            label = 'WiFi · AP模式';
            value = data.ssid || 'NFCKey-AP';
            sub = data.ip || '';
        } else if (data && data.ssid) {
            stateClass = 'state-success';
            label = 'WiFi · 已连接';
            value = data.ssid;
            sub = data.ip || '';
        } else {
            stateClass = 'state-muted';
            label = 'WiFi';
            value = '未连接';
            sub = '';
        }
        el.className = 'status-tile ' + stateClass;
        el.innerHTML = tileMarkup(WIFI_ICON, label, value, sub ? '<div class="tile-sub">' + sub + '</div>' : '');
    }

    function renderWifiError() {
        const el = document.getElementById('wifi');
        el.className = 'status-tile state-muted';
        el.innerHTML = tileMarkup(WIFI_ICON, 'WiFi', '获取失败', '');
    }

    // 渲染电池磁贴（data: {voltage, status}）
    function renderBattery(data) {
        const el = document.getElementById('battery');
        if (!data || typeof data.voltage === 'undefined') { renderBatteryError(); return; }

        const voltage = data.voltage;
        const status = data.status || 'normal';
        const level = voltageToLevel(voltage);

        let stateClass = 'state-success';
        let statusText = '正常';
        if (status === 'low') { stateClass = 'state-warning'; statusText = '电量低'; }
        else if (status === 'critical') { stateClass = 'state-danger'; statusText = '电量危急'; }

        const bar = '<div class="battery-row">'
            + '<span class="battery-bar"><span class="battery-bar-fill" style="width:' + level + '%"></span></span>'
            + '<span class="battery-pct">' + level + '%</span>'
            + '</div>';

        el.className = 'status-tile ' + stateClass;
        el.innerHTML = tileMarkup(BATTERY_ICON, '电池 · ' + statusText, voltage + ' V', bar);
    }

    function renderBatteryError() {
        const el = document.getElementById('battery');
        el.className = 'status-tile state-muted';
        el.innerHTML = tileMarkup(BATTERY_ICON, '电池', '获取失败', '');
    }

    // 页面加载时获取系统信息
    function init() {
        waitForWebSocketReady()
            .then(loadSystemInfo)
            .catch((error) => {
                console.error('等待 WebSocket 连接失败:', error);
                renderWifiError();
                loadBatteryInfo();
            });
    }

    // 获取系统信息
    function loadSystemInfo() {
        loadBatteryInfo();

        // 获取WiFi信息
        sendWsRequest('wifi/getInfo')
            .then(data => renderWifi(data))
            .catch(error => {
                console.error('获取WiFi信息失败:', error);
                renderWifiError();
            });
    }

    function loadBatteryInfo() {
        fetch('/api/battery')
            .then(response => response.json())
            .then(response => {
                if (response.success && response.data) {
                    renderBattery(response.data);
                } else {
                    renderBatteryError();
                }
            })
            .catch(error => {
                console.error('获取电池电压失败:', error);
                renderBatteryError();
            });
    }

    // 重启系统
    function restartSystem() {
        const restartBtn = document.querySelector('button[onclick="restartSystem()"]');
        showConfirm('确定要重启系统吗？\n\n重启后将回到正常门禁模式，Web服务器将停止运行。', {
            type: 'danger',
            title: '确认重启'
        }).then(result => {
            if (!result) return;
            setButtonLoading(restartBtn, true);
            fetch('/api/system/actions/restart', { method: 'POST' })
                .then(response => {
                    if (response.ok) {
                        showToast('系统正在重启，请稍候...', 'info', { duration: 5000 });
                    } else {
                        showToast('重启失败，请重试', 'error');
                    }
                })
                .catch(error => {
                    showToast('请求失败：' + error, 'error');
                })
                .finally(() => {
                    setButtonLoading(restartBtn, false);
                });
        });
    }

    // WebSocket 重连后自动刷新状态（去抖：2s 内的重复事件忽略）
    let lastWsRefreshTs = 0;
    document.addEventListener('ws-connected', function() {
        const now = Date.now();
        if (now - lastWsRefreshTs < 2000) return;
        lastWsRefreshTs = now;
        loadSystemInfo();
    });

    // 暴露给全局
    window.restartSystem = restartSystem;

    // 页面加载完成后初始化
    window.addEventListener('DOMContentLoaded', init);
})();
