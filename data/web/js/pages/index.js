// 首页逻辑模块
(function() {
    // 页面加载时获取系统信息
    function init() {
        loadSystemInfo();
    }

    // 获取WiFi信息
    function loadSystemInfo() {
        // 获取WiFi信息
        fetch('/api/wifi')
            .then(response => response.json())
            .then(response => {
                if (response.success && response.data) {
                    document.getElementById('wifi').innerHTML = response.data.ssid + ' (' + response.data.ip + ')';
                } else {
                    document.getElementById('wifi').innerHTML = '获取失败';
                }
            })
            .catch(error => {
                console.error('获取WiFi信息失败:', error);
                document.getElementById('wifi').innerHTML = '获取失败';
            });

        // 获取电池电压
        fetch('/api/battery')
            .then(response => response.json())
            .then(response => {
                const batteryEl = document.getElementById('battery');
                if (response.success && response.data) {
                    const voltage = response.data.voltage;
                    const status = response.data.status;

                    // 设置电压显示文本
                    batteryEl.textContent = voltage + ' V';

                    // 根据状态设置颜色
                    if (status === 'normal') {
                        batteryEl.style.color = '#10b981'; // 绿色
                    } else if (status === 'low') {
                        batteryEl.style.color = '#f59e0b'; // 黄色
                    } else if (status === 'critical') {
                        batteryEl.style.color = '#ef4444'; // 红色
                    }
                } else {
                    batteryEl.textContent = '获取失败';
                }
            })
            .catch(error => {
                console.error('获取电池电压失败:', error);
                document.getElementById('battery').innerHTML = '获取失败';
            });
    }

    // 重启系统
    function restartSystem() {
        showConfirm('确定要重启系统吗？\n\n重启后将回到正常门禁模式，Web服务器将停止运行。', {
            type: 'danger',
            title: '确认重启'
        }).then(result => {
            if (result) {
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
                    });
            }
        });
    }

    // 暴露给全局
    window.restartSystem = restartSystem;

    // 页面加载完成后初始化
    window.addEventListener('DOMContentLoaded', init);
})();