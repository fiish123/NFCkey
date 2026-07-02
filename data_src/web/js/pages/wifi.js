// WiFi配置页面逻辑模块
(function() {
    let selectedNetwork = null;
    let isAPMode = false;
    let autoRefreshTimer = null;

    // 初始化
    function init() {
        waitForWebSocketReady()
            .then(getWifiStatus)
            .catch((error) => {
                console.error('等待 WebSocket 连接失败:', error);
                document.getElementById('wifiStatusText').textContent = '获取失败';
                showToast('WebSocket 连接失败，请稍后重试', 'error');
            });
        
        initPasswordToggle();
        initKeyboardSupport();
        initPasswordValidation();
    }

    // 获取WiFi状态
    function getWifiStatus() {
        sendWsRequest('wifi/getInfo')
            .then(data => {
                const banner = document.getElementById('wifiStatusBanner');
                const statusDot = document.getElementById('wifiStatusDot');
                const statusText = document.getElementById('wifiStatusText');
                let stateClass = 'state-muted';

                if (data.mode === 'AP') {
                    stateClass = 'state-warning';
                    statusText.textContent = 'AP模式: ' + data.ssid;
                    isAPMode = true;
                } else if (data.ssid) {
                    stateClass = 'state-success';
                    statusText.textContent = '已连接: ' + data.ssid + ' (' + data.ip + ')';
                    isAPMode = false;
                } else {
                    statusText.textContent = '未连接';
                    isAPMode = false;
                }

                // 语义状态由 banner 承载（success/warning/muted），
                // 驱动左边框、图标、标签和圆点颜色；圆点交由 CSS 着色。
                banner.className = 'wifi-status-banner ' + stateClass;
                statusDot.className = 'status-dot';
            })
            .catch(error => {
                console.error('获取WiFi状态失败:', error);
                document.getElementById('wifiStatusText').textContent = '获取失败';
            });
    }

    // 启动自动刷新
    function startAutoRefresh() {
        stopAutoRefresh();
        autoRefreshTimer = setInterval(() => {
            getWifiStatus();
        }, 5000); // 每5秒刷新一次
    }

    // 停止自动刷新
    function stopAutoRefresh() {
        if (autoRefreshTimer) {
            clearInterval(autoRefreshTimer);
            autoRefreshTimer = null;
        }
    }

    // 扫描WiFi网络
    function scanWifi() {
        const wifiList = document.getElementById('wifiList');
        const scanBtn = document.getElementById('scanBtn');
        
        // 显示雷达扫描进度
        wifiList.innerHTML = `
            <div class="scan-progress">
                <div class="radar-animation">
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                        <path d="M5 12.55a11 11 0 0 1 14.08 0"></path>
                        <path d="M1.42 9a16 16 0 0 1 21.16 0"></path>
                        <path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>
                        <line x1="12" y1="20" x2="12.01" y2="20"></line>
                    </svg>
                </div>
                <p>正在扫描WiFi网络...</p>
            </div>
        `;
        
        // 禁用扫描按钮
        scanBtn.disabled = true;
        scanBtn.innerHTML = `
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                <path d="M5 12.55a11 11 0 0 1 14.08 0"></path>
                <path d="M1.42 9a16 16 0 0 1 21.16 0"></path>
                <path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>
                <line x1="12" y1="20" x2="12.01" y2="20"></line>
            </svg>
            扫描中...
        `;
        
        // 启动自动刷新
        startAutoRefresh();
        
        // 设置超时定时器（35秒）
        let timeoutTimer = setTimeout(() => {
            offWsEvent('wifi/scanResult', scanResultHandler);
            scanBtn.disabled = false;
            scanBtn.innerHTML = `
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                    <path d="M5 12.55a11 11 0 0 1 14.08 0"></path>
                    <path d="M1.42 9a16 16 0 0 1 21.16 0"></path>
                    <path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>
                    <line x1="12" y1="20" x2="12.01" y2="20"></line>
                </svg>
                扫描WiFi网络
            `;
            wifiList.innerHTML = `
                <p class="empty">扫描超时</p>
                <div class="wifi-result-actions">
                    <button onclick="scanWifi()" class="btn btn-sm">重新扫描</button>
                </div>
            `;
            showToast('扫描超时，请重试', 'error');
        }, 35000);
        
        // 注册扫描结果监听器
        const scanResultHandler = (message) => {
            clearTimeout(timeoutTimer);
            offWsEvent('wifi/scanResult', scanResultHandler);
            
            if (message.success && message.data) {
                displayWifiList(message.data);
            } else {
                const errorMessage = message.data?.message || '未找到WiFi网络';
                wifiList.innerHTML = `
                    <p class="empty">${escapeHtml(errorMessage)}</p>
                    <div class="wifi-result-actions">
                        <button onclick="scanWifi()" class="btn btn-sm">重新扫描</button>
                    </div>
                `;
                if (errorMessage !== '未找到WiFi网络') {
                    showToast('扫描失败：' + errorMessage, 'error');
                }
            }
        };
        
        onWsEvent('wifi/scanResult', scanResultHandler);
        
        // 发送扫描请求
        sendWsRequest('wifi/scan')
            .catch(error => {
                clearTimeout(timeoutTimer);
                offWsEvent('wifi/scanResult', scanResultHandler);
                console.error('扫描WiFi失败:', error);
                wifiList.innerHTML = `
                    <p class="empty">扫描失败</p>
                    <div class="wifi-result-actions">
                        <button onclick="scanWifi()" class="btn btn-sm">重新扫描</button>
                    </div>
                `;
                showToast('扫描失败：' + error.message, 'error');
            })
            .finally(() => {
                // 恢复扫描按钮
                scanBtn.disabled = false;
                scanBtn.innerHTML = `
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                        <path d="M5 12.55a11 11 0 0 1 14.08 0"></path>
                        <path d="M1.42 9a16 16 0 0 1 21.16 0"></path>
                        <path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>
                        <line x1="12" y1="20" x2="12.01" y2="20"></line>
                    </svg>
                    扫描WiFi网络
                `;
            });
    }

    // 显示WiFi列表
    function displayWifiList(networks) {
        const wifiList = document.getElementById('wifiList');
        wifiList.innerHTML = '';
        
        // 检查是否为数组
        if (!Array.isArray(networks)) {
            console.error('networks 不是数组:', networks);
            wifiList.innerHTML = `
                <p class="empty">扫描结果格式错误</p>
                <div class="wifi-result-actions">
                    <button onclick="scanWifi()" class="btn btn-sm">重新扫描</button>
                </div>
            `;
            showToast('扫描结果格式错误', 'error');
            return;
        }
        
        if (networks.length === 0) {
            wifiList.innerHTML = `
                <p class="empty">未找到WiFi网络</p>
                <div class="wifi-result-actions">
                    <button onclick="scanWifi()" class="btn btn-sm">重新扫描</button>
                </div>
            `;
            return;
        }
        
        // 按信号强度排序
        networks.sort((a, b) => b.rssi - a.rssi);
        
        // 添加扫描结果提示
        const countInfo = document.createElement('p');
        countInfo.className = 'wifi-count';
        countInfo.textContent = `找到 ${networks.length} 个WiFi网络`;
        wifiList.appendChild(countInfo);
        
        networks.forEach((network, index) => {
            const div = document.createElement('div');
            div.className = 'wifi-item';
            div.onclick = () => selectNetwork(network, div);
            
            const signalStrength = getSignalStrength(network.rssi);
            const encryptionText = getEncryptionText(network.encryption);
            const signalLevel = getSignalLevel(network.rssi);
            
            // 添加延迟动画（通过类切换，无内联样式；时序 index*50 保持不变）
            div.classList.add('wifi-item-enter');
            setTimeout(() => {
                div.classList.add('wifi-item-visible');
            }, index * 50);
            
            // 检查是否是当前连接的网络
            const currentStatusText = document.getElementById('wifiStatusText').textContent;
            const isConnected = currentStatusText.includes(network.ssid);
            
            div.innerHTML = `
                <div class="wifi-info">
                    <svg class="wifi-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                        <path d="M5 12.55a11 11 0 0 1 14.08 0"></path>
                        <path d="M1.42 9a16 16 0 0 1 21.16 0"></path>
                        <path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>
                        <line x1="12" y1="20" x2="12.01" y2="20"></line>
                    </svg>
                    <div class="wifi-main">
                        <div class="wifi-name-row">
                            <span class="wifi-name">${escapeHtml(network.ssid)}</span>
                            ${isConnected ? `
                                <span class="connected-badge">
                                    <span class="status-dot"></span>
                                    已连接
                                </span>
                            ` : ''}
                        </div>
                        <div class="wifi-strength">信号: ${signalStrength} (${network.rssi} dBm)</div>
                    </div>
                    <div class="wifi-right">
                        <div class="signal-icon signal-${signalLevel}" title="信号强度: ${signalStrength}">
                            <div class="signal-bar"></div>
                            <div class="signal-bar"></div>
                            <div class="signal-bar"></div>
                            <div class="signal-bar"></div>
                        </div>
                        ${network.encryption !== 0 ? `
                            <div class="encryption-icon" title="${encryptionText}">
                                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                                    <rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect>
                                    <path d="M7 11V7a5 5 0 0 1 10 0v4"></path>
                                </svg>
                            </div>
                        ` : ''}
                    </div>
                </div>
            `;
            
            wifiList.appendChild(div);
        });
    }
    
    // 获取信号等级（1-4）
    function getSignalLevel(rssi) {
        if (rssi > -50) return 4;
        if (rssi > -60) return 3;
        if (rssi > -70) return 2;
        return 1;
    }

    // 选择WiFi网络
    function selectNetwork(network, element) {
        // 移除之前的选择
        document.querySelectorAll('.wifi-item').forEach(item => {
            item.classList.remove('selected');
        });
        
        // 添加新选择
        element.classList.add('selected');
        selectedNetwork = network;
        
        // 显示配置表单
        const wifiConfig = document.getElementById('wifiConfig');
        wifiConfig.style.display = 'block';
        document.getElementById('selectedSSID').textContent = network.ssid;
        document.getElementById('wifiPassword').value = '';
        document.getElementById('wifiPassword').classList.remove('input-error');
        document.getElementById('passwordError').classList.remove('visible');
        
        // 聚焦密码输入框
        document.getElementById('wifiPassword').focus();
        
        // 平滑滚动到配置表单
        wifiConfig.scrollIntoView({ behavior: 'smooth', block: 'center' });
    }

    // 初始化密码显示/隐藏切换
    function initPasswordToggle() {
        const toggle = document.getElementById('passwordToggle');
        const passwordInput = document.getElementById('wifiPassword');
        let isVisible = false;
        
        toggle.addEventListener('click', function() {
            isVisible = !isVisible;
            passwordInput.type = isVisible ? 'text' : 'password';
            
            // 切换图标
            if (isVisible) {
                toggle.innerHTML = `
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                        <path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24"></path>
                        <line x1="1" y1="1" x2="23" y2="23"></line>
                    </svg>
                `;
                toggle.setAttribute('aria-label', '隐藏密码');
            } else {
                toggle.innerHTML = `
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                        <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11 8z"></path>
                        <circle cx="12" cy="12" r="3"></circle>
                    </svg>
                `;
                toggle.setAttribute('aria-label', '显示密码');
            }
        });
    }

    // 初始化键盘支持
    function initKeyboardSupport() {
        const passwordInput = document.getElementById('wifiPassword');
        
        // Enter键提交
        passwordInput.addEventListener('keydown', function(e) {
            if (e.key === 'Enter') {
                e.preventDefault();
                if (selectedNetwork) {
                    testConnection();
                }
            }
        });
        
        // ESC键取消选择
        document.addEventListener('keydown', function(e) {
            if (e.key === 'Escape' && selectedNetwork) {
                e.preventDefault();
                // 取消选择
                document.querySelectorAll('.wifi-item').forEach(item => {
                    item.classList.remove('selected');
                });
                selectedNetwork = null;
                document.getElementById('wifiConfig').style.display = 'none';
            }
        });
    }

    // 初始化密码验证
    function initPasswordValidation() {
        const passwordInput = document.getElementById('wifiPassword');
        const passwordError = document.getElementById('passwordError');
        
        passwordInput.addEventListener('input', function() {
            // 隐藏错误提示
            passwordInput.classList.remove('input-error');
            passwordError.classList.remove('visible');
        });
        
        passwordInput.addEventListener('blur', function() {
            if (selectedNetwork && selectedNetwork.encryption !== 0 && this.value.length === 0) {
                passwordInput.classList.add('input-error');
                passwordError.classList.add('visible');
            }
        });
    }

    // 测试连接
    function testConnection() {
        if (!selectedNetwork) {
            showToast('请先选择WiFi网络', 'warning');
            return;
        }
        
        const passwordInput = document.getElementById('wifiPassword');
        const passwordError = document.getElementById('passwordError');
        const password = passwordInput.value;
        
        if (selectedNetwork.encryption !== 0 && password.length === 0) {
            passwordInput.classList.add('input-error');
            passwordError.classList.add('visible');
            showToast('请输入WiFi密码', 'warning');
            passwordInput.focus();
            return;
        }
        
        const testBtn = document.getElementById('testBtn');
        const statusText = document.getElementById('wifiStatusText');
        
        // 禁用按钮并显示加载状态
        setButtonLoading(testBtn, true);
        statusText.textContent = '测试连接中...';
        
        // 注册测试结果监听器
        const testResultHandler = (message) => {
            offWsEvent('wifi/testResult', testResultHandler);
            
            const testBtn = document.getElementById('testBtn');
            setButtonLoading(testBtn, false);
            
            if (message.success && message.data) {
                const data = message.data;
                if (data.success) {
                    showToast('连接成功！IP: ' + data.ip, 'success');
                    statusText.textContent = '连接成功: ' + data.ip;
                    setTimeout(() => {
                        getWifiStatus();
                    }, 2000);
                } else {
                    showToast('连接失败: ' + (data.errorMessage || '未知错误'), 'error');
                    statusText.textContent = '连接失败';
                    passwordInput.classList.add('input-error');
                    setTimeout(() => {
                        getWifiStatus();
                    }, 2000);
                }
            }
        };
        
        onWsEvent('wifi/testResult', testResultHandler);
        
        // 发送测试请求
        sendWsRequest('wifi/test', {
            ssid: selectedNetwork.ssid,
            password: password
        }).catch(error => {
            offWsEvent('wifi/testResult', testResultHandler);
            console.error('测试请求失败:', error);
            const testBtn = document.getElementById('testBtn');
            setButtonLoading(testBtn, false);
            showToast('测试失败：' + error.message, 'error');
        });
    }

    // 保存配置
    function saveConfig() {
        if (!selectedNetwork) {
            showToast('请先选择WiFi网络', 'warning');
            return;
        }
        
        const passwordInput = document.getElementById('wifiPassword');
        const password = passwordInput.value;
        
        if (selectedNetwork.encryption !== 0 && password.length === 0) {
            passwordInput.classList.add('input-error');
            document.getElementById('passwordError').classList.add('visible');
            showToast('请输入WiFi密码', 'warning');
            passwordInput.focus();
            return;
        }
        
        showConfirm('确定要保存WiFi配置吗？', {
            type: 'info',
            title: '确认保存'
        }).then(result => {
            if (result) {
                const saveBtn = document.getElementById('saveBtn');
                setButtonLoading(saveBtn, true);
                
                sendWsRequest('wifi/saveConfig', {
                    ssid: selectedNetwork.ssid,
                    password: password
                })
                .then(() => {
                    showToast('配置保存成功', 'success');
                    stopAutoRefresh();
                    setTimeout(() => {
                        location.reload();
                    }, 3000);
                })
                .catch(error => {
                    console.error('保存配置失败:', error);
                    showToast('保存失败：' + error.message, 'error');
                    setButtonLoading(saveBtn, false);
                });
            }
        });
    }

    // 清除配置
    function clearConfig() {
        showConfirm('确定要清除WiFi配置吗？', {
            type: 'danger',
            title: '确认清除'
        }).then(result => {
            if (result) {
                const clearBtn = document.getElementById('clearBtn');
                setButtonLoading(clearBtn, true);
                
                sendWsRequest('wifi/clearConfig')
                .then(() => {
                    showToast('配置已清除', 'info');
                    stopAutoRefresh();
                    setTimeout(() => {
                        location.reload();
                    }, 3000);
                })
                .catch(error => {
                    console.error('清除配置失败:', error);
                    showToast('清除失败：' + error.message, 'error');
                    setButtonLoading(clearBtn, false);
                });
            }
        });
    }

    // 设置按钮加载状态：使用 common.js 提供的全局 setButtonLoading(element, bool)
    // （切换锁定 .btn-loading 类 + disabled，统一加载态视觉）。

    // 获取信号强度描述
    function getSignalStrength(rssi) {
        if (rssi > -50) return '强';
        if (rssi > -60) return '良好';
        if (rssi > -70) return '一般';
        return '弱';
    }

    // 获取加密类型描述
    function getEncryptionText(encryption) {
        switch (encryption) {
            case 0: return '开放';
            case 1: return 'WEP';
            case 2: return 'WPA-PSK';
            case 3: return 'WPA2-PSK';
            case 4: return 'WPA/WPA2-PSK';
            default: return '加密';
        }
    }
    // 暴露给全局
    window.scanWifi = scanWifi;
    window.testConnection = testConnection;
    window.saveConfig = saveConfig;
    window.clearConfig = clearConfig;

    // 页面加载完成后初始化
    window.addEventListener('DOMContentLoaded', init);

    // 页面卸载时清理定时器
    window.addEventListener('beforeunload', function() {
        stopAutoRefresh();
    });
})();
