  // 舵机管理页面逻辑模块
(function () {
  let actionInProgress = false;
  const MAX_VALUE = 1280;
  let testPositionDebounceTimer = null;
  
  // 队列重试相关变量
  let pendingPosition = null;    // 待执行的位置
  let retryCount = 0;          // 重试次数
  let isWaiting = false;        // 是否在等待中
  const MAX_RETRY = 10;        // 最大重试次数
  const RETRY_DELAY = 500;     // 重试延迟(ms)

  // 初始化
  function init() {
    loadConfig();
    setupSliderSync();
  }

  // 执行位置控制（带自动重试）
  function executePosition(position) {
    fetch("/api/servo/actions/position", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ position: position })
    })
      .then((response) => response.json())
      .then((response) => {
        if (response.success) {
          isWaiting = false;
          pendingPosition = null;
          retryCount = 0;
        } else {
          if (response.code === 409) {
            // 舵机忙碌，加入队列等待重试
            pendingPosition = position;
            isWaiting = true;
            retryCount++;
            
            if (retryCount <= MAX_RETRY) {
              showToast("舵机忙碌，等待重试 (" + retryCount + "/" + MAX_RETRY + ")...", "warning");
              setTimeout(() => {
                executePosition(pendingPosition);
              }, RETRY_DELAY);
            } else {
              // 超时
              isWaiting = false;
              pendingPosition = null;
              retryCount = 0;
              showToast("等待超时，请重试", "error");
            }
          } else {
            // 其他错误
            isWaiting = false;
            pendingPosition = null;
            retryCount = 0;
            showToast("位置控制失败: " + (response.message || "未知错误"), "error");
          }
        }
      })
      .catch((err) => {
        // 网络错误，不重试
        isWaiting = false;
        pendingPosition = null;
        retryCount = 0;
        console.error("位置控制请求失败:", err);
      });
  }

  // 发送测试位置指令（带防抖和队列）
  function sendTestPosition(position) {
    clearTimeout(testPositionDebounceTimer);
    
    // 如果已经在等待舵机空闲，只更新队列中的位置
    if (isWaiting) {
      pendingPosition = position;
      retryCount = 0; // 重置重试计数
      return;
    }
    
    testPositionDebounceTimer = setTimeout(() => {
      executePosition(position);
    }, 200); // 200ms防抖延迟
  }

  // 设置滑动条与输入框双向同步
  function setupSliderSync() {
    const unlockSlider = document.getElementById("unlockSlider");
    const unlockInput = document.getElementById("unlockPos");
    const lockSlider = document.getElementById("lockSlider");
    const lockInput = document.getElementById("lockPos");
    const testSlider = document.getElementById("testSlider");
    const testInput = document.getElementById("testPos");

    // 解锁位置滑动条事件
    unlockSlider.addEventListener("input", function () {
      unlockInput.value = this.value;
      updatePercent("unlock", this.value);
    });

    // 解锁位置输入框事件
    unlockInput.addEventListener("input", function () {
      let value = parseInt(this.value) || 0;
      value = Math.max(0, Math.min(MAX_VALUE, value));
      unlockSlider.value = value;
      updatePercent("unlock", value);
    });

    // 锁定位置滑动条事件
    lockSlider.addEventListener("input", function () {
      lockInput.value = this.value;
      updatePercent("lock", this.value);
    });

    // 锁定位置输入框事件
    lockInput.addEventListener("input", function () {
      let value = parseInt(this.value) || 0;
      value = Math.max(0, Math.min(MAX_VALUE, value));
      lockSlider.value = value;
      updatePercent("lock", value);
    });

    // 测试位置滑动条事件
    testSlider.addEventListener("input", function () {
      testInput.value = this.value;
      updatePercent("test", this.value);
      sendTestPosition(this.value);
    });

    // 测试位置输入框事件
    testInput.addEventListener("input", function () {
      let value = parseInt(this.value) || 0;
      value = Math.max(0, Math.min(MAX_VALUE, value));
      testSlider.value = value;
      updatePercent("test", value);
      sendTestPosition(value);
    });
  }

  // 更新百分比显示
  function updatePercent(type, value) {
    const percentElement = document.getElementById(type + "Percent");
    const percent = Math.round((value / MAX_VALUE) * 100);
    percentElement.textContent = percent + "%";
  }

  function updateServoStatus(busy) {
    const statusDiv = document.getElementById("servoStatus");
    const unlockBtn = document.getElementById("unlockBtn");
    const lockBtn = document.getElementById("lockBtn");

    if (busy) {
      statusDiv.className = "servo-status busy";
      statusDiv.innerHTML =
        '<span class="status-dot"></span><span>状态: 执行中...</span>';
      unlockBtn.disabled = true;
      lockBtn.disabled = true;
    } else {
      statusDiv.className = "servo-status idle";
      statusDiv.innerHTML =
        '<span class="status-dot"></span><span>状态: 空闲</span>';
      unlockBtn.disabled = false;
      lockBtn.disabled = false;
    }
  }

  // 加载配置
  function loadConfig() {
    fetch("/api/servo/config")
      .then((response) => {
        if (!response.ok) throw new Error("HTTP " + response.status);
        return response.json();
      })
      .then((response) => {
        if (response.success && response.data) {
          const unlock = response.data.unlock;
          const lock = response.data.lock;

          document.getElementById("unlockPos").value = unlock;
          document.getElementById("unlockSlider").value = unlock;
          updatePercent("unlock", unlock);

          document.getElementById("lockPos").value = lock;
          document.getElementById("lockSlider").value = lock;
          updatePercent("lock", lock);

          showToast("配置加载成功", "success");
        } else {
          showToast(
            "加载配置失败: " + (response.message || "未知错误"),
            "error",
          );
        }
      })
      .catch((err) => showToast("加载配置失败: " + err.message, "error"));
  }

  // 保存配置
  function saveConfig() {
    const unlock = parseInt(document.getElementById("unlockPos").value);
    const lock = parseInt(document.getElementById("lockPos").value);

    if (isNaN(unlock) || isNaN(lock)) {
      showToast("请输入有效的位置值", "error");
      return;
    }

    if (unlock < 0 || unlock > 1280 || lock < 0 || lock > 1280) {
      showToast("位置值必须在0-1280范围内", "error");
      return;
    }

    fetch("/api/servo/config", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ unlock: unlock, lock: lock }),
    })
      .then((response) => response.json())
      .then((response) => {
        if (response.success) {
          showToast("配置保存成功", "success");
        } else {
          showToast(
            "保存配置失败: " + (response.message || "未知错误"),
            "error",
          );
        }
      })
      .catch((err) => showToast("请求失败: " + err.message, "error"));
  }

  // 解锁
  function unlock() {
    if (actionInProgress) return;
    actionInProgress = true;
    updateServoStatus(true);

    fetch("/api/servo/actions/unlock", { method: "POST" })
      .then((response) => response.json())
      .then((response) => {
        if (response.success) {
          showToast("解锁指令已发送", "success");
        } else {
          showToast("解锁失败: " + (response.message || "未知错误"), "error");
        }
      })
      .catch((err) => showToast("请求失败: " + err.message, "error"))
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

    fetch("/api/servo/actions/lock", { method: "POST" })
      .then((response) => response.json())
      .then((response) => {
        if (response.success) {
          showToast("锁定指令已发送", "success");
        } else {
          showToast("锁定失败: " + (response.message || "未知错误"), "error");
        }
      })
      .catch((err) => showToast("请求失败: " + err.message, "error"))
      .finally(() => {
        setTimeout(() => {
          actionInProgress = false;
          updateServoStatus(false);
        }, 1500);
      });
  }

  // 暴露给全局
  window.saveConfig = saveConfig;
  window.unlock = unlock;
  window.lock = lock;

  // 页面加载完成后初始化
  window.addEventListener("DOMContentLoaded", init);
})();
