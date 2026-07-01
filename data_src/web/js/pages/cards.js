// 卡片管理页面脚本

// 当前读取到的卡片UID
let currentCardUID = "";

// 页面加载完成后初始化
document.addEventListener("DOMContentLoaded", function () {
  loadCardsList();
  loadCardLogicConfig();

  // WebSocket 重连后自动刷新卡片列表（去抖：2s 内的重复事件忽略）
  let lastWsRefreshTs = 0;
  document.addEventListener("ws-connected", function () {
    const now = Date.now();
    if (now - lastWsRefreshTs < 2000) return;
    lastWsRefreshTs = now;
    loadCardsList();
  });
});

// 读取卡片
async function readCard() {
  const readBtn = document.getElementById("readBtn");
  const statusText = document.getElementById("statusText");
  const readerStatus = document.getElementById("readerStatus");

  // 禁用按钮，显示读取中状态
  readBtn.disabled = true;
  readBtn.innerHTML = `
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M21 12a9 9 0 1 1-6.219-8.56"></path>
        </svg>
        读取中...
    `;
  statusText.textContent = "正在读取...";
  readerStatus.classList.add("reading");

  try {
    const response = await fetch("/api/cards/read");
    const result = await response.json();

    if (result.success) {
      // 显示读取到的UID并自动填入输入框
      currentCardUID = result.data.uid;
      document.getElementById("cardUID").value = currentCardUID;

      // 更新状态显示
      statusText.textContent = "读取成功";
      readerStatus.classList.remove("reading");
      readerStatus.classList.add("success");

      showToast("读取成功: " + currentCardUID, "success");
    } else {
      // 读取失败
      statusText.textContent = "读取失败";
      readerStatus.classList.remove("reading");
      readerStatus.classList.add("error");

      showToast(result.message || "读取失败", "error");
    }
  } catch (error) {
    console.error("读取卡片失败:", error);
    statusText.textContent = "连接失败";
    readerStatus.classList.remove("reading");
    readerStatus.classList.add("error");
    showToast("连接失败，请重试", "error");
  }

  // 恢复按钮
  readBtn.disabled = false;
  readBtn.innerHTML = `
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <path d="M3 7V5a2 2 0 0 1 2-2h2"></path>
            <path d="M17 3h2a2 2 0 0 1 2 2v2"></path>
            <path d="M21 17v2a2 2 0 0 1-2 2h-2"></path>
            <path d="M7 21H5a2 2 0 0 1-2-2v-2"></path>
            <line x1="12" y1="8" x2="12" y2="16"></line>
            <line x1="8" y1="12" x2="16" y2="12"></line>
        </svg>
        读取
    `;

  // 3秒后重置状态
  setTimeout(() => {
    statusText.textContent = "等待读取...";
    readerStatus.classList.remove("success", "error");
  }, 3000);
}

// 添加卡片
async function addCard() {
  const uidInput = document.getElementById("cardUID");
  const nameInput = document.getElementById("cardName");

  const uid = uidInput.value.trim().toUpperCase();
  const name = nameInput.value.trim();

  // 验证输入
  if (!uid) {
    showToast("请输入卡UID", "warning");
    uidInput.focus();
    return;
  }

  // 验证UID格式（8位十六进制）
  const uidRegex = /^[0-9A-F]{8}$/;
  if (!uidRegex.test(uid)) {
    showToast("UID格式错误，应为8位十六进制（如A1B2C3D4）", "warning");
    uidInput.focus();
    return;
  }

  if (!name) {
    showToast("请输入卡片名称", "warning");
    nameInput.focus();
    return;
  }

  const addBtn = document.querySelector('button[onclick="addCard()"]');
  setButtonLoading(addBtn, true);

  // 发送添加请求
  try {
    const response = await fetch("/api/cards", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ uid, name }),
    });

    const result = await response.json();

    if (result.success) {
      showToast("卡片添加成功", "success");
      // 清空输入
      uidInput.value = "";
      nameInput.value = "";
      currentCardUID = "";
      // 刷新列表
      loadCardsList();
    } else {
      showToast(result.message || "添加失败", "error");
    }
  } catch (error) {
    console.error("添加卡片失败:", error);
    showToast("连接失败，请重试", "error");
  } finally {
    setButtonLoading(addBtn, false);
  }
}

// 删除卡片
async function deleteCard(uid) {
  const deleteBtn = document.querySelector('.card-item[data-uid="' + uid + '"] .btn-icon');
  const confirmed = await showConfirm(`确定要删除卡片 ${uid} 吗？`, {
    type: "danger",
    title: "确认删除",
  });

  if (!confirmed) {
    return;
  }

  setButtonLoading(deleteBtn, true);
  try {
    const response = await fetch("/api/cards?uid=" + encodeURIComponent(uid), {
      method: "DELETE",
    });

    const result = await response.json();

    if (result.success) {
      showToast("卡片删除成功", "success");
      // 刷新列表
      loadCardsList();
    } else {
      showToast(result.message || "删除失败", "error");
    }
  } catch (error) {
    console.error("删除卡片失败:", error);
    showToast("连接失败，请重试", "error");
  } finally {
    setButtonLoading(deleteBtn, false);
  }
}

// 加载卡片列表
async function loadCardsList() {
  const cardsList = document.getElementById("cardsList");
  const cardCount = document.getElementById("cardCount");

  try {
    const response = await fetch("/api/cards");
    const result = await response.json();

    if (result.success) {
      const cards = result.data.cards;
      cardCount.textContent = `(${cards.length})`;

      if (cards.length === 0) {
        // 显示空状态
        cardsList.innerHTML = `
                    <div class="empty-state">
                        <svg viewBox="0 0 24 24" fill="none" stroke="#ccc" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                            <rect x="2" y="5" width="20" height="14" rx="2"></rect>
                            <line x1="2" y1="10" x2="22" y2="10"></line>
                        </svg>
                        <p>暂无授权卡片</p>
                    </div>
                `;
        return;
      }

      // 渲染卡片列表
      let html = "";
      cards.forEach((card) => {
        html += `
                    <div class="card-item" data-uid="${card.uid}">
                        <div class="card-content">
                            <div class="card-uid">${card.uid}</div>
                            <div class="card-name">${escapeHtml(card.name)}</div>
                        </div>
                        <button onclick="deleteCard('${card.uid}')" class="btn-icon btn-danger" aria-label="删除卡片">
                            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                                <polyline points="3 6 5 6 21 6"></polyline>
                                <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path>
                            </svg>
                        </button>
                    </div>
                `;
      });
      cardsList.innerHTML = html;
    } else {
      showToast(result.message || "加载失败", "error");
    }
  } catch (error) {
    console.error("加载卡片列表失败:", error);
    showToast("连接失败，请重试", "error");
  }
}

// 加载卡片逻辑配置
async function loadCardLogicConfig() {
  try {
    const response = await fetch("/api/cards/logic-enabled");
    const result = await response.json();

    if (result.success) {
      document.getElementById("cardLogicToggle").checked = result.data.enabled;
    }
  } catch (error) {
    console.error("加载卡片逻辑配置失败:", error);
  }
}

// 切换卡片逻辑
async function toggleCardLogic() {
  const enabled = document.getElementById("cardLogicToggle").checked;

  try {
    const response = await fetch("/api/cards/logic-enabled", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ enabled }),
    });

    const result = await response.json();

    if (result.success) {
      showToast(enabled ? "刷卡逻辑已启用" : "刷卡逻辑已禁用", "success");
    } else {
      showToast(result.message || "设置失败", "error");
      // 恢复开关状态
      document.getElementById("cardLogicToggle").checked = !enabled;
    }
  } catch (error) {
    console.error("设置卡片逻辑失败:", error);
    showToast("连接失败，请重试", "error");
    // 恢复开关状态
    document.getElementById("cardLogicToggle").checked = !enabled;
  }
}
