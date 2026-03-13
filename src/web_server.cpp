/**
 * @file web_server.cpp
 * @brief Web服务器实现，提供WiFi连接、文件管理、OTA升级和舵机控制API。
 *        注：WiFi管理功能已迁移至WebSocket通信。
 */

#include "web_server.h"

// 声明外部变量和函数
struct NFCcard
{
    uint8_t uid[4];
    uint8_t uidLength;
    String name;
};
extern std::vector<NFCcard> authorizedCards;
extern const char *CARDS_FILE_PATH;
extern void loadCardsDataFromFile();
extern bool saveCardsToFile();
extern NFCcard ReadCard();
extern void sendCardSearchCommand();
// extern uint8_t connectchoice;  // 未使用，已删除
extern bool cardLogicEnabled;

// 全局 Web 服务器实例
AsyncWebServer *server = nullptr;
// Web 服务器运行状态
WebServerMode webserverMode = WS_MODE_OFF;

// WebSocket 服务器实例
AsyncWebSocket *ws = nullptr;
// 日志缓存
std::deque<LogEntry> logCache;
#define LOG_CACHE_SIZE 500

// WiFi Preferences存储
Preferences wifiPreferences;
const char *WIFI_PREF_NAMESPACE = "wifi";
const char *WIFI_PREF_SSID = "ssid";
const char *WIFI_PREF_PASSWORD = "password";

// AP模式配置
const char *AP_SSID = "NFCKey-AP";
const char *AP_PASSWORD = "";
const int AP_CHANNEL = 1;
const bool AP_HIDDEN = false;
const int AP_MAX_CONNECTION = 1;

// 服务器配置
static WebServerConfig serverConfig = {
    "1.0.0",
    80};

// 文件上传状态管理（类型来自头文件）
static FileUploadState uploadState;

// WiFi测试状态管理（仍用于异步任务协调）
static bool wifiTestRunning = false;
static bool wifiTestSuccess = false;
static String wifiTestResultIP = "";
static String wifiTestErrorMessage = "";
static uint32_t wifiTestClientId = 0;  // 发起测试的客户端ID
static uint32_t wifiTestRequestId = 0; // 请求ID

// WiFi扫描状态管理
static bool wifiScanRunning = false;

// WiFi测试参数结构体
struct WifiTestParams
{
    uint32_t clientId;
    String ssid;
    String password;
};

uint32_t uploadsize = 0; // 请求ID

// =============================================================================
// 辅助工具函数
// =============================================================================

/**
 * @brief 规范化文件路径：确保以 '/' 开头，移除重复斜杠。
 * @param path 原始路径
 * @return 规范化后的路径
 */
String normalizePath(String path) // 按值传递，允许修改副本
{
    if (path.length() == 0 || path[0] != '/')
        path = '/' + path;

    int writePos = 1;
    bool prevWasSlash = true;

    for (unsigned int readPos = 1; readPos < path.length(); readPos++)
    {
        char c = path[readPos];
        if (c == '/')
        {
            if (!prevWasSlash)
                path[writePos++] = c;
            prevWasSlash = true;
        }
        else
        {
            path[writePos++] = c;
            prevWasSlash = false;
        }
    }

    if (writePos > 1 && path[writePos - 1] == '/')
        writePos--;

    path.remove(writePos);
    return path;
}

/**
 * @brief 从请求中获取路径参数，规范化后返回。
 * @param request       HTTP请求指针
 * @param paramName     参数名
 * @param isPostParam   是否为POST参数（true为POST表单参数，false为URL参数）
 * @param defaultPath   默认路径
 * @param outPath       输出规范化后的路径
 * @return true 如果路径非空，false 如果路径为空
 */
static bool getPathParam(AsyncWebServerRequest *request, const char *paramName,
                         bool isPostParam, const char *defaultPath, String &outPath)
{
    if (request->hasParam(paramName, isPostParam))
        outPath = request->getParam(paramName, isPostParam)->value();
    else
        outPath = defaultPath;

    outPath = normalizePath(outPath);
    return !outPath.isEmpty(); // 仅检查非空
}

/**
 * @brief 验证舵机位置是否在有效范围内（0-1280）。
 * @param position 舵机位置值
 * @return true 有效，false 无效
 */
bool validateServoPosition(uint16_t position)
{
    return position <= 1280;
}

/**
 * @brief 验证OTA文件大小是否≤2MB。
 * @param size 文件大小（字节）
 * @return true 有效，false 无效
 */
bool validateOtaFileSize(size_t size)
{
    return size <= 2097152;
}

// =============================================================================
// WiFi配置相关函数
// =============================================================================

/**
 * @brief 从Preferences加载WiFi配置
 * @param ssid 输出SSID
 * @param password 输出密码
 * @return true 成功，false 失败或配置为空
 */
bool loadWifiConfig(String &ssid, String &password)
{
    wifiPreferences.begin(WIFI_PREF_NAMESPACE, true);
    ssid = wifiPreferences.getString(WIFI_PREF_SSID, "");
    password = wifiPreferences.getString(WIFI_PREF_PASSWORD, "");
    wifiPreferences.end();

    if (ssid.length() == 0 || password.length() == 0)
    {
        LOG_D("WiFi配置未保存，配置为空");
        return false;
    }

    LOG_I("WiFi配置读取成功: SSID=%s", ssid.c_str());
    return true;
}

/**
 * @brief 保存WiFi配置到Preferences
 * @param ssid WiFi SSID
 * @param password WiFi密码
 * @return true 成功，false 失败
 */
bool saveWifiConfig(const String &ssid, const String &password)
{
    if (ssid.length() == 0 || ssid.length() > 32 || password.length() > 63)
    {
        LOG_E("WiFi配置参数无效: SSID长度=%d, 密码长度=%d", ssid.length(), password.length());
        return false;
    }

    wifiPreferences.begin(WIFI_PREF_NAMESPACE, false);
    bool ssidOk = wifiPreferences.putString(WIFI_PREF_SSID, ssid);
    bool passOk = wifiPreferences.putString(WIFI_PREF_PASSWORD, password);
    wifiPreferences.end();

    if (ssidOk && passOk)
    {
        LOG_I("WiFi配置已保存: SSID=%s", ssid.c_str());
        return true;
    }

    LOG_E("WiFi配置保存失败: Preferences写入错误");
    return false;
}

/**
 * @brief 清除WiFi配置
 * @return true 成功，false 失败
 */
bool clearWifiConfig()
{
    wifiPreferences.begin(WIFI_PREF_NAMESPACE, false);
    bool ssidOk = wifiPreferences.remove(WIFI_PREF_SSID);
    bool passOk = wifiPreferences.remove(WIFI_PREF_PASSWORD);
    wifiPreferences.end();

    if (ssidOk && passOk)
    {
        LOG_I("WiFi配置已清除");
        return true;
    }

    LOG_E("WiFi配置清除失败: Preferences删除错误");
    return false;
}

/**
 * @brief 启动AP模式（热点）
 */
void startAPMode()
{
    LOG_I("启动AP热点模式: SSID=%s", AP_SSID);

    WiFi.mode(WIFI_MODE_APSTA);
    WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, AP_HIDDEN, AP_MAX_CONNECTION);

    LOG_I("AP热点启动成功: IP=%s", WiFi.softAPIP().toString().c_str());
}

// =============================================================================
// 统一WiFi连接函数（静态，文件作用域）
// =============================================================================
static bool connectToWiFi(const String &ssid, const String &password, int maxAttempts = 20, int delayMs = 500)
{
    WiFi.begin(ssid.c_str(), password.c_str());
    for (int i = 0; i < maxAttempts; i++)
    {
        if (WiFi.status() == WL_CONNECTED)
            return true;
        vTaskDelay(pdMS_TO_TICKS(delayMs));
        esp_task_wdt_reset();
    }
    return false;
}

// =============================================================================
// API 响应处理（仅用于HTTP API，保持不变）
// =============================================================================

/**
 * @brief 将ApiResponse对象序列化为JSON字符串。
 * @return JSON字符串
 */
String ApiResponse::toJson() const
{
    JsonDocument doc;
    doc["success"] = success;
    doc["message"] = message;

    if (hasData)
    {
        doc["data"] = data; // 将 data 赋值给 "data" 字段
    }
    else
    {
        doc["data"] = nullptr; // 显式设置为 null
    }

    String response;
    serializeJson(doc, response);
    return response;
}

/**
 * @brief 发送JSON格式的API响应。
 * @param request  HTTP请求指针
 * @param response ApiResponse对象
 */
void sendApiResponse(AsyncWebServerRequest *request, const ApiResponse &response)
{
    request->send(response.code, "application/json", response.toJson());
}

// 函数前向声明
static void sendSuccessResponse(AsyncWebServerRequest *request);
static void sendSuccessResponse(AsyncWebServerRequest *request, const JsonDocument &data);

/**
 * @brief 发送成功响应（200 OK），无额外数据。
 * @param request HTTP请求指针
 */
static void sendSuccessResponse(AsyncWebServerRequest *request)
{
    JsonDocument emptyDoc;
    sendSuccessResponse(request, emptyDoc);
}

/**
 * @brief 发送成功响应（200 OK），带附加数据。
 * @param request HTTP请求指针
 * @param data    可选的附加数据（JSON文档）
 */
static void sendSuccessResponse(AsyncWebServerRequest *request, const JsonDocument &data)
{
    ApiResponse response;
    response.success = true;
    response.code = 200;
    response.message = "Success";
    response.data = data;
    response.hasData = !data.isNull();
    sendApiResponse(request, response);
}

/**
 * @brief 发送错误响应。
 * @param request HTTP请求指针
 * @param code    HTTP状态码
 * @param message 错误信息
 */
static void sendErrorResponse(AsyncWebServerRequest *request, int code, const String &message)
{
    if (code >= 500)
    {
        LOG_E("API错误响应: HTTP %d - %s", code, message.c_str());
    }
    else
    {
        LOG_W("API错误响应: HTTP %d - %s", code, message.c_str());
    }

    ApiResponse response;
    response.success = false;
    response.code = code;
    response.message = message;
    response.hasData = false;
    sendApiResponse(request, response);
}

// =============================================================================
// JSON 辅助处理函数（统一错误处理）
// =============================================================================

/**
 * @brief 处理包含JSON体的请求，自动解析并调用回调函数。
 * @param request HTTP请求指针
 * @param data    数据指针
 * @param len     当前数据块长度
 * @param index   数据块索引
 * @param total   总数据长度
 * @param func    回调函数，接受 request 和解析后的 JsonDocument
 */
template <typename Func>
static void handleJsonBody(AsyncWebServerRequest *request, uint8_t *data, size_t len,
                           size_t index, size_t total, Func func)
{
    if (index == 0)
    {
        // 可选：记录开始解析
    }
    // 只在最后一个数据块时处理
    if (index + len < total)
        return;

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, data, total);
    if (error)
    {
        LOG_E("JSON解析错误: %s", error.c_str());
        sendErrorResponse(request, 400, "无效的JSON");
        return;
    }
    func(request, doc);
}

// =============================================================================
// WebSocket 响应辅助函数
// =============================================================================

/**
 * @brief 通过WebSocket向指定客户端发送JSON响应。
 * @param client 客户端指针
 * @param action 动作类型（与请求对应）
 * @param success 是否成功
 * @param data 附加数据（可选）
 * @param requestId 请求ID（若请求中提供）
 */
static void sendWsResponse(AsyncWebSocketClient *client, const String &action, bool success,
                           const JsonDocument *data = nullptr, uint32_t requestId = 0)
{
    if (!client)
        return;

    JsonDocument doc;
    doc["action"] = action;
    doc["success"] = success;
    if (requestId != 0)
        doc["requestId"] = requestId;
    if (data && !data->isNull())
        doc["data"] = *data;

    String response;
    serializeJson(doc, response);
    client->text(response);
}

/**
 * @brief 发送错误响应。
 */
static void sendWsError(AsyncWebSocketClient *client, const String &action,
                        const String &message, uint32_t requestId = 0)
{
    JsonDocument errData;
    errData["message"] = message;
    sendWsResponse(client, action, false, &errData, requestId);
}

// =============================================================================
// WiFi 管理 WebSocket 处理函数（异步）
// =============================================================================

/**
 * @brief 处理获取当前WiFi信息
 */
void handleWsWifiGetInfo(AsyncWebSocketClient *client, const JsonDocument &req)
{
    JsonDocument data;
    data["ssid"] = WiFi.SSID().c_str();
    data["ip"] = WiFi.localIP().toString();
    data["rssi"] = WiFi.RSSI();
    data["mode"] = WiFi.getMode() == WIFI_AP ? "AP" : "STA";
    sendWsResponse(client, "wifi/getInfo", true, &data, req["requestId"]);
}

/**
 * @brief 处理保存WiFi配置
 */
void handleWsWifiSaveConfig(AsyncWebSocketClient *client, const JsonDocument &req)
{
    uint32_t requestId = req["requestId"] | 0;

    if (!req["ssid"].is<const char *>() || !req["password"].is<const char *>())
    {
        sendWsError(client, "wifi/saveConfig", "缺少ssid或password参数", requestId);
        return;
    }

    String ssid = req["ssid"].as<String>();
    String password = req["password"].as<String>();

    if (ssid.length() == 0)
    {
        sendWsError(client, "wifi/saveConfig", "SSID不能为空", requestId);
        return;
    }

    if (ssid.length() > 32 || password.length() > 63)
    {
        sendWsError(client, "wifi/saveConfig", "参数长度超出限制", requestId);
        return;
    }

    if (saveWifiConfig(ssid, password))
    {
        sendWsResponse(client, "wifi/saveConfig", true, nullptr, requestId);
    }
    else
    {
        sendWsError(client, "wifi/saveConfig", "保存失败", requestId);
    }
}

/**
 * @brief 处理清除WiFi配置
 */
void handleWsWifiClearConfig(AsyncWebSocketClient *client, const JsonDocument &req)
{
    uint32_t requestId = req["requestId"] | 0;
    if (clearWifiConfig())
    {
        sendWsResponse(client, "wifi/clearConfig", true, nullptr, requestId);
    }
    else
    {
        sendWsError(client, "wifi/clearConfig", "清除失败", requestId);
    }
}

/**
 * @brief WiFi扫描任务（在单独任务中执行）
 */
void wifiScanTask(void *pvParameters)
{
    // 开始扫描
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING)
    {
        WiFi.scanDelete(); // 取消之前的扫描
    }
    WiFi.scanNetworks(true); // 异步扫描

    // 等待扫描完成
    int timeout = 60; // 30秒 (每次检查500ms)
    while (timeout-- > 0)
    {
        n = WiFi.scanComplete();
        if (n >= 0)
            break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    JsonDocument doc;
    bool success = true;
    String errorMessage = "";

    if (n > 0)
    {
        // 找到网络，填充结果
        JsonArray networks = doc.to<JsonArray>();
        for (int i = 0; i < n; i++)
        {
            JsonObject net = networks.add<JsonObject>();
            net["ssid"] = WiFi.SSID(i).c_str();
            net["rssi"] = WiFi.RSSI(i);
            net["encryption"] = WiFi.encryptionType(i);
            net["bssid"] = WiFi.BSSIDstr(i);
            net["channel"] = WiFi.channel(i);
        }
        WiFi.scanDelete();
    }
    else if (n == 0)
    {
        // 未找到网络 - 返回空数组
        doc.to<JsonArray>();
    }
    else
    {
        // 扫描失败
        success = false;
        errorMessage = "扫描超时或失败";
    }

    // 广播结果给所有客户端
    JsonDocument resultDoc;
    resultDoc["action"] = "wifi/scanResult";
    resultDoc["success"] = success;
    if (success)
    {
        resultDoc["data"] = doc;
    }
    else
    {
        JsonDocument errData;
        errData["message"] = errorMessage;
        resultDoc["data"] = errData;
    }

    // 等待客户端连接
    int waitTimeout = 20;
    bool sent = false;
    while (waitTimeout-- > 0)
    {
        if (ws && ws->count() > 0)
        {
            // 有客户端连接，发送结果
            String response;
            serializeJson(resultDoc, response);
            ws->textAll(response);
            sent = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!sent)
    {
        LOG_W("WiFi扫描完成但无WebSocket客户端连接，结果未发送");
    }

    wifiScanRunning = false;
    vTaskDelete(NULL);
}

/**
 * @brief 处理开始WiFi扫描
 */
void handleWsWifiScan(AsyncWebSocketClient *client, const JsonDocument &req)
{
    uint32_t requestId = req["requestId"] | 0;

    if (wifiScanRunning)
    {
        sendWsError(client, "wifi/scan", "扫描已在进行中", requestId);
        return;
    }

    // 标记扫描中
    wifiScanRunning = true;

    // 立即回复已开始
    JsonDocument startData;
    startData["status"] = "started";
    sendWsResponse(client, "wifi/scan", true, &startData, requestId);

    // 创建任务执行扫描
    xTaskCreatePinnedToCore(
        wifiScanTask,
        "wifi_scan",
        4096,
        NULL,
        1,
        NULL,
        tskNO_AFFINITY);
}

/**
 * @brief WiFi测试任务
 */
void wifiTestTask(void *pvParameters)
{
    // 获取参数结构体
    WifiTestParams *params = (WifiTestParams *)pvParameters;
    uint32_t clientId = params->clientId;
    String ssid = params->ssid;
    String password = params->password;
    uint32_t requestId = wifiTestRequestId;

    // 保存原始网络配置
    String originalSSID, originalPassword;
    uint8_t originalmode = 0;

    if (WiFi.softAPgetStationNum() > 0)
    {
        if (WiFi.status() == WL_CONNECTED)
            originalmode = 3; // 混合模式
        else
            originalmode = 2; // 仅AP模式
    }
    else if (WiFi.status() == WL_CONNECTED)
    {
        originalmode = 1; // 仅STA模式
    }
    else
    {
        // 异常情况，发送错误
        JsonDocument errData;
        errData["message"] = "检测当前连接模式异常";
        AsyncWebSocketClient *client = ws->client(clientId);
        if (client && client->status() == WS_CONNECTED)
        {
            sendWsResponse(client, "wifi/testResult", false, &errData, requestId);
        }
        wifiTestRunning = false;
        delete params;
        vTaskDelete(NULL);
        return;
    }

    // 尝试加载配置中的wifi信息
    bool hasSavedConfig = loadWifiConfig(originalSSID, originalPassword);

    // 断开当前连接
    WiFi.disconnect(true);

    LOG_I("开始连接WiFi: SSID=%s", ssid.c_str());
    bool wifisuccess = false;
    String errorMessage = "";

    if (connectToWiFi(ssid, password, 20))
    {
        LOG_I("WiFi连接成功: IP=%s", WiFi.localIP().toString().c_str());
        wifisuccess = true;
    }
    else
    {
        LOG_D("WiFi连接失败: 错误码=%d, %s", WiFi.status(), errorMessage.c_str());
        WiFi.disconnect(true);
        switch (WiFi.status())
        {
        case WL_NO_SSID_AVAIL:
            errorMessage = "错误：找不到该SSID";
            break;
        case WL_CONNECT_FAILED:
            errorMessage = "错误：连接失败（可能是密码错误）";
            break;
        default:
            errorMessage = "错误：连接超时";
            break;
        }
    }

    // 准备结果数据
    JsonDocument resultData;
    resultData["success"] = wifisuccess;
    if (wifisuccess)
    {
        resultData["ip"] = WiFi.localIP().toString();
    }
    else
    {
        resultData["errorMessage"] = errorMessage;
    }

    // 断开测试网络
    WiFi.disconnect(true);

    // 恢复原始连接
    if (originalmode == 1 || originalmode == 3)
    {
        LOG_I("恢复原WiFi连接: SSID=%s", originalSSID.c_str());
        connectToWiFi(originalSSID, originalPassword, 20);
        if (WiFi.status() != WL_CONNECTED)
        {
            LOG_E("恢复WiFi失败，切换至AP模式");
            startAPMode();
        }
    }

    // 改为全局广播（模仿扫描结果的方式）
    JsonDocument resultDoc;
    resultDoc["action"] = "wifi/testResult";
    resultDoc["success"] = true;
    resultDoc["data"] = resultData;

    // 等待客户端连接
    int waitTimeout = 20;
    bool sent = false;
    while (waitTimeout-- > 0)
    {
        if (ws && ws->count() > 0)
        {
            // 有客户端连接，发送结果
            String response;
            serializeJson(resultDoc, response);
            ws->textAll(response);
            sent = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!sent)
    {
        LOG_W("WiFi连接测试完成但无WebSocket客户端连接，结果未发送");
    }

    wifiTestRunning = false;
    delete params;
    vTaskDelete(NULL);
}

/**
 * @brief 处理开始WiFi测试
 */
void handleWsWifiTest(AsyncWebSocketClient *client, const JsonDocument &req)
{
    uint32_t requestId = req["requestId"] | 0;

    if (wifiTestRunning)
    {
        sendWsError(client, "wifi/test", "测试已在进行中", requestId);
        return;
    }

    if (!req["ssid"].is<const char *>() || !req["password"].is<const char *>())
    {
        sendWsError(client, "wifi/test", "缺少ssid或password参数", requestId);
        return;
    }

    String ssid = req["ssid"].as<String>();
    String password = req["password"].as<String>();

    if (ssid.isEmpty())
    {
        sendWsError(client, "wifi/test", "SSID不能为空", requestId);
        return;
    }

    // 创建参数结构体
    WifiTestParams *params = new WifiTestParams();
    params->clientId = client->id();
    params->ssid = ssid;
    params->password = password;

    wifiTestRunning = true;
    wifiTestClientId = client->id();
    wifiTestRequestId = requestId;

    // 立即回复已开始
    JsonDocument startData;
    startData["status"] = "started";
    sendWsResponse(client, "wifi/test", true, &startData, requestId);

    // 创建任务执行测试
    xTaskCreatePinnedToCore(
        wifiTestTask,
        "wifi_test",
        8192, // 测试需要更多栈
        (void *)params,
        1,
        NULL,
        tskNO_AFFINITY);
}

/**
 * @brief 处理查询WiFi测试状态（可选，保留以备客户端需要轮询）
 */
void handleWsWifiTestStatus(AsyncWebSocketClient *client, const JsonDocument &req)
{
    uint32_t requestId = req["requestId"] | 0;
    JsonDocument data;
    data["running"] = wifiTestRunning;
    data["success"] = wifiTestSuccess;
    data["ip"] = wifiTestResultIP;
    data["errorMessage"] = wifiTestErrorMessage;
    sendWsResponse(client, "wifi/testStatus", true, &data, requestId);
}

// =============================================================================
// WebSocket 主事件处理
// =============================================================================

void initWebSocket()
{
    if (ws != nullptr)
        return;

    ws = new AsyncWebSocket("/ws");

    ws->onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                   AwsEventType type, void *arg, uint8_t *data, size_t len)
                {
        switch (type) {
            case WS_EVT_CONNECT:
            {
                LOG_D("WebSocket客户端连接: ID=%u", client->id());
                // 发送缓存的日志给新连接的客户端（批量发送）
                JsonDocument doc;
                JsonArray logsArray = doc.to<JsonArray>();
                
                for (const auto &entry : logCache) {
                    JsonObject logObj = logsArray.add<JsonObject>();
                    logObj["level"] = entry.level;
                    logObj["tag"] = entry.tag;
                    logObj["message"] = entry.message;
                    logObj["timestamp"] = entry.timestamp;
                }
                
                String response;
                serializeJson(doc, response);
                client->text(response);
                break;
            }
            case WS_EVT_DISCONNECT:
            {
                LOG_D("WebSocket客户端断开: ID=%u", client->id());
                break;
            }
            case WS_EVT_DATA:
            {
                AwsFrameInfo *info = (AwsFrameInfo*)arg;
                if (info->opcode == WS_TEXT) {
                    String msg = String((char*)data, len);

                    JsonDocument req;
                    DeserializationError error = deserializeJson(req, msg);
                    if (error) {
                        LOG_E("JSON解析错误: %s", error.c_str());
                        return;
                    }

                    if (!req["action"].is<const char*>()) {
                        LOG_W("WebSocket消息缺少action字段");
                        return;
                    }

                    String action = req["action"].as<String>();
                    // 分发到对应处理函数
                    if (action == "wifi/getInfo") {
                        handleWsWifiGetInfo(client, req);
                    } else if (action == "wifi/saveConfig") {
                        handleWsWifiSaveConfig(client, req);
                    } else if (action == "wifi/clearConfig") {
                        handleWsWifiClearConfig(client, req);
                    } else if (action == "wifi/scan") {
                        handleWsWifiScan(client, req);
                    } else if (action == "wifi/test") {
                        handleWsWifiTest(client, req);
                    } else if (action == "wifi/testStatus") {
                        handleWsWifiTestStatus(client, req);
                    } else {
                        LOG_W("未知的WebSocket action: %s", action.c_str());
                    }
                }
                break;
            }
            case WS_EVT_ERROR:
            {
                LOG_E("WebSocket错误: 客户端ID=%u, 错误码=%u", client->id(), type);
                break;
            }
            case WS_EVT_PONG:
                break;
        } });

    LOG_I("WebSocket服务器初始化完成: 路径=/ws");
}

/**
 * @brief 清理 WebSocket 服务器
 */
void cleanupWebSocket()
{
    if (ws != nullptr)
    {
        ws->closeAll();
        delete ws;
        ws = nullptr;
        logCache.clear();
        LOG_I("WebSocket服务器已清理，断开所有客户端");
    }
}

// =============================================================================
// 其余 API 处理器实现（保持不变，仅保留非WiFi的HTTP路由）
// =============================================================================

/**
 * @brief 处理GET /api/battery，返回电池电压和状态。
 */
void handleGetBatteryInfo(AsyncWebServerRequest *request)
{
    LOG_D("HTTP GET /api/battery");
    float voltage = read_battery_voltage();
    JsonDocument doc;
    doc["voltage"] = round(voltage * 100) / 100;
    doc["status"] = voltage > 3.4 ? "normal" : (voltage > 3.2 ? "low" : "critical");
    sendSuccessResponse(request, doc);
}

/**
 * @brief 处理GET /api/system/info，返回系统信息（芯片型号、ID、固件版本）。
 */
void handleGetSystemInfo(AsyncWebServerRequest *request)
{
    LOG_D("HTTP GET /api/system/info");
    JsonDocument doc;
    doc["chipModel"] = ESP.getChipModel();
    uint64_t chipId = ESP.getEfuseMac();
    char chipIdStr[17];
    snprintf(chipIdStr, sizeof(chipIdStr), "%016llX", chipId);
    doc["chipId"] = chipIdStr;
    doc["version"] = serverConfig.firmwareVersion;
    sendSuccessResponse(request, doc);
}

/**
 * @brief 处理GET /api/filesystem，返回文件系统使用情况。
 */
void handleGetFileSystemInfo(AsyncWebServerRequest *request)
{
    LOG_D("HTTP GET /api/filesystem");
    uint64_t totalBytes = LittleFS.totalBytes();
    uint64_t usedBytes = LittleFS.usedBytes();
    uint64_t freeBytes = totalBytes - usedBytes;

    uint32_t totalKB = totalBytes / 1024;
    uint32_t usedKB = usedBytes / 1024;
    uint32_t freeKB = freeBytes / 1024;
    uint32_t freePercent = totalBytes > 0 ? (freeBytes * 100) / totalBytes : 0;

    JsonDocument doc;
    doc["total"] = totalKB;
    doc["used"] = usedKB;
    doc["free"] = freeKB;
    doc["freePercent"] = freePercent;
    sendSuccessResponse(request, doc);
}

/**
 * @brief 处理GET /api/files，列出指定目录下的文件。
 * @param request 包含可选的"path"参数，默认为"/"
 */
void handleListFiles(AsyncWebServerRequest *request)
{
    String path;
    if (!getPathParam(request, "path", false, "/", path))
    {
        sendErrorResponse(request, 400, "无效路径");
        return;
    }
    LOG_D("HTTP GET /api/files?path=%s", path.c_str());

    File dir = LittleFS.open(path);
    if (!dir || !dir.isDirectory())
    {
        sendErrorResponse(request, 404, "目录未找到");
        return;
    }

    JsonDocument doc;
    JsonArray array = doc.to<JsonArray>();
    File entry = dir.openNextFile();
    while (entry)
    {
        JsonObject obj = array.add<JsonObject>();
        obj["name"] = String(entry.name());
        obj["isDirectory"] = entry.isDirectory();
        if (!entry.isDirectory())
            obj["size"] = entry.size();
        entry.close();
        entry = dir.openNextFile();
    }
    dir.close();
    sendSuccessResponse(request, doc);
}

/**
 * @brief 处理POST /api/directories，创建新目录。
 * @param request 必须包含"path"参数（POST表单）
 */
void handleCreateDirectory(AsyncWebServerRequest *request)
{
    String path;
    if (!getPathParam(request, "path", true, "/", path))
    {
        sendErrorResponse(request, 400, "无效路径");
        return;
    }
    LOG_D("HTTP POST /api/directories?path=%s", path.c_str());

    if (LittleFS.mkdir(path))
    {
        LOG_D("目录创建成功");
        sendSuccessResponse(request);
    }
    else
    {
        LOG_E("目录创建失败: %s", path.c_str());
        sendErrorResponse(request, 500, "目录创建失败");
    }
}

/**
 * @brief 处理DELETE /api/files，删除文件或空目录。
 * @param request 必须包含"path"参数（URL参数）
 */
void handleDeleteResource(AsyncWebServerRequest *request)
{
    String path;
    if (!getPathParam(request, "path", false, "/", path))
    {
        sendErrorResponse(request, 400, "无效路径");
        return;
    }
    LOG_D("HTTP DELETE /api/files?path=%s", path.c_str());

    File f = LittleFS.open(path);
    if (!f)
    {
        LOG_E("文件删除失败: 路径不存在: %s", path.c_str());
        sendErrorResponse(request, 404, "未找到");
        return;
    }

    bool isDir = f.isDirectory();
    f.close();

    if (isDir)
    {
        File dir = LittleFS.open(path);
        File entry = dir.openNextFile();
        if (entry)
        {
            entry.close();
            dir.close();
            LOG_E("目录删除失败: 目录非空: %s", path.c_str());
            sendErrorResponse(request, 409, "目录非空");
            return;
        }
        dir.close();
    }

    bool success = isDir ? LittleFS.rmdir(path) : LittleFS.remove(path);
    if (success)
    {
        LOG_D("删除成功");
        sendSuccessResponse(request);
    }
    else
    {
        LOG_E("删除操作失败: %s", path.c_str());
        sendErrorResponse(request, 500, "删除失败");
    }
}

/**
 * @brief 处理GET /api/files/download，下载文件。
 * @param request 必须包含"path"参数（URL参数）
 */
void handleDownloadFile(AsyncWebServerRequest *request)
{
    String path;
    if (!getPathParam(request, "path", false, "/", path))
    {
        LOG_D("HTTP GET /api/files/download 无效路径");
        sendErrorResponse(request, 400, "无效路径");
        return;
    }
    LOG_D("GET /api/files/download 请求，路径: %s", path.c_str());

    File file = LittleFS.open(path, "r");
    if (!file || file.isDirectory())
    {
        if (file)
            file.close();
        if (!file)
        {
            LOG_E("文件下载失败: 文件不存在: %s", path.c_str());
        }
        else
        {
            LOG_E("文件下载失败: 路径不是文件: %s", path.c_str());
        }
        sendErrorResponse(request, !file ? 404 : 400,
                          !file ? "文件未找到" : "路径不是文件");
        return;
    }

    String filename = path.substring(path.lastIndexOf('/') + 1);
    AsyncFileResponse *response = new AsyncFileResponse(file, path, "application/octet-stream");
    String disposition = "attachment; filename=\"" + filename + "\"";
    response->addHeader("Content-Disposition", disposition.c_str());
    request->send(response);
}

/**
 * @brief 处理POST /api/files 的文件上传回调。
 * @param request  HTTP请求
 * @param filename 上传的文件名
 * @param index    数据块索引（0表示开始）
 * @param data     数据指针
 * @param len      数据长度
 * @param final    是否为最后一块
 */
void handleUploadFile(AsyncWebServerRequest *request, String filename, size_t index,
                      uint8_t *data, size_t len, bool final)
{
    // 无需互斥锁，因为回调在同一任务中串行执行
    if (!index)
    {
        uploadState.reset(); // 确保清理之前的状态
        uploadState.active = true;

        String reqPath = request->hasParam("path") ? request->getParam("path")->value() : "/";
        uploadState.path = normalizePath(reqPath);
        if (!uploadState.path.endsWith("/"))
            uploadState.path += "/";

        String fullPath = normalizePath(uploadState.path + filename);
        LOG_I("文件上传开始: %s (Content-Length: %u bytes)", fullPath.c_str(), request->contentLength());

        uint64_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
        if (request->contentLength() > freeBytes)
        {
        LOG_W("文件上传被拒绝: 存储空间不足 (需要: %u, 可用: %u bytes)", request->contentLength(), freeBytes);
            uploadState.error = true;
            uploadState.reset();
            return;
        }

        uploadState.file = LittleFS.open(fullPath, "w");
        if (!uploadState.file)
        {
            LOG_E("文件上传失败: 无法创建文件: %s", fullPath.c_str());
            uploadState.error = true;
            uploadState.reset();
            return;
        }
    }

    if (uploadState.file && data && len > 0)
    {
        size_t written = uploadState.file.write(data, len);
        if (written != len)
        {
            LOG_E("文件上传失败: 写入错误，已删除临时文件");
            uploadState.file.close();
            String fullPath = normalizePath(uploadState.path + filename);
            LittleFS.remove(fullPath);
            uploadState.error = true;
            uploadState.reset();
            return;
        }
        uploadsize += len;
    }

    if (final)
    {
        if (uploadState.file)
            uploadState.file.close();
        // 上传完成，状态将在 handleUploadFileComplete 中清理
    }
}

/**
 * @brief 处理文件上传完成后的回调，返回上传结果。
 */
void handleUploadFileComplete(AsyncWebServerRequest *request)
{
    // 无需互斥锁
    if (uploadState.error)
    {
        sendErrorResponse(request, 500, "上传失败");
    }
    else if (uploadState.active)
    {
        LOG_I("文件上传完成: %u bytes", uploadsize);

        sendSuccessResponse(request);
    }
    else
    {
        sendErrorResponse(request, 400, "未进行上传");
    }
    uploadsize = 0;
    uploadState.reset();
}

/**
 * @brief 处理GET /api/servo/config，获取舵机解锁/锁定位置配置。
 */
void handleGetServoConfig(AsyncWebServerRequest *request)
{
    LOG_D("HTTP GET /api/servo/config");
    uint16_t unlock, lock;
    getServoConfig(unlock, lock);
    JsonDocument doc;
    doc["unlock"] = unlock;
    doc["lock"] = lock;
    sendSuccessResponse(request, doc);
}

/**
 * @brief 处理POST /api/servo/actions/unlock，执行解锁动作。
 */
void handleServoUnlock(AsyncWebServerRequest *request)
{
    LOG_D("POST /api/servo/actions/unlock 请求");
    if (isservobusy)
    {
        LOG_W("舵机解锁失败: 舵机忙碌");
        sendErrorResponse(request, 409, "舵机忙碌");
        return;
    }
    executeUnlock();
    sendSuccessResponse(request);
}

/**
 * @brief 处理POST /api/servo/actions/lock，执行锁定动作。
 */
void handleServoLock(AsyncWebServerRequest *request)
{
    LOG_D("POST /api/servo/actions/lock 请求");
    if (isservobusy)
    {
        LOG_W("舵机锁定失败: 舵机忙碌");
        sendErrorResponse(request, 409, "舵机忙碌");
        return;
    }
    executeLock();
    sendSuccessResponse(request);
}

/**
 * @brief 处理POST /api/servo/actions/position，执行任意位置动作。
 */
static void handleServoPosition(AsyncWebServerRequest *request, uint8_t *data,
                                size_t len, size_t index, size_t total)
{
    LOG_D("POST /api/servo/actions/position 请求");

    if (isservobusy)
    {
        LOG_W("舵机位置控制失败: 舵机忙碌");
        sendErrorResponse(request, 409, "舵机忙碌");
        return;
    }

    handleJsonBody(request, data, len, index, total, [](AsyncWebServerRequest *req, JsonDocument &doc)
                   {
        // 检查字段是否存在
        if (doc["position"].isNull())
        {
            sendErrorResponse(req, 400, "缺少position参数");
            return;
        }

        // 获取字符串并尝试转换为数字
        String posStr = doc["position"].as<String>();
        if (posStr.length() == 0 || posStr == "null")
        {
            sendErrorResponse(req, 400, "缺少position参数");
            return;
        }

        uint16_t position = doc["position"].as<uint16_t>();
        if (!validateServoPosition(position))
        {
            sendErrorResponse(req, 400, "位置值无效（必须<= 1280）");
            return;
        }

        executePosition(position);
        sendSuccessResponse(req); });
}

/**
 * @brief 处理PUT /api/servo/config，更新舵机配置。
 */
static void handlePutServoConfig(AsyncWebServerRequest *request, uint8_t *data,
                                 size_t len, size_t index, size_t total)
{
    if (index == 0)
        LOG_D("HTTP PUT /api/servo/config 请求体大小: %d bytes", total);

    handleJsonBody(request, data, len, index, total, [](AsyncWebServerRequest *req, JsonDocument &doc)
                   {
        if (!doc["unlock"].is<uint16_t>() || !doc["lock"].is<uint16_t>())
        {
            sendErrorResponse(req, 400, "缺少unlock或lock参数");
            return;
        }

        uint16_t unlock = doc["unlock"];
        uint16_t lock = doc["lock"];

        if (!validateServoPosition(unlock) || !validateServoPosition(lock))
        {
            sendErrorResponse(req, 400, "舵机位置无效（必须<= 1280）");
            return;
        }

        saveServoConfig(unlock, lock);
        LOG_I("舵机配置已更新: 解锁=%d, 锁定=%d", unlock, lock);
        sendSuccessResponse(req); });
}

/**
 * @brief 处理POST /api/system/actions/restart，重启系统。
 */
void handleRestartSystem(AsyncWebServerRequest *request)
{
    LOG_I("系统重启请求");
    sendSuccessResponse(request);
    delay(500);
    ESP.restart();
}

/**
 * @brief 处理POST /api/system/firmware 的OTA更新回调。
 * @param request  HTTP请求
 * @param filename 固件文件名
 * @param index    数据块索引
 * @param data     数据指针
 * @param len      数据长度
 * @param final    是否为最后一块
 */
void handleOtaUpdate(AsyncWebServerRequest *request, String filename, size_t index,
                     uint8_t *data, size_t len, bool final)
{
    if (!index)
    {
        LOG_I("OTA固件更新开始: %s (大小: %u bytes)", filename.c_str(), request->contentLength());
        if (!validateOtaFileSize(request->contentLength()))
        {
            LOG_E("OTA更新失败: 固件大小超限 (最大2MB)");
            Update.abort();
            return;
        }
        if (!Update.begin(request->contentLength()))
        {
            Update.printError(Serial);
            Update.abort();
            return;
        }
        Update.onProgress([](size_t progress, size_t total)
                          { LOG_D("OTA更新进度: %u%%", (progress * 100) / total); });
    }

    if (Update.write(data, len) != len)
    {
        Update.printError(Serial);
        Update.abort();
        return;
    }

    if (final)
    {
        if (Update.end(true))
            LOG_I("OTA固件更新成功，系统即将重启");
        else
        {
            Update.printError(Serial);
            Update.abort();
        }
    }
}

/**
 * @brief 处理OTA更新完成后的回调，返回更新结果。
 */
void handleOtaUpdateComplete(AsyncWebServerRequest *request)
{
    if (Update.hasError())
    {
        sendErrorResponse(request, 500, "OTA更新失败");
    }
    else
    {
        sendSuccessResponse(request);
        delay(1000);
        ESP.restart();
    }
}

// =============================================================================
// 卡片管理 API 处理器实现（保持不变）
// =============================================================================

/**
 * @brief 处理GET /api/cards，获取所有授权卡片列表
 */
void handleGetCardsList(AsyncWebServerRequest *request)
{
    LOG_D("HTTP GET /api/cards");

    JsonDocument doc;
    JsonArray cardsArray = doc["cards"].to<JsonArray>();

    for (const auto &card : authorizedCards)
    {
        JsonObject cardObj = cardsArray.add<JsonObject>();
        cardObj["length"] = card.uidLength;
        cardObj["name"] = card.name;

        char uidStr[9];
        snprintf(uidStr, sizeof(uidStr), "%02X%02X%02X%02X", card.uid[0], card.uid[1], card.uid[2], card.uid[3]);
        cardObj["uid"] = uidStr;
    }

    sendSuccessResponse(request, doc);
    LOG_D("卡片列表查询完成: 共%d张", authorizedCards.size());
}

/**
 * @brief 处理POST /api/cards，添加新卡片
 */
void handleAddCard(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
    if (index == 0)
        LOG_D("HTTP POST /api/cards 请求体大小: %d bytes", total);

    handleJsonBody(request, data, len, index, total, [](AsyncWebServerRequest *req, JsonDocument &doc)
                   {
        if (!doc["uid"].is<const char*>() || !doc["name"].is<const char*>())
        {
            sendErrorResponse(req, 400, "缺少uid或name参数");
            return;
        }

        String uid = doc["uid"].as<String>();
        String name = doc["name"].as<String>();

        // 验证UID格式（8位十六进制）
        if (uid.length() != 8)
        {
            sendErrorResponse(req, 400, "UID格式错误，应为8位十六进制");
            return;
        }
        // 额外检查是否为有效十六进制
        for (char c : uid) {
            if (!isxdigit(c)) {
                sendErrorResponse(req, 400, "UID包含非法字符");
                return;
            }
        }
        // 限制名称长度
        if (name.length() > 32) {
            sendErrorResponse(req, 400, "名称长度不能超过32字符");
            return;
        }

        // 检查UID是否已存在
        for (const auto &card : authorizedCards)
        {
            char existingUid[9];
            snprintf(existingUid, sizeof(existingUid), "%02X%02X%02X%02X", card.uid[0], card.uid[1], card.uid[2], card.uid[3]);
            if (uid.equalsIgnoreCase(existingUid))
            {
                sendErrorResponse(req, 409, "该UID已存在");
                return;
            }
        }

        // 创建新卡片
        NFCcard newCard;
        newCard.uidLength = 4;
        newCard.name = name;

        // 将十六进制字符串转换为字节数组
        for (int i = 0; i < 4; i++)
        {
            String byteStr = uid.substring(i * 2, i * 2 + 2);
            newCard.uid[i] = (uint8_t)strtol(byteStr.c_str(), NULL, 16);
        }

        authorizedCards.push_back(newCard);

        if (saveCardsToFile())
        {
            LOG_I("授权卡片添加成功: UID=%s, 名称=%s", uid.c_str(), name.c_str());
            sendSuccessResponse(req);
        }
        else
        {
            authorizedCards.pop_back();
            sendErrorResponse(req, 500, "保存失败");
        } });
}

/**
 * @brief 处理DELETE /api/cards，删除指定卡片
 */
void handleDeleteCard(AsyncWebServerRequest *request)
{
    if (!request->hasParam("uid"))
    {
        sendErrorResponse(request, 400, "缺少uid参数");
        return;
    }

    String uid = request->getParam("uid")->value();
    LOG_D("HTTP DELETE /api/cards?uid=%s", uid.c_str());

    bool found = false;
    auto it = authorizedCards.begin();
    while (it != authorizedCards.end())
    {
        char existingUid[9];
        snprintf(existingUid, sizeof(existingUid), "%02X%02X%02X%02X", it->uid[0], it->uid[1], it->uid[2], it->uid[3]);

        if (uid.equalsIgnoreCase(existingUid))
        {
            authorizedCards.erase(it);
            found = true;
            break;
        }
        it++;
    }

    if (!found)
    {
        sendErrorResponse(request, 404, "未找到该卡片");
        return;
    }

    if (saveCardsToFile())
    {
        LOG_I("授权卡片删除成功: UID=%s", uid.c_str());
        sendSuccessResponse(request);
    }
    else
    {
        sendErrorResponse(request, 500, "保存失败");
    }
}

/**
 * @brief 处理GET /api/cards/read，从读卡器读取卡片UID
 */
void handleReadCard(AsyncWebServerRequest *request)
{
    LOG_D("HTTP GET /api/cards/read");

    sendCardSearchCommand();

    unsigned long startTime = millis();
    const unsigned long timeout = 2000; // 2秒超时

    NFCcard readResult;
    readResult.uidLength = 0;

    // 使用无符号减法，即使 millis() 溢出也能正确计算时间差（前提是 timeout < 2^32）
    while ((millis() - startTime) < timeout)
    {
        esp_task_wdt_reset();

        if (Serial1.available() >= 14)
        {
            readResult = ReadCard();
            if (readResult.uidLength != 0)
            {
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (readResult.uidLength == 0)
    {
        sendErrorResponse(request, 404, "未读取到卡片，请将卡片放在读卡器上");
        return;
    }

    JsonDocument doc;
    char uidStr[9];
    snprintf(uidStr, sizeof(uidStr), "%02X%02X%02X%02X", readResult.uid[0], readResult.uid[1], readResult.uid[2], readResult.uid[3]);
    doc["uid"] = uidStr;
    doc["length"] = readResult.uidLength;

    sendSuccessResponse(request, doc);
    LOG_I("NFC卡片读取成功: UID=%s", uidStr);
}

/**
 * @brief 处理GET /api/cards/test，测试卡片是否授权
 */
void handleTestCard(AsyncWebServerRequest *request)
{
    if (!request->hasParam("uid"))
    {
        sendErrorResponse(request, 400, "缺少uid参数");
        return;
    }

    String uid = request->getParam("uid")->value();
    LOG_D("HTTP GET /api/cards/test?uid=%s", uid.c_str());

    // 验证UID格式
    if (uid.length() != 8)
    {
        sendErrorResponse(request, 400, "UID格式错误");
        return;
    }
    for (char c : uid)
    {
        if (!isxdigit(c))
        {
            sendErrorResponse(request, 400, "UID包含非法字符");
            return;
        }
    }

    // 创建测试卡片
    NFCcard testCard;
    testCard.uidLength = 4;
    for (int i = 0; i < 4; i++)
    {
        String byteStr = uid.substring(i * 2, i * 2 + 2);
        testCard.uid[i] = (uint8_t)strtol(byteStr.c_str(), NULL, 16);
    }

    bool authorized = false;
    for (const auto &card : authorizedCards)
    {
        if (testCard.uidLength == card.uidLength)
        {
            bool match = true;
            for (int j = 0; j < testCard.uidLength; j++)
            {
                if (testCard.uid[j] != card.uid[j])
                {
                    match = false;
                    break;
                }
            }
            if (match)
            {
                authorized = true;
                break;
            }
        }
    }

    JsonDocument doc;
    doc["authorized"] = authorized;
    doc["uid"] = uid;

    if (authorized)
    {
        LOG_D("卡片验证通过: UID=%s", uid.c_str());
        doc["message"] = "该卡片已授权";
    }
    else
    {
        LOG_D("卡片验证失败: UID=%s 未授权", uid.c_str());
        doc["message"] = "该卡片未授权";
    }

    sendSuccessResponse(request, doc);
}

/**
 * @brief 处理GET /api/cards/logic-enabled，获取卡片逻辑启用状态
 */
void handleGetCardLogicConfig(AsyncWebServerRequest *request)
{
    LOG_D("HTTP GET /api/cards/logic-enabled");
    JsonDocument doc;
    doc["enabled"] = cardLogicEnabled;
    sendSuccessResponse(request, doc);
}

/**
 * @brief 处理POST /api/cards/logic-enabled，设置卡片逻辑启用状态
 */
void handleSetCardLogicConfig(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
    if (index == 0)
        LOG_D("HTTP POST /api/cards/logic-enabled 请求体大小: %d bytes", total);

    handleJsonBody(request, data, len, index, total, [](AsyncWebServerRequest *req, JsonDocument &doc)
                   {
        if (!doc["enabled"].is<bool>())
        {
            sendErrorResponse(req, 400, "缺少enabled参数");
            return;
        }
        cardLogicEnabled = doc["enabled"];
        LOG_I("卡片逻辑状态已更新: %s", cardLogicEnabled ? "启用" : "禁用");
        sendSuccessResponse(req); });
}

// =============================================================================
// 路由注册函数（移除了所有WiFi HTTP路由）
// =============================================================================

/**
 * @brief 智能发送文件：若客户端支持 gzip 且存在 .gz 文件，则发送压缩版并添加 Content-Encoding: gzip 和缓存头
 * @param request    HTTP 请求对象
 * @param path       原始文件路径（如 "/web/index.html"）
 * @param contentType MIME 类型（如 "text/html; charset=utf-8"）
 */
void servePrecompiledFile(AsyncWebServerRequest *request, const String &path, const String &contentType)
{
    String gzPath = path + ".gz";

    bool acceptGzip = false;
    if (request->hasHeader("Accept-Encoding"))
    {
        String accept = request->getHeader("Accept-Encoding")->value();
        if (accept.indexOf("gzip") >= 0)
        {
            acceptGzip = true;
        }
    }

    if (acceptGzip && LittleFS.exists(gzPath))
    {
        AsyncWebServerResponse *response = request->beginResponse(LittleFS, gzPath, contentType);
        response->addHeader("Content-Encoding", "gzip");
        response->addHeader("Cache-Control", "public, max-age=1800");
        request->send(response);
        return;
    }

    if (LittleFS.exists(path))
    {
        AsyncWebServerResponse *response = request->beginResponse(LittleFS, path, contentType);
        response->addHeader("Cache-Control", "public, max-age=1800");
        request->send(response);
    }
    else
    {
        request->send(404, "text/plain", "File not found");
    }
}

/**
 * @brief 注册静态资源路由（CSS、JS），支持预压缩 gzip。
 * @param server AsyncWebServer指针
 */
void registerStaticRoutes(AsyncWebServer *server)
{
    server->on("/web/css/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/css/style.css", "text/css"); });
    server->on("/web/css/pages.css", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/css/pages.css", "text/css"); });
    server->on("/web/js/common.js", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/js/common.js", "application/javascript"); });
    server->on("/web/js/pages/index.js", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/js/pages/index.js", "application/javascript"); });
    server->on("/web/js/pages/wifi.js", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/js/pages/wifi.js", "application/javascript"); });
    server->on("/web/js/pages/files.js", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/js/pages/files.js", "application/javascript"); });
    server->on("/web/js/pages/servo.js", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/js/pages/servo.js", "application/javascript"); });
    server->on("/web/js/pages/ota.js", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/js/pages/ota.js", "application/javascript"); });
    server->on("/web/js/pages/cards.js", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/js/pages/cards.js", "application/javascript"); });
}

/**
 * @brief 注册页面路由（HTML页面），支持预压缩 gzip。
 * @param server AsyncWebServer指针
 */
void registerPageRoutes(AsyncWebServer *server)
{
    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/index.html", "text/html; charset=utf-8"); });
    server->on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/wifi.html", "text/html; charset=utf-8"); });
    server->on("/ota", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/ota.html", "text/html; charset=utf-8"); });
    server->on("/files", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/files.html", "text/html; charset=utf-8"); });
    server->on("/cards", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/cards.html", "text/html; charset=utf-8"); });
    server->on("/servo", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/servo.html", "text/html; charset=utf-8"); });
}

/**
 * @brief 注册API路由（移除了所有WiFi相关的路由）。
 * @param server AsyncWebServer指针
 */
void registerApiRoutes(AsyncWebServer *server)
{
    // 注意：WiFi管理已迁移至WebSocket，此处不再注册相关路由
    server->on("/api/battery", HTTP_GET, handleGetBatteryInfo);
    server->on("/api/system/info", HTTP_GET, handleGetSystemInfo);
    server->on("/api/system/actions/restart", HTTP_POST, handleRestartSystem);
    server->on("/api/system/firmware", HTTP_POST, handleOtaUpdateComplete, handleOtaUpdate);
    server->on("/api/filesystem", HTTP_GET, handleGetFileSystemInfo);
    server->on("/api/files/download", HTTP_GET, handleDownloadFile);
    server->on("/api/files", HTTP_GET, handleListFiles);
    server->on("/api/files", HTTP_DELETE, handleDeleteResource);
    server->on("/api/files", HTTP_POST, handleUploadFileComplete, handleUploadFile);
    server->on("/api/directories", HTTP_POST, handleCreateDirectory);
    server->on("/api/servo/config", HTTP_GET, handleGetServoConfig);
    server->on("/api/servo/config", HTTP_PUT, [](AsyncWebServerRequest *request) {}, NULL, handlePutServoConfig);
    server->on("/api/servo/actions/unlock", HTTP_POST, handleServoUnlock);
    server->on("/api/servo/actions/lock", HTTP_POST, handleServoLock);
    server->on("/api/servo/actions/position", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, handleServoPosition);
    server->on("/api/cards/read", HTTP_GET, handleReadCard);
    server->on("/api/cards/test", HTTP_GET, handleTestCard);
    server->on("/api/cards/logic-enabled", HTTP_GET, handleGetCardLogicConfig);
    server->on("/api/cards/logic-enabled", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, handleSetCardLogicConfig);
    server->on("/api/cards", HTTP_GET, handleGetCardsList);
    server->on("/api/cards", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, handleAddCard);
    server->on("/api/cards", HTTP_DELETE, handleDeleteCard);
}

// =============================================================================
// 主要函数
// =============================================================================

/**
 * @brief 初始化Web服务器，连接WiFi并启动HTTP服务。
 */
void initWebServer()
{
    if (server != nullptr)
        return;

    LOG_I("初始化Web服务器");

    addTolist(8);

    String savedSSID, savedPassword;
    bool hasSavedConfig = loadWifiConfig(savedSSID, savedPassword);

    if (!hasSavedConfig)
    {
        LOG_I("未检测到已保存的WiFi配置，将进入AP模式");
    }
    else
    {
        LOG_I("连接WiFi: %s", savedSSID.c_str());
    }
    bool wifisuccess = false;

    WiFi.mode(WIFI_MODE_STA);

    // 使用统一的 connectToWiFi 函数
    if (connectToWiFi(savedSSID, savedPassword, 20))
    {
        addTolist(9);

        LOG_I("WiFi连接成功，IP: %s", WiFi.localIP().toString().c_str());
        wifisuccess = true;
    }
    else
    {
        addTolist(10);

        WiFi.disconnect(true);
        LOG_W("WiFi连接失败，切换到AP模式");
    }

    if (!wifisuccess)
    {
        startAPMode();
    }

    server = new AsyncWebServer(serverConfig.serverPort);
    webserverMode = WS_MODE_RUNNING;

    initWebSocket();

    registerStaticRoutes(server);
    registerPageRoutes(server);
    registerApiRoutes(server);

    server->addHandler(ws);
    server->begin();

    if (!wifisuccess)
    {
        LOG_I("Web服务器启动（AP模式），访问 http://%s", WiFi.softAPIP().toString().c_str());
    }
    else
    {
        LOG_I("Web服务器启动，访问 http://%s", WiFi.localIP().toString().c_str());
    }
}

/**
 * @brief 检查Web服务器是否正在运行。
 * @return true 运行中，false 未运行
 */
bool isWebServerRunning()
{
    return (webserverMode == WS_MODE_RUNNING) && (server != nullptr);
}

/**
 * @brief 停止Web服务器。
 */
void stopWebServer()
{
    if (server != nullptr)
    {
        LOG_I("停止Web服务器");
        server->end();
        delete server;
        server = nullptr;
        webserverMode = WS_MODE_OFF;
        LOG_I("Web服务器已停止");
    }

    cleanupWebSocket();
}

// =============================================================================
// WebSocket 日志系统（保持不变）
// =============================================================================

/**
 * @brief 广播日志到所有 WebSocket 客户端
 * @param level 日志级别
 * @param tag 日志标签
 * @param message 日志消息
 */
void broadcastLogToWebSocket(int level, const char *tag, const char *message)
{
    if (ws == nullptr || !isWebServerRunning())
        return;

    unsigned long timestamp = millis();
    LogEntry entry(level, tag, message, timestamp);

    logCache.push_back(entry);
    if (logCache.size() > LOG_CACHE_SIZE)
    {
        logCache.pop_front();
    }

    // 构建单条日志JSON
    JsonDocument doc;
    doc["level"] = level;
    doc["tag"] = tag;
    doc["message"] = message;
    doc["timestamp"] = timestamp;

    String response;
    serializeJson(doc, response);

    ws->textAll(response);
}
