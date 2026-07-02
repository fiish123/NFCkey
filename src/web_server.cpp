/**
 * @file web_server.cpp
 * @brief Web服务器实现，提供WiFi连接、文件管理、OTA升级和舵机控制API。
 *        注：WiFi管理功能已迁移至WebSocket通信。
 */

#include "web_server.h"

#include <deque>
#include <mbedtls/sha256.h>
#include <vector>

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
std::deque<LogEntry> logCache;
// 内存优化：由 256 降为 128。每条 LogEntry 约 160 字节，减少约 20KB 堆占用；
// 回放逻辑的 fallback(64)/max-batch(64) 在 128 容量下仍可完整工作。
static constexpr size_t LOG_CACHE_SIZE = 128;
static constexpr size_t LOG_REPLAY_FALLBACK_SIZE = 64;
static constexpr size_t LOG_REPLAY_MAX_BATCH_SIZE = 64;
static constexpr size_t LOG_SINGLE_ENTRY_JSON_BUFFER_SIZE = 384;
static constexpr size_t WS_MAX_INCOMING_MESSAGE_SIZE = 512;
static uint32_t logSessionId = 0;
static uint32_t nextLogId = 1;
static portMUX_TYPE logCacheMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t generateLogSessionId()
{
    uint32_t sessionId = esp_random();
    if (sessionId == 0)
    {
        sessionId = 1;
    }

    return sessionId;
}

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
    "1.1.0",
    80};

// 文件上传状态管理（类型来自头文件）
static FileUploadState uploadState;
static constexpr size_t OTA_SHA256_HEX_LENGTH = 64;
static constexpr size_t OTA_SHA256_BYTES = 32;
static const char *OTA_HASH_HEADER = "X-Firmware-SHA256";
static const char *FILE_HASH_HEADER = "X-File-SHA256";
static constexpr uint16_t SERVO_MAX_POSITION = 1280;

struct OtaUploadState
{
    bool active;
    bool finalSeen;
    bool success;
    bool error;
    bool hashProvided;
    bool hashFormatInvalid;
    bool hashMismatch;
    String expectedSha256;

    void reset()
    {
        active = false;
        finalSeen = false;
        success = false;
        error = false;
        hashProvided = false;
        hashFormatInvalid = false;
        hashMismatch = false;
        expectedSha256 = "";
    }
};

static OtaUploadState otaUploadState;
static mbedtls_sha256_context otaSha256Context;
static bool otaSha256ContextActive = false;
static mbedtls_sha256_context fileUploadSha256Context;
static bool fileUploadSha256ContextActive = false;
static String fileUploadTargetPath = "";
static String fileUploadTempPath = "";
static std::vector<uint8_t> jsonBodyBuffer;
static AsyncWebServerRequest *jsonBodyRequest = nullptr;

// WiFi测试状态管理（仍用于异步任务协调）
static bool wifiTestRunning = false;
static uint32_t wifiTestRequestId = 0; // 请求ID

// WiFi扫描状态管理
static bool wifiScanRunning = false;
static bool restartScheduled = false;
static uint32_t restartScheduledAtMs = 0;
static uint32_t restartDelayMs = 0;
static const char *restartReason = "unknown";

// WiFi测试参数结构体
struct WifiTestParams
{
    uint32_t clientId;
    String ssid;
    String password;
};

uint32_t uploadsize = 0; // 上传字节计数

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
    return position <= SERVO_MAX_POSITION;
}

/**
 * @brief 验证OTA文件大小是否≤2MB。
 * @param size 文件大小（字节）
 * @return true 有效，false 无效
 */
bool validateOtaFileSize(size_t size)
{
    return size <= (2 * 1024 * 1024);
}

static bool isLowerHexString(const String &value, size_t expectedLength)
{
    if (value.length() != expectedLength)
        return false;

    for (size_t i = 0; i < expectedLength; ++i)
    {
        if (!isxdigit(static_cast<unsigned char>(value[i])))
            return false;
    }

    return true;
}

static bool isReservedUploadPath(const String &path)
{
    return path.endsWith(".uploading") || path.endsWith(".backup");
}

static bool isValidFsPath(const String &path)
{
    if (path.isEmpty() || path[0] != '/')
        return false;

    if (path.indexOf("..") >= 0)
        return false;

    if (isReservedUploadPath(path))
        return false;

    return true;
}

static String getParentDirectory(const String &path)
{
    if (path.length() <= 1)
        return "/";

    int separatorIndex = path.lastIndexOf('/');
    if (separatorIndex <= 0)
        return "/";

    return path.substring(0, separatorIndex);
}

static bool ensureDirectoryExistsRecursive(const String &directoryPath)
{
    String normalized = normalizePath(directoryPath);
    if (normalized.isEmpty() || !isValidFsPath(normalized))
        return false;

    if (normalized == "/")
        return true;

    File existing = LittleFS.open(normalized);
    if (existing)
    {
        const bool isDirectory = existing.isDirectory();
        existing.close();
        return isDirectory;
    }

    const String parent = getParentDirectory(normalized);
    if (!ensureDirectoryExistsRecursive(parent))
        return false;

    if (LittleFS.mkdir(normalized))
        return true;

    File created = LittleFS.open(normalized);
    if (!created)
        return false;

    const bool isDirectory = created.isDirectory();
    created.close();
    return isDirectory;
}

/**
 * @brief 将SHA256摘要（32字节）转换为小写十六进制字符串。
 * @param digest SHA256摘要
 * @param outHex 输出64字符十六进制字符串
 */
static void sha256DigestToHex(const unsigned char digest[OTA_SHA256_BYTES], String &outHex)
{
    char hex[(OTA_SHA256_HEX_LENGTH + 1)] = {0};
    for (size_t i = 0; i < OTA_SHA256_BYTES; ++i)
    {
        snprintf(&hex[i * 2], 3, "%02x", digest[i]);
    }
    outHex = hex;
}

static bool computeLittleFsFileSha256(const String &path, String &outHash)
{
    File file = LittleFS.open(path, "r");
    if (!file || file.isDirectory())
    {
        if (file)
            file.close();
        return false;
    }

    mbedtls_sha256_context context;
    mbedtls_sha256_init(&context);
    if (mbedtls_sha256_starts_ret(&context, 0) != 0)
    {
        mbedtls_sha256_free(&context);
        file.close();
        return false;
    }

    uint8_t buffer[1024];
    while (file.available())
    {
        const size_t readBytes = file.read(buffer, sizeof(buffer));
        if (readBytes == 0)
            break;

        if (mbedtls_sha256_update_ret(&context, buffer, readBytes) != 0)
        {
            mbedtls_sha256_free(&context);
            file.close();
            return false;
        }
    }

    file.close();

    unsigned char digest[OTA_SHA256_BYTES];
    if (mbedtls_sha256_finish_ret(&context, digest) != 0)
    {
        mbedtls_sha256_free(&context);
        return false;
    }

    mbedtls_sha256_free(&context);

    sha256DigestToHex(digest, outHash);
    return true;
}

static bool isProtectedRuntimeFile(const String &path)
{
    return path == "/cards.json";
}

static void resetOtaHashContext()
{
    if (!otaSha256ContextActive)
        return;

    mbedtls_sha256_free(&otaSha256Context);
    otaSha256ContextActive = false;
}

static void resetFileUploadHashContext()
{
    if (!fileUploadSha256ContextActive)
        return;

    mbedtls_sha256_free(&fileUploadSha256Context);
    fileUploadSha256ContextActive = false;
}

static bool beginFileUploadHashContext()
{
    resetFileUploadHashContext();
    mbedtls_sha256_init(&fileUploadSha256Context);

    if (mbedtls_sha256_starts_ret(&fileUploadSha256Context, 0) != 0)
    {
        mbedtls_sha256_free(&fileUploadSha256Context);
        return false;
    }

    fileUploadSha256ContextActive = true;
    return true;
}

static bool updateFileUploadHashContext(const uint8_t *data, size_t len)
{
    if (!fileUploadSha256ContextActive)
        return false;

    if (mbedtls_sha256_update_ret(&fileUploadSha256Context, data, len) != 0)
    {
        resetFileUploadHashContext();
        return false;
    }

    return true;
}

static bool finalizeFileUploadHash(String &outHash)
{
    if (!fileUploadSha256ContextActive)
        return false;

    unsigned char digest[OTA_SHA256_BYTES];
    if (mbedtls_sha256_finish_ret(&fileUploadSha256Context, digest) != 0)
    {
        resetFileUploadHashContext();
        return false;
    }

    sha256DigestToHex(digest, outHash);
    resetFileUploadHashContext();
    return true;
}

static void resetFileUploadState()
{
    resetFileUploadHashContext();
    fileUploadTargetPath = "";
    fileUploadTempPath = "";
    uploadState.reset();
}

static void cleanupFileUploadTempFile()
{
    if (!fileUploadTempPath.isEmpty())
    {
        LittleFS.remove(fileUploadTempPath);
    }
}

static void resetOtaUploadState()
{
    resetOtaHashContext();
    otaUploadState.reset();
}

static bool beginOtaHashContext()
{
    resetOtaHashContext();
    mbedtls_sha256_init(&otaSha256Context);

    if (mbedtls_sha256_starts_ret(&otaSha256Context, 0) != 0)
    {
        mbedtls_sha256_free(&otaSha256Context);
        return false;
    }

    otaSha256ContextActive = true;
    return true;
}

static bool updateOtaHashContext(const uint8_t *data, size_t len)
{
    if (!otaSha256ContextActive)
        return false;

    if (mbedtls_sha256_update_ret(&otaSha256Context, data, len) != 0)
    {
        resetOtaHashContext();
        return false;
    }

    return true;
}

static bool finalizeOtaHash(String &outHash)
{
    if (!otaSha256ContextActive)
        return false;

    unsigned char digest[OTA_SHA256_BYTES];
    if (mbedtls_sha256_finish_ret(&otaSha256Context, digest) != 0)
    {
        resetOtaHashContext();
        return false;
    }

    sha256DigestToHex(digest, outHash);
    resetOtaHashContext();
    return true;
}

static void scheduleSystemRestart(uint32_t delayMs, const char *reason)
{
    if (restartScheduled)
    {
        LOG_W("系统重启已在队列中，忽略重复请求: %s", reason ? reason : "unknown");
        return;
    }

    restartScheduled = true;
    restartScheduledAtMs = millis();
    restartDelayMs = delayMs;
    restartReason = reason ? reason : "unknown";
}

void serviceScheduledRestart()
{
    if (!restartScheduled)
    {
        return;
    }

    if (static_cast<uint32_t>(millis() - restartScheduledAtMs) < restartDelayMs)
    {
        return;
    }

    restartScheduled = false;
    LOG_I("系统准备重启: %s", restartReason);
    ESP.restart();
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
        LOG_W("WiFi配置未保存，配置为空");
        return false;
    }

    LOG_D("WiFi配置读取成功: SSID=%s", ssid.c_str());
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
    LOG_I("开始连接WiFi: SSID=%s", ssid.c_str());
    WiFi.begin(ssid.c_str(), password.c_str());
    for (int i = 0; i < maxAttempts; i++)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            LOG_I("WiFi连接成功: SSID=%s, IP=%s", ssid.c_str(), WiFi.localIP().toString().c_str());
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(delayMs));
        esp_task_wdt_reset();
    }
    LOG_W("WiFi连接失败: SSID=%s, 状态码=%d", ssid.c_str(), WiFi.status());
    return false;
}

// =============================================================================
// API 响应处理（HTTP API）
//
// 内存优化说明：
// - 不再使用 ApiResponse 中间结构（曾内嵌 JsonDocument data 成员，并在
//   toJson() 中再次构建 JsonDocument，导致每次响应产生 2~3 个文档与多次深拷贝）。
// - 现采用直接内联序列化：data 文档仅序列化一次拼入响应字符串，避免跨文档深拷贝
//   （ArduinoJson 7 中 docA["data"]=docB 会整树深拷贝，且 serializeJson(doc,String)
//    会覆盖而非追加字符串，故先序列化 data 到临时字符串再拼接）。
// =============================================================================

/**
 * @brief 将一个 C 字符串以合法 JSON 字符串形式（含两侧引号）追加到 out。
 *        用于在不构造 JsonDocument 的情况下安全地内联字段，保证转义正确。
 */
static void appendJsonString(String &out, const char *value)
{
    out += '"';
    if (value)
    {
        for (const char *p = value; *p; ++p)
        {
            const unsigned char c = static_cast<unsigned char>(*p);
            switch (c)
            {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\b':
                out += "\\b";
                break;
            case '\f':
                out += "\\f";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                if (c < 0x20)
                {
                    char esc[8];
                    snprintf(esc, sizeof(esc), "\\u%04x", c);
                    out += esc;
                }
                else
                {
                    out += static_cast<char>(c);
                }
                break;
            }
        }
    }
    out += '"';
}

static void resetJsonBodyState()
{
    jsonBodyBuffer.clear();
    jsonBodyBuffer.shrink_to_fit(); // 释放上一次请求体缓冲占用的堆内存
    jsonBodyRequest = nullptr;
}

/**
 * @brief 发送成功响应（200 OK），带附加数据。
 *        data 直接序列化拼入响应，不再深拷贝到第二个 JsonDocument。
 */
static void sendSuccessResponse(AsyncWebServerRequest *request, const JsonDocument &data)
{
    String dataStr;
    dataStr.reserve(measureJson(data));
    serializeJson(data, dataStr); // 空文档将输出 "null"，与历史 hasData=false 行为一致

    String response;
    response.reserve(dataStr.length() + 48);
    response += "{\"success\":true,\"message\":\"Success\",\"data\":";
    response += dataStr;
    response += '}';
    request->send(200, "application/json", response);
}

/**
 * @brief 发送成功响应（200 OK），无额外数据。使用字面量，零 JsonDocument 分配。
 */
static void sendSuccessResponse(AsyncWebServerRequest *request)
{
    request->send(200, "application/json", "{\"success\":true,\"message\":\"Success\",\"data\":null}");
}

/**
 * @brief 发送错误响应。message 经 appendJsonString 安全转义后内联，无需 JsonDocument。
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

    String response;
    response.reserve(message.length() + 64);
    response += "{\"success\":false,\"message\":";
    appendJsonString(response, message.c_str());
    response += ",\"data\":null}";
    request->send(code, "application/json", response);
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
    if (index == 0 || jsonBodyRequest != request)
    {
        resetJsonBodyState();
        jsonBodyRequest = request;
        if (total > 0)
            jsonBodyBuffer.reserve(total);
    }

    if (index != jsonBodyBuffer.size())
    {
        LOG_E("JSON请求体分块顺序异常: index=%u buffered=%u total=%u", index, jsonBodyBuffer.size(), total);
        resetJsonBodyState();
        sendErrorResponse(request, 400, "无效的JSON");
        return;
    }

    if (data && len > 0)
    {
        jsonBodyBuffer.insert(jsonBodyBuffer.end(), data, data + len);
    }

    if (index + len < total)
        return;

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonBodyBuffer.data(), jsonBodyBuffer.size());
    if (error)
    {
        LOG_E("JSON解析错误: %s", error.c_str());
        resetJsonBodyState();
        sendErrorResponse(request, 400, "无效的JSON");
        return;
    }

    resetJsonBodyState();
    func(request, doc);
}

// =============================================================================
// WebSocket 响应辅助函数
// =============================================================================

/**
 * @brief 通过WebSocket向指定客户端发送JSON响应（内联序列化）。
 * @param client 客户端指针
 * @param action 动作类型（内部固定字面量，如 "wifi/getInfo"，无需转义）
 * @param success 是否成功
 * @param data 附加数据（可选）
 * @param requestId 请求ID（若请求中提供）
 *
 * 内存优化：data 直接序列化拼入响应字符串，避免 `doc["data"]=*data` 触发的整树深拷贝。
 */
static void sendWsResponse(AsyncWebSocketClient *client, const String &action, bool success,
                           const JsonDocument *data = nullptr, uint32_t requestId = 0)
{
    if (!client)
        return;

    String dataStr;
    size_t dataLen = 0;
    const bool hasData = data && !data->isNull();
    if (hasData)
    {
        dataLen = measureJson(*data);
        dataStr.reserve(dataLen);
        serializeJson(*data, dataStr);
    }

    String response;
    response.reserve(action.length() + dataLen + 64); // +64 覆盖 envelope + 可选 requestId(<=10位)
    response += "{\"action\":\"";
    response += action;
    response += "\",\"success\":";
    response += success ? "true" : "false";
    if (requestId != 0)
    {
        response += ",\"requestId\":";
        response += requestId;
    }
    response += ",\"data\":";
    if (hasData)
        response += dataStr; // const 引用追加，避免拷贝整个 data 载荷
    else
        response += "null";
    response += '}';
    client->text(response);
}

/**
 * @brief 发送错误响应（内联序列化，message 经 appendJsonString 安全转义）。
 */
static void sendWsError(AsyncWebSocketClient *client, const String &action,
                        const String &message, uint32_t requestId = 0)
{
    LOG_W("WebSocket请求失败: client=%u, action=%s, requestId=%u, message=%s",
          client ? client->id() : 0,
          action.c_str(),
          requestId,
          message.c_str());

    if (!client)
        return;

    String response;
    response.reserve(action.length() + message.length() + 80);
    response += "{\"action\":\"";
    response += action;
    response += "\",\"success\":false";
    if (requestId != 0)
    {
        response += ",\"requestId\":";
        response += requestId;
    }
    response += ",\"data\":{\"message\":";
    appendJsonString(response, message.c_str());
    response += "}}";
    client->text(response);
}

static void appendLogJson(JsonArray logsArray, const LogEntry &entry)
{
    JsonObject logObj = logsArray.add<JsonObject>();
    logObj["sessionId"] = entry.sessionId;
    logObj["id"] = entry.id;
    logObj["level"] = entry.level;
    logObj["tag"] = entry.tag;
    logObj["message"] = entry.message;
    logObj["timestamp"] = entry.timestamp;
}

static bool sendWsTextBufferToClient(AsyncWebSocketClient *client, const char *payload, size_t payloadLength)
{
    AsyncWebSocket *currentWs = ws;
    if (!currentWs || !client || !payload || payloadLength == 0)
        return false;

    currentWs->cleanupClients();

    if (client->status() != WS_CONNECTED || client->queueIsFull())
        return false;

    AsyncWebSocketMessageBuffer *buffer = currentWs->makeBuffer(reinterpret_cast<const uint8_t *>(payload), payloadLength);
    if (!buffer)
        return false;

    return currentWs->text(client->id(), buffer);
}

static AsyncWebSocket::SendStatus broadcastWsTextBuffer(const char *payload, size_t payloadLength)
{
    AsyncWebSocket *currentWs = ws;
    if (!currentWs || !payload || payloadLength == 0)
        return AsyncWebSocket::DISCARDED;

    currentWs->cleanupClients();

    if (currentWs->count() == 0)
        return AsyncWebSocket::DISCARDED;

    AsyncWebSocketMessageBuffer *buffer = currentWs->makeBuffer(reinterpret_cast<const uint8_t *>(payload), payloadLength);
    if (!buffer)
        return AsyncWebSocket::DISCARDED;

    return currentWs->textAll(buffer);
}

static size_t selectTailLogs(const std::deque<LogEntry> &source, LogEntry *target, size_t tailCount)
{
    const size_t copyCount = source.size() < tailCount ? source.size() : tailCount;
    const size_t startIndex = source.size() - copyCount;

    for (size_t i = 0; i < copyCount; ++i)
    {
        target[i] = source[startIndex + i];
    }

    return copyCount;
}

void handleWsLogReplay(AsyncWebSocketClient *client, const JsonDocument &req)
{
    uint32_t requestId = req["requestId"] | 0;
    const bool hasLastLogId = !req["lastLogId"].isNull();
    const uint32_t requestedLastLogId = hasLastLogId ? req["lastLogId"].as<uint32_t>() : 0;
    const bool hasRequestedSessionId = !req["sessionId"].isNull();
    const uint32_t requestedSessionId = hasRequestedSessionId ? req["sessionId"].as<uint32_t>() : 0;
    const bool sessionChanged = hasRequestedSessionId && requestedSessionId != 0 && requestedSessionId != logSessionId;
    LogEntry selectedLogs[LOG_REPLAY_MAX_BATCH_SIZE];
    size_t selectedLogCount = 0;
    const char *mode = "empty";
    uint32_t oldestAvailableLogId = 0;
    uint32_t latestAvailableLogId = 0;

    portENTER_CRITICAL(&logCacheMux);

    if (!logCache.empty())
    {
        oldestAvailableLogId = logCache.front().id;
        latestAvailableLogId = logCache.back().id;

        if (sessionChanged)
        {
            mode = "tail";
            selectedLogCount = selectTailLogs(logCache, selectedLogs, LOG_REPLAY_FALLBACK_SIZE);
        }
        else if (!hasLastLogId)
        {
            mode = "tail";
            selectedLogCount = selectTailLogs(logCache, selectedLogs, LOG_REPLAY_FALLBACK_SIZE);
        }
        else if (requestedLastLogId < oldestAvailableLogId || requestedLastLogId > latestAvailableLogId)
        {
            mode = "fallback";
            selectedLogCount = selectTailLogs(logCache, selectedLogs, LOG_REPLAY_FALLBACK_SIZE);
        }
        else if (requestedLastLogId != latestAvailableLogId)
        {
            size_t pendingCount = 0;
            for (const auto &entry : logCache)
            {
                if (entry.id > requestedLastLogId)
                {
                    pendingCount++;
                }
            }

            if (pendingCount > LOG_REPLAY_MAX_BATCH_SIZE)
            {
                mode = "fallback";
                selectedLogCount = selectTailLogs(logCache, selectedLogs, LOG_REPLAY_FALLBACK_SIZE);
            }
            else
            {
                mode = "incremental";
                for (const auto &entry : logCache)
                {
                    if (entry.id > requestedLastLogId && selectedLogCount < LOG_REPLAY_MAX_BATCH_SIZE)
                    {
                        selectedLogs[selectedLogCount++] = entry;
                    }
                }
            }
        }
    }

    portEXIT_CRITICAL(&logCacheMux);

    // 内存优化：在单个 response 文档内就地创建嵌套的 data 对象，
    // 避免 `response["data"] = data` 触发的整树深拷贝（可能含 64 条日志）。
    JsonDocument response;
    response["action"] = "log/replay";
    response["success"] = true;
    if (requestId != 0)
        response["requestId"] = requestId;

    JsonObject dataObj = response["data"].to<JsonObject>();
    dataObj["sessionId"] = logSessionId;
    dataObj["sessionChanged"] = sessionChanged;
    dataObj["mode"] = mode;
    dataObj["oldestAvailableLogId"] = oldestAvailableLogId;
    dataObj["latestAvailableLogId"] = latestAvailableLogId;

    if (hasLastLogId)
    {
        dataObj["requestedLastLogId"] = requestedLastLogId;
    }

    JsonArray logsArray = dataObj["logs"].to<JsonArray>();
    for (size_t i = 0; i < selectedLogCount; ++i)
    {
        appendLogJson(logsArray, selectedLogs[i]);
    }

    String payload;
    const size_t payloadLength = measureJson(response);
    if (!payload.reserve(payloadLength + 1))
    {
        sendWsError(client, "log/replay", "日志回放内存不足", requestId);
        return;
    }

    serializeJson(response, payload);
    if (!sendWsTextBufferToClient(client, payload.c_str(), payload.length()) && client && client->status() == WS_CONNECTED)
    {
        client->close(1013, "log replay busy");
    }
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
    LOG_I("开始扫描WiFi网络");

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

    // 内存优化：直接在单个 resultDoc 内就地构建 data（数组或对象），
    // 避免先构建独立 doc 再用 resultDoc["data"]=doc 对整个 networks 数组深拷贝。
    JsonDocument resultDoc;
    resultDoc["action"] = "wifi/scanResult";

    bool success = true;
    if (n > 0)
    {
        JsonArray networks = resultDoc["data"].to<JsonArray>();
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
        LOG_D("WiFi扫描完成，未发现可用网络");
        resultDoc["data"].to<JsonArray>();
    }
    else
    {
        // 扫描失败
        success = false;
        LOG_W("WiFi扫描失败: %s", "扫描超时或失败");
        JsonObject errObj = resultDoc["data"].to<JsonObject>();
        errObj["message"] = "扫描超时或失败";
    }

    resultDoc["success"] = success;

    if (n > 0)
    {
        LOG_I("WiFi扫描完成，发现 %d 个网络", n);
    }

    // 等待客户端连接
    int waitTimeout = 20;
    bool sent = false;
    while (waitTimeout-- > 0)
    {
        if (ws && ws->count() > 0)
        {
            // 有客户端连接，发送结果（预分配容量，避免序列化期间多次 realloc）
            String response;
            response.reserve(measureJson(resultDoc));
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

    bool wifisuccess = false;
    const char *errorMessage = ""; // 仅指向静态字面量，无需 String 堆分配

    if (connectToWiFi(ssid, password, 20))
    {
        wifisuccess = true;
    }
    else
    {
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
        LOG_W("WiFi连接测试失败: SSID=%s, 错误码=%d, 原因=%s", ssid.c_str(), WiFi.status(), errorMessage);
    }

    // 内存优化：直接在单个 resultDoc 内就地构建 data 对象，
    // 避免 resultDoc["data"]=resultData 的整树深拷贝。
    JsonDocument resultDoc;
    resultDoc["action"] = "wifi/testResult";
    resultDoc["success"] = true; // 信封层表示传输成功，业务结果见 data.success
    JsonObject dataObj = resultDoc["data"].to<JsonObject>();
    dataObj["success"] = wifisuccess;
    if (wifisuccess)
    {
        dataObj["ip"] = WiFi.localIP().toString();
    }
    else
    {
        dataObj["errorMessage"] = errorMessage; // errorMessage 指向静态字面量
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

    // 等待客户端连接
    int waitTimeout = 20;
    bool sent = false;
    while (waitTimeout-- > 0)
    {
        if (ws && ws->count() > 0)
        {
            // 有客户端连接，发送结果（预分配容量，避免序列化期间多次 realloc）
            String response;
            response.reserve(measureJson(resultDoc));
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
    data["success"] = false;
    data["ip"] = "";
    data["errorMessage"] = "";
    sendWsResponse(client, "wifi/testStatus", true, &data, requestId);
}

// =============================================================================
// WebSocket 主事件处理
// =============================================================================

void initWebSocket()
{
    if (ws != nullptr)
        return;

    if (logSessionId == 0)
    {
        logSessionId = generateLogSessionId();
    }

    ws = new AsyncWebSocket("/ws");

    ws->onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                   AwsEventType type, void *arg, uint8_t *data, size_t len)
                {
        switch (type) {
           case WS_EVT_CONNECT:
            {
                if (server->count() > 2) {
                    client->close(4013, "Too many WebSocket connections");
                    LOG_E("Client #%u rejected (limit reached)\n", client->id());
                }else{
                client->setCloseClientOnQueueFull(false);
                LOG_D("WebSocket 客户端连接: ID=%u (当前总数: %u)", client->id(), server->count());
                }
                break;
            }
            case WS_EVT_DISCONNECT:
            {
                server->cleanupClients();
                LOG_D("WebSocket客户端断开: ID=%u", client->id());
                break;
            }
            case WS_EVT_DATA:
            {
                AwsFrameInfo *info = (AwsFrameInfo*)arg;
                if (info->opcode == WS_TEXT) {
                    if (info->len > WS_MAX_INCOMING_MESSAGE_SIZE) {
                        LOG_W("WebSocket消息过大，关闭连接: ID=%u, 长度=%u", client->id(), info->len);
                        client->close(1009, "payload too large");
                        return;
                    }

                    if (!info->final || info->index != 0 || info->len != len) {
                        LOG_W("WebSocket暂不支持分片文本帧: ID=%u, 索引=%u, 帧长=%u, 片段=%u", client->id(), info->index, info->len, len);
                        client->close(1003, "fragmented payload unsupported");
                        return;
                    }

                    // 内存优化：直接从原始缓冲解析（免去 String msg 拷贝），
                    // action 以 const char* 零拷贝读取（指向文档内存），用 strcmp 比对。
                    JsonDocument req;
                    DeserializationError error = deserializeJson(req, data, len);
                    if (error) {
                        LOG_E("JSON解析错误: %s", error.c_str());
                        return;
                    }

                    const char *action = req["action"].as<const char *>();
                    if (!action) {
                        LOG_W("WebSocket消息缺少action字段");
                        return;
                    }

                    // 分发到对应处理函数
                    if (strcmp(action, "wifi/getInfo") == 0) {
                        handleWsWifiGetInfo(client, req);
                    } else if (strcmp(action, "wifi/saveConfig") == 0) {
                        handleWsWifiSaveConfig(client, req);
                    } else if (strcmp(action, "wifi/clearConfig") == 0) {
                        handleWsWifiClearConfig(client, req);
                    } else if (strcmp(action, "wifi/scan") == 0) {
                        handleWsWifiScan(client, req);
                    } else if (strcmp(action, "wifi/test") == 0) {
                        handleWsWifiTest(client, req);
                    } else if (strcmp(action, "wifi/testStatus") == 0) {
                        handleWsWifiTestStatus(client, req);
                    } else if (strcmp(action, "log/replay") == 0) {
                        handleWsLogReplay(client, req);
                    } else {
                        LOG_W("未知的WebSocket action: %s", action);
                    }
                }
                break;
            }
            case WS_EVT_ERROR:
            {
                const uint16_t errorCode = (arg != nullptr) ? *reinterpret_cast<uint16_t *>(arg) : 0;
                LOG_E("WebSocket错误: client=%u, error=%u, status=%d, queueFull=%s, dataLen=%u",
                      client ? client->id() : 0,
                      errorCode,
                      client ? client->status() : 0,
                      (client && client->queueIsFull()) ? "true" : "false",
                      static_cast<unsigned int>(len));
                if (client) client->close();  // 强制关闭异常连接
                server->cleanupClients();
                break;
            }
            case WS_EVT_PING:
                break;
            case WS_EVT_PONG:
                break;
        } });

    LOG_D("WebSocket服务器初始化完成: 路径=/ws");
}

/**
 * @brief 清理 WebSocket 服务器
 */
void cleanupWebSocket()
{
    AsyncWebSocket *currentWs = ws;
    ws = nullptr;

    if (currentWs != nullptr)
    {
        currentWs->cleanupClients();
        currentWs->closeAll();
        delete currentWs;
        portENTER_CRITICAL(&logCacheMux);
        logCache.clear();
        logSessionId = generateLogSessionId();
        nextLogId = 1;
        portEXIT_CRITICAL(&logCacheMux);
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
    doc["otaHashAlgorithm"] = "sha256";
    doc["otaHashHeader"] = OTA_HASH_HEADER;
    doc["otaHashRequired"] = false;
    doc["fileHashAlgorithm"] = "sha256";
    doc["fileHashHeader"] = FILE_HASH_HEADER;
    doc["fileHashRequired"] = true;
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

    if (isProtectedRuntimeFile(path))
    {
        sendErrorResponse(request, 403, "运行时数据文件受保护，禁止删除");
        return;
    }

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
    if (!file)
    {
        LOG_E("文件下载失败: 文件不存在: %s", path.c_str());
        sendErrorResponse(request, 404, "文件未找到");
        return;
    }
    if (file.isDirectory())
    {
        file.close();
        LOG_E("文件下载失败: 路径不是文件: %s", path.c_str());
        sendErrorResponse(request, 400, "路径不是文件");
        return;
    }

    String filename = path.substring(path.lastIndexOf('/') + 1);
    AsyncFileResponse *response = new AsyncFileResponse(file, path, "application/octet-stream");
    String disposition = "attachment; filename=\"" + filename + "\"";
    response->addHeader("Content-Disposition", disposition.c_str());
    request->send(response);
}

void handleFileSyncCheck(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
    handleJsonBody(request, data, len, index, total, [](AsyncWebServerRequest *req, JsonDocument &doc)
                   {
        JsonArray files = doc["files"].as<JsonArray>();
        if (files.isNull())
        {
            sendErrorResponse(req, 400, "缺少files数组");
            return;
        }

        JsonDocument responseDoc;
        JsonArray results = responseDoc["results"].to<JsonArray>();

        for (JsonVariant fileVariant : files)
        {
            JsonObject fileObj = fileVariant.as<JsonObject>();
            JsonObject result = results.add<JsonObject>();

            String rawPath = fileObj["path"].is<const char *>() ? String(fileObj["path"].as<const char *>()) : String("");
            rawPath.trim();

            String normalizedPath = normalizePath(rawPath);
            result["path"] = normalizedPath;

            if (!isValidFsPath(normalizedPath) || normalizedPath == "/")
            {
                result["action"] = "invalid";
                result["reason"] = "invalid-path";
                continue;
            }

            String expectedSha256 = fileObj["sha256"].is<const char *>() ? String(fileObj["sha256"].as<const char *>()) : String("");
            expectedSha256.trim();
            expectedSha256.toLowerCase();
            if (!isLowerHexString(expectedSha256, OTA_SHA256_HEX_LENGTH))
            {
                result["action"] = "invalid";
                result["reason"] = "invalid-hash";
                continue;
            }

            const bool exists = LittleFS.exists(normalizedPath);
            result["exists"] = exists;
            result["protected"] = isProtectedRuntimeFile(normalizedPath);

            if (isProtectedRuntimeFile(normalizedPath))
            {
                result["action"] = "preserve";
                result["reason"] = "protected-runtime-file";
                continue;
            }

            if (!exists)
            {
                result["action"] = "upload";
                result["reason"] = "missing";
                continue;
            }

            String currentSha256;
            if (!computeLittleFsFileSha256(normalizedPath, currentSha256))
            {
                result["action"] = "upload";
                result["reason"] = "hash-read-failed";
                continue;
            }

            result["currentSha256"] = currentSha256;
            if (currentSha256 == expectedSha256)
            {
                result["action"] = "skip";
                result["reason"] = "hash-match";
            }
            else
            {
                result["action"] = "upload";
                result["reason"] = "hash-mismatch";
            }
        }

        sendSuccessResponse(req, responseDoc); });
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
        resetFileUploadState(); // 确保清理之前的状态
        uploadState.active = true;

        String reqPath = request->hasParam("path") ? request->getParam("path")->value() : "/";
        uploadState.path = normalizePath(reqPath);
        if (!uploadState.path.endsWith("/"))
            uploadState.path += "/";

        if (!isValidFsPath(uploadState.path))
        {
            LOG_W("文件上传失败: 目标路径无效: %s", uploadState.path.c_str());
            uploadState.error = true;
            return;
        }

        if (!ensureDirectoryExistsRecursive(uploadState.path))
        {
            LOG_W("文件上传失败: 无法创建目标目录: %s", uploadState.path.c_str());
            uploadState.error = true;
            return;
        }

        File targetDir = LittleFS.open(uploadState.path);
        if (!targetDir || !targetDir.isDirectory())
        {
            if (targetDir)
                targetDir.close();
            LOG_W("文件上传失败: 目标目录无效: %s", uploadState.path.c_str());
            uploadState.error = true;
            return;
        }
        targetDir.close();

        fileUploadTargetPath = normalizePath(uploadState.path + filename);
        fileUploadTempPath = fileUploadTargetPath + ".uploading";
        if (!isValidFsPath(fileUploadTargetPath))
        {
            LOG_W("文件上传失败: 目标文件路径无效: %s", fileUploadTargetPath.c_str());
            uploadState.error = true;
            return;
        }

        if (isProtectedRuntimeFile(fileUploadTargetPath))
        {
            LOG_W("文件上传失败: 运行时数据文件受保护: %s", fileUploadTargetPath.c_str());
            uploadState.error = true;
            return;
        }
        LOG_I("文件上传开始: %s (Content-Length: %u bytes)", fileUploadTargetPath.c_str(), request->contentLength());

        if (request->hasHeader(FILE_HASH_HEADER))
        {
            uploadState.expectedSha256 = request->getHeader(FILE_HASH_HEADER)->value();
            uploadState.expectedSha256.trim();
            uploadState.expectedSha256.toLowerCase();
        }

        if (uploadState.expectedSha256.isEmpty())
        {
            LOG_W("文件上传失败: 缺少文件哈希");
            uploadState.error = true;
            uploadState.hashFormatInvalid = true;
            return;
        }

        uploadState.hashProvided = true;
        if (!isLowerHexString(uploadState.expectedSha256, OTA_SHA256_HEX_LENGTH))
        {
            LOG_W("文件上传失败: 文件哈希格式无效");
            uploadState.error = true;
            uploadState.hashFormatInvalid = true;
            return;
        }

        if (!beginFileUploadHashContext())
        {
            LOG_E("文件上传失败: 无法初始化哈希上下文");
            uploadState.error = true;
            return;
        }

        uploadState.file = LittleFS.open(fileUploadTempPath, "w");
        if (!uploadState.file)
        {
            LOG_E("文件上传失败: 无法创建临时文件: %s", fileUploadTempPath.c_str());
            uploadState.error = true;
            return;
        }
    }

    if (uploadState.error)
        return;

    if (uploadState.file && data && len > 0)
    {
        size_t written = uploadState.file.write(data, len);
        if (written != len)
        {
            LOG_E("文件上传失败: 写入错误，已删除临时文件");
            uploadState.file.close();
            cleanupFileUploadTempFile();
            uploadState.error = true;
            return;
        }

        if (!updateFileUploadHashContext(data, len))
        {
            LOG_E("文件上传失败: 哈希计算错误，已删除临时文件");
            uploadState.file.close();
            cleanupFileUploadTempFile();
            uploadState.error = true;
            return;
        }

        uploadsize += len;
        uploadState.size += len;
    }

    if (final)
    {
        uploadState.finalSeen = true;

        String calculatedSha256;
        if (!finalizeFileUploadHash(calculatedSha256))
        {
            LOG_E("文件上传失败: 无法完成哈希校验");
            if (uploadState.file)
                uploadState.file.close();
            cleanupFileUploadTempFile();
            uploadState.error = true;
            return;
        }

        if (calculatedSha256 != uploadState.expectedSha256)
        {
            LOG_E("文件上传失败: 哈希校验不匹配");
            if (uploadState.file)
                uploadState.file.close();
            cleanupFileUploadTempFile();
            uploadState.hashMismatch = true;
            uploadState.error = true;
            return;
        }

        if (uploadState.file)
            uploadState.file.close();

        const String backupPath = fileUploadTargetPath + ".backup";
        const bool targetExists = LittleFS.exists(fileUploadTargetPath);
        if (targetExists)
        {
            if (!LittleFS.rename(fileUploadTargetPath, backupPath))
            {
                LOG_E("文件上传失败: 无法备份原文件: %s", fileUploadTargetPath.c_str());
                cleanupFileUploadTempFile();
                uploadState.error = true;
                return;
            }
        }

        if (!LittleFS.rename(fileUploadTempPath, fileUploadTargetPath))
        {
            LOG_E("文件上传失败: 无法替换目标文件: %s", fileUploadTargetPath.c_str());
            cleanupFileUploadTempFile();
            if (targetExists)
            {
                if (!LittleFS.rename(backupPath, fileUploadTargetPath))
                {
                    LOG_E("文件上传回滚失败: 原文件仍保留在备份路径: %s", backupPath.c_str());
                    uploadState.rollbackRestoreFailed = true;
                }
            }
            uploadState.error = true;
            return;
        }

        if (targetExists)
        {
            if (LittleFS.exists(backupPath))
            {
                LittleFS.remove(backupPath);
            }
        }

        uploadState.success = true;
        // 上传完成，状态将在 handleUploadFileComplete 中清理
    }
}

/**
 * @brief 处理文件上传完成后的回调，返回上传结果。
 */
void handleUploadFileComplete(AsyncWebServerRequest *request)
{
    // 无需互斥锁
    if (uploadState.hashFormatInvalid)
    {
        sendErrorResponse(request, 400, "文件哈希格式无效");
    }
    else if (uploadState.hashMismatch)
    {
        sendErrorResponse(request, 400, "文件哈希校验失败");
    }
    else if (uploadState.rollbackRestoreFailed)
    {
        sendErrorResponse(request, 500, "上传失败，原文件已保留在 .backup 文件中");
    }
    else if (uploadState.error)
    {
        sendErrorResponse(request, 500, "上传失败");
    }
    else if (uploadState.active && uploadState.finalSeen && uploadState.success)
    {
        LOG_I("文件上传完成: %u bytes", uploadsize);

        sendSuccessResponse(request);
    }
    else
    {
        sendErrorResponse(request, 400, "未进行上传");
    }
    uploadsize = 0;
    resetFileUploadState();
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
    if (!executeUnlock())
    {
        LOG_W("舵机解锁失败: 舵机忙碌");
        sendErrorResponse(request, 409, "舵机忙碌");
        return;
    }
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
    if (!executeLock())
    {
        LOG_W("舵机锁定失败: 舵机忙碌");
        sendErrorResponse(request, 409, "舵机忙碌");
        return;
    }
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

        if (!executePosition(position))
        {
            sendErrorResponse(req, 409, "舵机忙碌");
            return;
        }
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
    scheduleSystemRestart(500, "manual restart request");
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
        resetOtaUploadState();
        otaUploadState.active = true;
        LOG_I("OTA固件更新开始: %s (大小: %u bytes)", filename.c_str(), request->contentLength());
        if (!validateOtaFileSize(request->contentLength()))
        {
            otaUploadState.error = true;
            LOG_E("OTA更新失败: 固件大小超限 (最大2MB)");
            Update.abort();
            return;
        }
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH))
        {
            otaUploadState.error = true;
            Update.printError(Serial);
            Update.abort();
            return;
        }

        if (request->hasHeader(OTA_HASH_HEADER))
        {
            otaUploadState.expectedSha256 = request->getHeader(OTA_HASH_HEADER)->value();
            otaUploadState.expectedSha256.trim();
            otaUploadState.expectedSha256.toLowerCase();

            if (!otaUploadState.expectedSha256.isEmpty())
            {
                otaUploadState.hashProvided = true;
                if (!isLowerHexString(otaUploadState.expectedSha256, OTA_SHA256_HEX_LENGTH))
                {
                    otaUploadState.hashFormatInvalid = true;
                    otaUploadState.error = true;
                    Update.abort();
                    return;
                }

                if (!beginOtaHashContext())
                {
                    otaUploadState.error = true;
                    Update.abort();
                    return;
                }
            }
        }
    }

    if (Update.write(data, len) != len)
    {
        otaUploadState.error = true;
        Update.printError(Serial);
        Update.abort();
        return;
    }

    if (otaUploadState.hashProvided && len > 0 && !updateOtaHashContext(data, len))
    {
        otaUploadState.error = true;
        Update.abort();
        return;
    }

    esp_task_wdt_reset();

    if (final)
    {
        otaUploadState.finalSeen = true;

        if (otaUploadState.hashProvided)
        {
            String calculatedSha256;
            if (!finalizeOtaHash(calculatedSha256))
            {
                otaUploadState.error = true;
                Update.abort();
                return;
            }

            if (calculatedSha256 != otaUploadState.expectedSha256)
            {
                otaUploadState.hashMismatch = true;
                otaUploadState.error = true;
                Update.abort();
                return;
            }
        }

        if (Update.end(true))
        {
            otaUploadState.success = true;
            LOG_I("OTA固件更新成功，系统即将重启");
        }
        else
        {
            otaUploadState.error = true;
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
    if (otaUploadState.hashFormatInvalid)
    {
        resetOtaUploadState();
        sendErrorResponse(request, 400, "固件哈希格式无效");
    }
    else if (otaUploadState.hashMismatch)
    {
        resetOtaUploadState();
        sendErrorResponse(request, 400, "固件哈希校验失败");
    }
    else if (!otaUploadState.active || otaUploadState.error || Update.hasError())
    {
        resetOtaUploadState();
        sendErrorResponse(request, 500, "OTA更新失败");
    }
    else if (!otaUploadState.finalSeen || !otaUploadState.success)
    {
        resetOtaUploadState();
        sendErrorResponse(request, 500, "OTA更新未完成");
    }
    else
    {
        sendSuccessResponse(request);
        resetOtaUploadState();
        scheduleSystemRestart(1000, "ota update complete");
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
        LOG_W("HTTP读卡超时: timeout=%lu ms, 串口缓存=%d字节", timeout, Serial1.available());
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
        LOG_D("静态资源命中gzip版本: %s", gzPath.c_str());
        AsyncWebServerResponse *response = request->beginResponse(LittleFS, gzPath, contentType);
        response->addHeader("Content-Encoding", "gzip");
        response->addHeader("Cache-Control", "public, max-age=1800");
        request->send(response);
        return;
    }

    if (LittleFS.exists(path))
    {
        LOG_D("静态资源命中原始版本: %s", path.c_str());
        AsyncWebServerResponse *response = request->beginResponse(LittleFS, path, contentType);
        response->addHeader("Cache-Control", "public, max-age=1800");
        request->send(response);
    }
    else
    {
        LOG_W("静态资源不存在: %s", path.c_str());
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
    server->on("/web/js/vendor/fflate.bundle", HTTP_GET, [](AsyncWebServerRequest *request)
               { servePrecompiledFile(request, "/web/js/vendor/fflate.bundle", "application/javascript"); });
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
    server->on("/api/files/sync-check", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, handleFileSyncCheck);
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

        wifisuccess = true;
    }
    else
    {
        addTolist(10);

        WiFi.disconnect(true);
    }

    if (!wifisuccess)
    {
        LOG_W("WiFi连接失败，切换到AP模式");
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
    uint32_t currentSessionId = 0;
    uint32_t logId = 0;

    portENTER_CRITICAL(&logCacheMux);
    currentSessionId = logSessionId;
    logId = nextLogId++;
    logCache.push_back(LogEntry(currentSessionId, logId, level, tag, message, timestamp));
    if (logCache.size() > LOG_CACHE_SIZE)
    {
        logCache.pop_front();
    }
    portEXIT_CRITICAL(&logCacheMux);

    // 构建单条日志JSON
    JsonDocument doc;
    doc["sessionId"] = currentSessionId;
    doc["id"] = logId;
    doc["level"] = level;
    doc["tag"] = tag;
    doc["message"] = message;
    doc["timestamp"] = timestamp;

    char response[LOG_SINGLE_ENTRY_JSON_BUFFER_SIZE];
    const size_t responseLength = serializeJson(doc, response, sizeof(response));
    if (responseLength == 0 || responseLength >= sizeof(response))
    {
        return;
    }

    broadcastWsTextBuffer(response, responseLength);
}
