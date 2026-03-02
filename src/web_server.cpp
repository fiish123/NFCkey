/**
 * @file web_server.cpp
 * @brief Web服务器实现，提供WiFi连接、文件管理、OTA升级和舵机控制API。
 */

#include "web_server.h"
#include "logger.h"

// 全局 Web 服务器实例
AsyncWebServer *server = nullptr;
// Web 服务器运行状态
WebServerMode webserverMode = WS_MODE_OFF;

// WiFi 连接配置
const char *ssid = "OpenWIFI2.4G@320";
const char *password = "SUshe320";

// 服务器配置
static WebServerConfig serverConfig = {
    ssid,
    password,
    "1.0.0",
    80};

// 文件上传状态管理
static FileUploadState uploadState;

// =============================================================================
// 辅助工具函数
// =============================================================================

/**
 * @brief 规范化文件路径：确保以 '/' 开头，移除重复斜杠。
 * @param path 原始路径
 * @return 规范化后的路径
 */
String normalizePath(String path)
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
 * @brief 验证舵机位置是否在有效范围内（0-4095）。
 * @param position 舵机位置值
 * @return true 有效，false 无效
 */
bool validateServoPosition(uint16_t position)
{
    return position <= 4095;
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
// API 响应处理
// =============================================================================

/**
 * @brief 将ApiResponse对象序列化为JSON字符串。
 * @return JSON字符串
 */
String ApiResponse::toJson() const
{
    JsonDocument doc;
    doc["success"] = success;
    doc["code"] = code;
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

/**
 * @brief 发送成功响应（200 OK）。
 * @param request HTTP请求指针
 * @param data    可选的附加数据（JSON文档）
 */
static void sendSuccessResponse(AsyncWebServerRequest *request, const JsonDocument &data = JsonDocument())
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
    LOG_E("API错误响应: %d - %s", code, message.c_str());
    ApiResponse response;
    response.success = false;
    response.code = code;
    response.message = message;
    response.hasData = false;
    sendApiResponse(request, response);
}

// =============================================================================
// API 处理器实现
// =============================================================================

/**
 * @brief 处理GET /api/wifi，返回当前WiFi信息。
 */
void handleGetWifiInfo(AsyncWebServerRequest *request)
{
    LOG_D("GET /api/wifi 请求");
    JsonDocument doc;
    doc["ssid"] = WiFi.SSID();
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    sendSuccessResponse(request, doc);
}

/**
 * @brief 处理GET /api/battery，返回电池电压和状态。
 */
void handleGetBatteryInfo(AsyncWebServerRequest *request)
{
    LOG_D("GET /api/battery 请求");
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
    LOG_D("GET /api/system/info 请求");
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
    LOG_D("GET /api/filesystem 请求");
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
        sendErrorResponse(request, 400, "Invalid path");
        return;
    }
    LOG_D("GET /api/files 请求，路径: %s", path.c_str());

    File dir = LittleFS.open(path);
    if (!dir || !dir.isDirectory())
    {
        sendErrorResponse(request, 404, "Directory not found");
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
        sendErrorResponse(request, 400, "Invalid path");
        return;
    }
    LOG_D("POST /api/directories 请求，路径: %s", path.c_str());

    if (LittleFS.mkdir(path))
    {
        LOG_D("目录创建成功");
        sendSuccessResponse(request);
    }
    else
    {
        sendErrorResponse(request, 500, "Failed to create directory");
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
        sendErrorResponse(request, 400, "Invalid path");
        return;
    }
    LOG_D("DELETE /api/files 请求，路径: %s", path.c_str());

    File f = LittleFS.open(path);
    if (!f)
    {
        sendErrorResponse(request, 404, "Not found");
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
            sendErrorResponse(request, 409, "Directory not empty");
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
        sendErrorResponse(request, 500, "Failed to delete");
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
        LOG_D("GET /api/files/download 请求，无效路径");
        sendErrorResponse(request, 400, "Invalid path");
        return;
    }
    LOG_D("GET /api/files/download 请求，路径: %s", path.c_str());

    File file = LittleFS.open(path, "r");
    if (!file || file.isDirectory())
    {
        if (file)
            file.close();
        sendErrorResponse(request, !file ? 404 : 400,
                          !file ? "File not found" : "Path is not a file");
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
    if (!index)
    {
        uploadState.error = false;
        uploadState.active = true;

        String reqPath = request->hasParam("path") ? request->getParam("path")->value() : "/";
        uploadState.path = normalizePath(reqPath);
        if (!uploadState.path.endsWith("/"))
            uploadState.path += "/";

        String fullPath = normalizePath(uploadState.path + filename);
        LOG_I("开始上传: %s", fullPath.c_str());

        // 不再进行路径安全性检查

        uint64_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
        if (request->contentLength() > freeBytes)
        {
            LOG_W("空间不足，拒绝上传");
            uploadState.error = true;
            return;
        }

        uploadState.file = LittleFS.open(fullPath, "w");
        if (!uploadState.file)
        {
            LOG_E("无法创建文件");
            uploadState.error = true;
            return;
        }
        uploadState.size = 0;
    }

    if (uploadState.file)
    {
        size_t written = uploadState.file.write(data, len);
        if (written != len)
        {
            LOG_E("写入失败，回滚删除");
            uploadState.file.close();
            String fullPath = normalizePath(uploadState.path + filename);
            LittleFS.remove(fullPath);
            uploadState.error = true;
            return;
        }
        uploadState.size += len;
    }

    if (final)
    {
        if (uploadState.file)
            uploadState.file.close();
        uploadState.path = "";
    }
}

/**
 * @brief 处理文件上传完成后的回调，返回上传结果。
 */
void handleUploadFileComplete(AsyncWebServerRequest *request)
{
    if (uploadState.active && uploadState.size > 0)
    {
        uploadState.file.close();
        LOG_I("上传完成: 大小 %u 字节", uploadState.size);
        uploadState.size = 0;
    }

    if (uploadState.error)
    {
        sendErrorResponse(request, 500, "Upload failed");
        uploadState.error = false;
    }
    else
    {
        sendSuccessResponse(request);
    }
    uploadState.reset();
}

/**
 * @brief 处理GET /api/servo/config，获取舵机解锁/锁定位置配置。
 */
void handleGetServoConfig(AsyncWebServerRequest *request)
{
    LOG_D("GET /api/servo/config 请求");
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
        sendErrorResponse(request, 409, "Servo is busy");
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
        sendErrorResponse(request, 409, "Servo is busy");
        return;
    }
    executeLock();
    sendSuccessResponse(request);
}

/**
 * @brief 处理PUT /api/servo/config，更新舵机配置。
 * @param request HTTP请求
 * @param data    JSON数据
 * @param len     当前数据块长度
 * @param index   数据块索引（0表示开始）
 * @param total   总数据长度
 */
static void handlePutServoConfig(AsyncWebServerRequest *request, uint8_t *data,
                                 size_t len, size_t index, size_t total)
{
    if (index == 0)
        LOG_D("PUT /api/servo/config 请求体大小: %d", total);

    if (len == 0 || total == 0)
    {
        sendErrorResponse(request, 400, "Empty request body");
        return;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, data, len);
    if (error)
    {
        LOG_E("JSON解析错误: %s", error.c_str());
        sendErrorResponse(request, 400, "Invalid JSON");
        return;
    }

    if (!doc["unlock"].is<uint16_t>() || !doc["lock"].is<uint16_t>())
    {
        sendErrorResponse(request, 400, "Missing parameters");
        return;
    }

    uint16_t unlock = doc["unlock"];
    uint16_t lock = doc["lock"];

    if (!validateServoPosition(unlock) || !validateServoPosition(lock))
    {
        sendErrorResponse(request, 400, "Invalid servo position (must be <= 4095)");
        return;
    }

    saveServoConfig(unlock, lock);
    LOG_I("舵机配置更新: 解锁=%d, 锁定=%d", unlock, lock);
    sendSuccessResponse(request);
}

/**
 * @brief 处理POST /api/system/actions/restart，重启系统。
 */
void handleRestartSystem(AsyncWebServerRequest *request)
{
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
        LOG_I("开始OTA: %s, 大小: %u", filename.c_str(), request->contentLength());
        if (!validateOtaFileSize(request->contentLength()))
        {
            LOG_E("固件过大");
            return;
        }
        if (!Update.begin(request->contentLength()))
        {
            Update.printError(Serial);
            return;
        }
        Update.onProgress([](size_t progress, size_t total)
                          { LOG_D("OTA进度: %u%%", (progress * 100) / total); });
    }

    if (Update.write(data, len) != len)
    {
        Update.printError(Serial);
        return;
    }

    if (final)
    {
        if (Update.end(true))
            LOG_I("OTA成功，即将重启");
        else
            Update.printError(Serial);
    }
}

/**
 * @brief 处理OTA更新完成后的回调，返回更新结果。
 */
void handleOtaUpdateComplete(AsyncWebServerRequest *request)
{
    if (Update.hasError())
        sendErrorResponse(request, 500, "Update failed");
    else
    {
        sendSuccessResponse(request);
        delay(1000);
        ESP.restart();
    }
}

// =============================================================================
// 路由注册函数
// =============================================================================

/**
 * @brief 注册静态资源路由（CSS、JS）。
 * @param server AsyncWebServer指针
 */
void registerStaticRoutes(AsyncWebServer *server)
{
    server->on("/web/css/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send(LittleFS, "/web/css/style.css", "text/css"); });
    server->on("/web/js/common.js", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send(LittleFS, "/web/js/common.js", "application/javascript"); });
}

/**
 * @brief 注册页面路由（HTML页面）。
 * @param server AsyncWebServer指针
 */
void registerPageRoutes(AsyncWebServer *server)
{
    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send(LittleFS, "/web/index.html", "text/html; charset=utf-8"); });
    server->on("/ota", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send(LittleFS, "/web/ota.html", "text/html; charset=utf-8"); });
    server->on("/files", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send(LittleFS, "/web/files.html", "text/html; charset=utf-8"); });
    server->on("/servo", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send(LittleFS, "/web/servo.html", "text/html; charset=utf-8"); });
}

/**
 * @brief 注册API路由。
 * @param server AsyncWebServer指针
 */
void registerApiRoutes(AsyncWebServer *server)
{
    server->on("/api/wifi", HTTP_GET, handleGetWifiInfo);
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

    LOG_I("\n\n=== 初始化Web服务器 ===");

    if (WiFi.status() != WL_CONNECTED)
    {
        LOG_I("连接WiFi: %s", serverConfig.ssid);
        extern void addTolist(unsigned int);
        addTolist(8);

        WiFi.begin(serverConfig.ssid, serverConfig.password);
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 40)
        {
            delay(500);
            Serial.print(".");
            attempts++;
        }

        if (WiFi.status() == WL_CONNECTED)
        {
            LOG_I("连接成功，IP: %s", WiFi.localIP().toString().c_str());
            addTolist(9);
        }
        else
        {
            LOG_E("连接失败");
            addTolist(10);
            return;
        }
    }
    else
    {
        LOG_I("WiFi已连接，IP: %s", WiFi.localIP().toString().c_str());
    }

    server = new AsyncWebServer(serverConfig.serverPort);
    webserverMode = WS_MODE_RUNNING;

    registerStaticRoutes(server);
    registerPageRoutes(server);
    registerApiRoutes(server);

    server->begin();
    LOG_I("Web服务器启动，访问 http://%s", WiFi.localIP().toString().c_str());
    LOG_I("===========================");
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
}