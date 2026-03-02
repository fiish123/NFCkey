#include "web_server.h"
#include "logger.h"

// 全局变量
AsyncWebServer *server = nullptr;
WebServerMode webserverMode = WS_MODE_OFF;

// WiFi配置
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

// ========== 辅助工具函数 ==========

/**
 * 路径规范化 - 确保以/开头，去除多余斜杠
 */
String normalizePath(String path)
{
    if (!path.startsWith("/"))
    {
        path = "/" + path;
    }
    while (path.indexOf("//") != -1)
    {
        path.replace("//", "/");
    }
    return path;
}

/**
 * 路径安全验证 - 防止路径遍历攻击
 */
bool validatePath(const String &path)
{
    if (path.indexOf("..") != -1 || path.isEmpty())
    {
        return false;
    }
    return true;
}

/**
 * 舵机位置范围验证
 */
bool validateServoPosition(uint16_t position)
{
    return position <= 4095;
}

/**
 * OTA固件大小验证（限制为2MB）
 */
bool validateOtaFileSize(size_t size)
{
    return size <= 2097152; // 2MB
}

// ========== API响应处理 ==========

/**
 * ApiResponse::toJson - 将响应序列化为JSON字符串
 */
String ApiResponse::toJson() const
{
    JsonDocument doc;
    doc["success"] = success;
    doc["code"] = code;
    doc["message"] = message;
    if (data != nullptr)
    {
        doc["data"] = *data;
    }
    else
    {
        doc["data"] = nullptr;
    }
    String response;
    serializeJson(doc, response);
    return response;
}

/**
 * 发送API响应
 */
void sendApiResponse(AsyncWebServerRequest *request, const ApiResponse &response)
{
    String json = response.toJson();
    request->send(response.code, "application/json", json);
}

/**
 * 发送成功响应（便捷方法）
 */
static void sendSuccessResponse(AsyncWebServerRequest *request, JsonDocument *data = nullptr)
{
    ApiResponse response;
    response.success = true;
    response.code = 200;
    response.message = "Success";
    response.data = data;
    sendApiResponse(request, response);
}

/**
 * 发送错误响应（便捷方法）
 */
static void sendErrorResponse(AsyncWebServerRequest *request, int code, const String &message)
{
    ApiResponse response;
    response.success = false;
    response.code = code;
    response.message = message;
    response.data = nullptr;
    sendApiResponse(request, response);
}

// ========== API处理器实现 ==========

/**
 * GET /api/wifi - 获取WiFi信息
 */
void handleGetWifiInfo(AsyncWebServerRequest *request)
{
    JsonDocument *doc = new JsonDocument();
    (*doc)["ssid"] = WiFi.SSID();
    (*doc)["ip"] = WiFi.localIP().toString();
    (*doc)["rssi"] = WiFi.RSSI();
    sendSuccessResponse(request, doc);
    delete doc;
}

/**
 * GET /api/battery - 获取电池电压
 */
void handleGetBatteryInfo(AsyncWebServerRequest *request)
{
    float voltage = read_battery_voltage();
    
    JsonDocument *doc = new JsonDocument();
    (*doc)["voltage"] = round(voltage * 100) / 100; // 保留2位小数
    (*doc)["status"] = voltage > 3.4 ? "normal" : (voltage > 3.2 ? "low" : "critical");
    
    sendSuccessResponse(request, doc);
    delete doc;
}

/**
 * GET /api/system/info - 获取系统信息
 */
void handleGetSystemInfo(AsyncWebServerRequest *request)
{
    JsonDocument *doc = new JsonDocument();
    (*doc)["chipModel"] = ESP.getChipModel();
    
    uint64_t chipId = ESP.getEfuseMac();
    char chipIdStr[17];
    snprintf(chipIdStr, sizeof(chipIdStr), "%016llX", chipId);
    (*doc)["chipId"] = chipIdStr;
    
    (*doc)["version"] = serverConfig.firmwareVersion;
    
    sendSuccessResponse(request, doc);
    delete doc;
}

/**
 * GET /api/filesystem - 获取文件系统信息
 */
void handleGetFileSystemInfo(AsyncWebServerRequest *request)
{
    uint64_t totalBytes = LittleFS.totalBytes();
    uint64_t usedBytes = LittleFS.usedBytes();
    uint64_t freeBytes = totalBytes - usedBytes;

    uint32_t totalKB = totalBytes / 1024;
    uint32_t usedKB = usedBytes / 1024;
    uint32_t freeKB = freeBytes / 1024;
    uint32_t freePercent = totalBytes > 0 ? (freeBytes * 100) / totalBytes : 0;

    JsonDocument *doc = new JsonDocument();
    (*doc)["total"] = totalKB;
    (*doc)["used"] = usedKB;
    (*doc)["free"] = freeKB;
    (*doc)["freePercent"] = freePercent;

    sendSuccessResponse(request, doc);
    delete doc;
}

/**
 * GET /api/files?path=/ - 列出文件
 */
void handleListFiles(AsyncWebServerRequest *request)
{
    String path = "/";
    if (request->hasParam("path"))
    {
        path = request->getParam("path")->value();
        path = normalizePath(path);
    }

    if (!validatePath(path))
    {
        sendErrorResponse(request, 400, "Invalid path");
        return;
    }

    File dir = LittleFS.open(path);
    if (!dir || !dir.isDirectory())
    {
        sendErrorResponse(request, 404, "Directory not found");
        return;
    }

    JsonDocument *doc = new JsonDocument();
    JsonArray array = doc->to<JsonArray>();

    File entry = dir.openNextFile();
    while (entry)
    {
        JsonObject obj = array.add<JsonObject>();
        obj["name"] = String(entry.name());
        obj["isDirectory"] = entry.isDirectory();
        if (!entry.isDirectory())
        {
            obj["size"] = entry.size();
        }
        entry.close();
        entry = dir.openNextFile();
    }
    dir.close();

    sendSuccessResponse(request, doc);
    delete doc;
}

/**
 * POST /api/directories - 创建目录
 */
void handleCreateDirectory(AsyncWebServerRequest *request)
{
    if (!request->hasParam("path", true))
    {
        sendErrorResponse(request, 400, "Missing path parameter");
        return;
    }
    
    String path = request->getParam("path", true)->value();
    path = normalizePath(path);

    if (!validatePath(path))
    {
        sendErrorResponse(request, 400, "Invalid path");
        return;
    }

    if (LittleFS.mkdir(path))
    {
        sendSuccessResponse(request, nullptr);
    }
    else
    {
        sendErrorResponse(request, 500, "Failed to create directory");
    }
}

/**
 * DELETE /api/files?path=/ - 删除文件或目录
 */
void handleDeleteResource(AsyncWebServerRequest *request)
{
    if (!request->hasParam("path"))
    {
        sendErrorResponse(request, 400, "Missing path parameter");
        return;
    }
    
    String path = request->getParam("path")->value();
    path = normalizePath(path);

    if (!validatePath(path))
    {
        sendErrorResponse(request, 400, "Invalid path");
        return;
    }

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
        sendSuccessResponse(request, nullptr);
    }
    else
    {
        sendErrorResponse(request, 500, "Failed to delete");
    }
}

/**
 * GET /api/files/download?path=/ - 下载文件
 */
void handleDownloadFile(AsyncWebServerRequest *request)
{
    if (!request->hasParam("path"))
    {
        sendErrorResponse(request, 400, "Missing path parameter");
        return;
    }
    
    String path = request->getParam("path")->value();
    path = normalizePath(path);
    
    if (!validatePath(path))
    {
        sendErrorResponse(request, 400, "Invalid path");
        return;
    }
    
    if (!LittleFS.exists(path))
    {
        sendErrorResponse(request, 404, "File not found");
        return;
    }
    
    File f = LittleFS.open(path);
    if (!f || f.isDirectory())
    {
        if (f) f.close();
        sendErrorResponse(request, 400, "Path is not a file");
        return;
    }
    
    f.close();
    
    String filename = path.substring(path.lastIndexOf('/') + 1);
    LOG_I("下载文件: %s (文件名: %s)", path.c_str(), filename.c_str());
    
    File file = LittleFS.open(path, "r");
    if (!file)
    {
        sendErrorResponse(request, 500, "Failed to open file");
        return;
    }
    
    AsyncFileResponse *response = new AsyncFileResponse(file, path, "application/octet-stream");
    String disposition = "attachment; filename=\"" + filename + "\"";
    response->addHeader("Content-Disposition", disposition.c_str());
    request->send(response);
}

/**
 * POST /api/files - 上传文件（处理函数）
 */
void handleUploadFile(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
    if (!index)
    {
        uploadState.error = false;
        uploadState.active = true;

        if (request->hasParam("path"))
        {
            uploadState.path = request->getParam("path")->value();
        }
        else
        {
            uploadState.path = "/";
        }

        uploadState.path = normalizePath(uploadState.path);
        if (!uploadState.path.endsWith("/"))
            uploadState.path += "/";

        String fullPath = uploadState.path + filename;
        fullPath = normalizePath(fullPath);

        LOG_I("开始文件上传");
        LOG_I("目标路径: %s", fullPath.c_str());

        if (!validatePath(fullPath))
        {
            LOG_W("无效的路径");
            uploadState.error = true;
            return;
        }

        uint64_t totalBytes = LittleFS.totalBytes();
        uint64_t usedBytes = LittleFS.usedBytes();
        uint64_t freeBytes = totalBytes - usedBytes;
        uint64_t fileSize = request->contentLength();

        if (fileSize > freeBytes)
        {
            LOG_W("空间不足！文件需要 %u 字节，剩余空间 %u 字节", fileSize, freeBytes);
            LOG_W("拒绝上传");
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
            LOG_E("写入失败！期望 %u 字节，实际写入 %u 字节", len, written);
            uploadState.file.close();
            String fullPath = uploadState.path + filename;
            fullPath = normalizePath(fullPath);
            LittleFS.remove(fullPath);
            LOG_W("已回滚：删除不完整的文件");
            uploadState.error = true;
            return;
        }
        uploadState.size += len;
    }

    if (final)
    {
        if (uploadState.file)
        {
            uploadState.file.close();
            LOG_I("文件上传完成: %s (大小: %u 字节)", filename.c_str(), uploadState.size);
        }
        uploadState.path = "";
    }
}

/**
 * POST /api/files - 上传文件（完成回调）
 */
void handleUploadFileComplete(AsyncWebServerRequest *request)
{
    if (uploadState.active && uploadState.size > 0)
    {
        uploadState.file.close();
        LOG_I("文件上传完成: %s (大小: %u 字节)", uploadState.path.c_str(), uploadState.size);
        uploadState.size = 0;
    }
    
    if (uploadState.error)
    {
        sendErrorResponse(request, 500, "Upload failed");
        uploadState.error = false;
    }
    else
    {
        sendSuccessResponse(request, nullptr);
    }
    
    uploadState.reset();
}

/**
 * GET /api/servo/config - 获取舵机配置
 */
void handleGetServoConfig(AsyncWebServerRequest *request)
{
    uint16_t unlock, lock;
    getServoConfig(unlock, lock);

    JsonDocument *doc = new JsonDocument();
    (*doc)["unlock"] = unlock;
    (*doc)["lock"] = lock;

    sendSuccessResponse(request, doc);
    delete doc;
}

/**
 * PUT /api/servo/config - 更新舵机配置
 */
void handleUpdateServoConfig(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
{
    static String requestBody = "";
    if (index == 0)
    {
        requestBody = "";
    }
    requestBody += String((char *)data, len);

    if (index + len == total)
    {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, requestBody);

        if (error)
        {
            LOG_E("JSON解析错误");
            return;
        }

        if (!doc["unlock"].is<uint16_t>() || !doc["lock"].is<uint16_t>())
        {
            LOG_E("缺少参数");
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
        LOG_I("舵机配置已保存 - 解锁: %d, 锁定: %d", unlock, lock);
        sendSuccessResponse(request, nullptr);
    }
}

/**
 * POST /api/servo/actions/unlock - 解锁动作
 */
void handleServoUnlock(AsyncWebServerRequest *request)
{
    if (isservobusy)
    {
        sendErrorResponse(request, 409, "Servo is busy");
        return;
    }

    executeUnlock();
    sendSuccessResponse(request, nullptr);
}

/**
 * POST /api/servo/actions/lock - 锁定动作
 */
void handleServoLock(AsyncWebServerRequest *request)
{
    if (isservobusy)
    {
        sendErrorResponse(request, 409, "Servo is busy");
        return;
    }

    executeLock();
    sendSuccessResponse(request, nullptr);
}

/**
 * POST /api/system/actions/restart - 重启系统
 */
void handleRestartSystem(AsyncWebServerRequest *request)
{
    sendSuccessResponse(request, nullptr);
    LOG_I("收到重启请求");
    delay(500);
    ESP.restart();
}

/**
 * POST /api/system/firmware - OTA固件上传（处理函数）
 */
void handleOtaUpdate(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
    if (!index)
    {
        LOG_I("开始OTA上传");
        LOG_I("文件名: %s", filename.c_str());
        LOG_I("文件大小: %u 字节", request->contentLength());

        if (!validateOtaFileSize(request->contentLength()))
        {
            LOG_E("固件文件过大");
            return;
        }

        if (!Update.begin(request->contentLength()))
        {
            Update.printError(Serial);
            return;
        }

        Update.onProgress([](size_t progress, size_t total)
                          { LOG_I("OTA进度: %u%%", (progress * 100) / total); });
    }

    if (Update.write(data, len) != len)
    {
        Update.printError(Serial);
        return;
    }

    if (final)
    {
        if (Update.end(true))
        {
            LOG_I("OTA上传成功，准备重启");
        }
        else
        {
            Update.printError(Serial);
        }
    }
}

/**
 * POST /api/system/firmware - OTA固件上传（完成回调）
 */
void handleOtaUpdateComplete(AsyncWebServerRequest *request)
{
    if (Update.hasError())
    {
        sendErrorResponse(request, 500, "Update failed");
    }
    else
    {
        sendSuccessResponse(request, nullptr);
        delay(1000);
        ESP.restart();
    }
}

// ========== 路由注册函数 ==========

/**
 * 注册静态资源路由
 */
void registerStaticRoutes(AsyncWebServer *server)
{
    server->on("/web/css/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send(LittleFS, "/web/css/style.css", "text/css"); });

    server->on("/web/js/common.js", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send(LittleFS, "/web/js/common.js", "application/javascript"); });
}

/**
 * 注册页面路由
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
 * 注册API路由
 */
void registerApiRoutes(AsyncWebServer *server)
{
    // WiFi信息
    server->on("/api/wifi", HTTP_GET, handleGetWifiInfo);

    // 电池信息
    server->on("/api/battery", HTTP_GET, handleGetBatteryInfo);

    // 系统信息
    server->on("/api/system/info", HTTP_GET, handleGetSystemInfo);
    server->on("/api/system/actions/restart", HTTP_POST, handleRestartSystem);

    // OTA固件更新
    server->on("/api/system/firmware", HTTP_POST, handleOtaUpdateComplete, handleOtaUpdate);

    // 文件系统
    server->on("/api/filesystem", HTTP_GET, handleGetFileSystemInfo);
    server->on("/api/files", HTTP_GET, handleListFiles);
    server->on("/api/files", HTTP_DELETE, handleDeleteResource);
    server->on("/api/files", HTTP_POST, handleUploadFileComplete, handleUploadFile);
    server->on("/api/files/download", HTTP_GET, handleDownloadFile);
    server->on("/api/directories", HTTP_POST, handleCreateDirectory);

    // 舵机控制
    server->on("/api/servo/config", HTTP_GET, handleGetServoConfig);
    server->on("/api/servo/config", HTTP_PUT,
               [](AsyncWebServerRequest *request) {},
               [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
                   // 忽略上传参数，直接调用处理函数
                   handleUpdateServoConfig(request, data, len, index, len);
               });
    server->on("/api/servo/actions/unlock", HTTP_POST, handleServoUnlock);
    server->on("/api/servo/actions/lock", HTTP_POST, handleServoLock);
}

// ========== 主要函数 ==========

/**
 * 初始化Web服务器
 */
void initWebServer()
{
    if (server != nullptr)
    {
        return;
    }

    LOG_I("\n\n=== 初始化Web服务器 ===");

    if (WiFi.status() != WL_CONNECTED)
    {
        LOG_I("正在连接WiFi: %s", serverConfig.ssid);
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
            LOG_I("WiFi连接成功!");
            LOG_I("IP地址: %s", WiFi.localIP().toString().c_str());
            addTolist(9);
        }
        else
        {
            LOG_E("WiFi连接失败");
            addTolist(10);
            return;
        }
    }
    else
    {
        LOG_I("WiFi已连接，IP地址: %s", WiFi.localIP().toString().c_str());
    }

    server = new AsyncWebServer(serverConfig.serverPort);
    webserverMode = WS_MODE_RUNNING;

    registerStaticRoutes(server);
    registerPageRoutes(server);
    registerApiRoutes(server);

    server->begin();
    LOG_I("Web服务器已启动");
    LOG_I("主页访问地址: http://%s", WiFi.localIP().toString().c_str());
    LOG_I("===========================");
}

/**
 * 检查Web服务器是否运行
 */
bool isWebServerRunning()
{
    return (webserverMode == WS_MODE_RUNNING) && (server != nullptr);
}

/**
 * 停止Web服务器
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