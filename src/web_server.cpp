#include "web_server.h"
#include "html_index.h"
#include "html_ota.h"
#include "html_files.h"
#include "html_servo.h"
#include "logger.h"


// 全局变量
AsyncWebServer *server = nullptr;
WebServerMode webserverMode = WS_MODE_OFF;

// WiFi配置（与main.cpp保持一致）
const char *ssid = "OpenWIFI2.4G@320";
const char *password = "SUshe320";

// 静态变量用于文件上传处理
static File uploadFile;
static String uploadPath;
static uint64_t uploadFileSize;

// 初始化Web服务器
void initWebServer()
{
    if (server != nullptr)
    {
        // 服务器已存在，不需要重新初始化
        return;
    }

    LOG_I("\n\n=== 初始化Web服务器 ===");

    // 检查WiFi连接状态
    if (WiFi.status() != WL_CONNECTED)
    {
        LOG_I("正在连接WiFi: %s", ssid);
        WiFi.begin(ssid, password);

        // 等待连接
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
        }
        else
        {
            LOG_E("WiFi连接失败");
            return;
        }
    }
    else
    {
        LOG_I("WiFi已连接，IP地址: %s", WiFi.localIP().toString().c_str());
    }

    // 创建Web服务器实例
    server = new AsyncWebServer(80);
    webserverMode = WS_MODE_RUNNING;

    // 主页路由
    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request)
               {
        String html = String(index_html);
        // 添加WiFi名称和IP地址
        html.replace("<span id=\"ip\">-</span>", WiFi.localIP().toString());
        html.replace("<span id=\"wifi\">-</span>", WiFi.SSID());
        request->send_P(200, "text/html; charset=utf-8", html.c_str()); });

    // OTA页面路由
    server->on("/ota", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send_P(200, "text/html; charset=utf-8", ota_html); });

    // OTA上传处理
    server->on("/update", HTTP_POST, [](AsyncWebServerRequest *request)
               {
            // 上传完成后的回调
            if (Update.hasError()) {
                AsyncWebServerResponse *response = request->beginResponse(500, "text/plain", "Update failed");
                request->send(response);
            } else {
                AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Update success");
                request->send(response);
                // 延迟重启，让响应发送完成
                delay(1000);
                ESP.restart();
            } }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
               {
            // 上传过程中的回调
            handleOTAUpload(request, filename, index, data, len, final); });

    // 文件管理页面路由
    server->on("/files", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send_P(200, "text/html; charset=utf-8", files_html); });

    // ========== 文件管理API ==========

    // 获取文件系统空间信息
    server->on("/api/fs/info", HTTP_GET, [](AsyncWebServerRequest *request)
               {
        uint64_t totalBytes = LittleFS.totalBytes();
        uint64_t usedBytes = LittleFS.usedBytes();
        uint64_t freeBytes = totalBytes - usedBytes;

        // 转换为 KB
        uint32_t totalKB = totalBytes / 1024;
        uint32_t usedKB = usedBytes / 1024;
        uint32_t freeKB = freeBytes / 1024;
        uint32_t freePercent = totalBytes > 0 ? (freeBytes * 100) / totalBytes : 0;

        DynamicJsonDocument doc(256);
        doc["total"] = totalKB;
        doc["used"] = usedKB;
        doc["free"] = freeKB;
        doc["freePercent"] = freePercent;

        String response;
        serializeJsonPretty(doc, response);
        request->send(200, "application/json", response); });

    // 获取目录内容（支持path参数）
    server->on("/list", HTTP_GET, [](AsyncWebServerRequest *request)
               {
        String path = "/";
        if (request->hasParam("path")) {
            path = request->getParam("path")->value();
            // 路径安全处理：确保以/开头，去除多余的斜杠
            if (!path.startsWith("/")) path = "/" + path;
            while (path.indexOf("//") != -1) path.replace("//", "/");
        }

        File dir = LittleFS.open(path);
        if (!dir || !dir.isDirectory()) {
            request->send(404, "application/json", "{\"error\":\"Directory not found\"}");
            return;
        }

        // 使用 ArduinoJson 构建响应
        DynamicJsonDocument doc(4096);
        JsonArray array = doc.to<JsonArray>();

        File entry = dir.openNextFile();
        while (entry) {
            JsonObject obj = array.createNestedObject();
            obj["name"] = String(entry.name());
            obj["isDirectory"] = entry.isDirectory();
            if (!entry.isDirectory()) {
                obj["size"] = entry.size();
            }
            entry.close();
            entry = dir.openNextFile();
        }
        dir.close();

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response); });

    // 创建目录
    server->on("/mkdir", HTTP_POST, [](AsyncWebServerRequest *request)
               {
        if (!request->hasParam("path", true)) {
            request->send(400, "text/plain", "Missing path");
            return;
        }
        String path = request->getParam("path", true)->value();
        // 路径安全处理
        if (!path.startsWith("/")) path = "/" + path;
        while (path.indexOf("//") != -1) path.replace("//", "/");

        if (LittleFS.mkdir(path)) {
            request->send(200, "text/plain", "OK");
        } else {
            request->send(500, "text/plain", "Failed to create directory");
        } });

    // 删除文件或空目录
    server->on("/delete", HTTP_POST, [](AsyncWebServerRequest *request)
               {
        if (!request->hasParam("path", true)) {
            request->send(400, "text/plain", "Missing path");
            return;
        }
        String path = request->getParam("path", true)->value();
        if (!path.startsWith("/")) path = "/" + path;
        while (path.indexOf("//") != -1) path.replace("//", "/");

        // 打开文件以检查是否存在和类型
        File f = LittleFS.open(path);
        if (!f) {
            request->send(404, "text/plain", "Not found");
            return;
        }

        bool isDir = f.isDirectory();
        f.close();

        // 如果是目录，检查是否为空
        if (isDir) {
            File dir = LittleFS.open(path);
            File entry = dir.openNextFile();
            if (entry) {
                entry.close();
                dir.close();
                request->send(409, "text/plain", "Directory not empty");
                return;
            }
            dir.close();
        }

        // 所有文件描述符已关闭，现在执行删除
        bool success = isDir ? LittleFS.rmdir(path) : LittleFS.remove(path);

        if (success) {
            request->send(200, "text/plain", "OK");
        } else {
            request->send(500, "text/plain", "Failed to delete");
        } });

    // 文件上传（支持path参数指定目标目录）
    server->on("/upload", HTTP_POST, 
               [](AsyncWebServerRequest *request) {
                   // 上传完成后的回调
                   if (uploadFile && uploadFileSize > 0) {
                       uploadFile.close();
                       LOG_I("文件上传完成: %s (大小: %u KB)", request->getParam("path", true) ? request->getParam("path", true)->value().c_str() : "/", uploadFileSize / 1024);
                       uploadFileSize = 0;
                   }
                   request->send(200, "text/plain", "OK");
               },
               [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
               { handleFileUpload(request, filename, index, data, len, final); });

    // 下载文件
    server->on("/download", HTTP_GET, [](AsyncWebServerRequest *request)
               {
        if (!request->hasParam("path")) {
            request->send(400, "text/plain", "Missing path parameter");
            return;
        }
        
        String path = request->getParam("path")->value();
        // 路径安全处理
        if (!path.startsWith("/")) path = "/" + path;
        while (path.indexOf("//") != -1) path.replace("//", "/");
        
        // 检查文件是否存在
        if (!LittleFS.exists(path)) {
            request->send(404, "text/plain", "File not found");
            return;
        }
        
        // 检查是否为文件（而非目录）
        File f = LittleFS.open(path);
        if (!f || f.isDirectory()) {
            if (f) f.close();
            request->send(400, "text/plain", "Path is not a file");
            return;
        }
        
        f.close();
        
        // 流式传输文件，带文件名
        String filename = path.substring(path.lastIndexOf('/') + 1);
        LOG_I("下载文件: %s (文件名: %s)", path.c_str(), filename.c_str());
        
        // 使用AsyncFileResponse类设置文件名
        File file = LittleFS.open(path, "r");
        if (!file) {
            request->send(500, "text/plain", "Failed to open file");
            return;
        }
        
        // 创建文件响应
        AsyncFileResponse *response = new AsyncFileResponse(file, path, "application/octet-stream");
        
        // 设置Content-Disposition头以指定文件名
        String disposition = "attachment; filename=\"" + filename + "\"";
        response->addHeader("Content-Disposition", disposition.c_str());
        
        // 发送响应（服务器会自动删除response对象）
        request->send(response); });

    // ========== 舵机管理API ==========

    // 舵机管理页面
    server->on("/servo", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send_P(200, "text/html; charset=utf-8", servo_html); });

    // 获取舵机配置
    server->on("/api/servo/config", HTTP_GET, [](AsyncWebServerRequest *request)
               {
        uint16_t unlock, lock;
        getServoConfig(unlock, lock);

        DynamicJsonDocument doc(128);
        doc["unlock"] = unlock;
        doc["lock"] = lock;

        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
               });

    // 保存舵机配置（POST）
    server->on("/api/servo/config", HTTP_POST,
        [](AsyncWebServerRequest *request) {
            // 请求完成后的回调 - 发送成功响应
            request->send(200, "application/json", "{\"status\":\"success\"}");
        },
        NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            // 处理请求体
            static String requestBody = "";
            if (index == 0) {
                requestBody = "";
            }
            requestBody += String((char *)data, len);

            // 当接收完整请求体时处理
            if (index + len == total) {
                DynamicJsonDocument doc(128);
                DeserializationError error = deserializeJson(doc, requestBody);

                if (error) {
                    LOG_E("JSON解析错误");
                    return;
                }

                if (!doc.containsKey("unlock") || !doc.containsKey("lock")) {
                    LOG_E("缺少参数");
                    return;
                }

                uint16_t unlock = doc["unlock"];
                uint16_t lock = doc["lock"];

                saveServoConfig(unlock, lock);
                LOG_I("舵机配置已保存 - 解锁: %d, 锁定: %d", unlock, lock);
            }
        });

    // 执行解锁动作
    server->on("/api/servo/unlock", HTTP_POST, [](AsyncWebServerRequest *request)
               {
        if (isservobusy)
        {
            request->send(409, "application/json", "{\"error\":\"Servo is busy\"}");
            return;
        }

        executeUnlock();
        request->send(200, "application/json", "{\"status\":\"success\"}");
               });

    // 执行锁定动作
    server->on("/api/servo/lock", HTTP_POST, [](AsyncWebServerRequest *request)
               {
        if (isservobusy)
        {
            request->send(409, "application/json", "{\"error\":\"Servo is busy\"}");
            return;
        }

        executeLock();
        request->send(200, "application/json", "{\"status\":\"success\"}");
               });

    // 重启系统路由
    server->on("/restart", HTTP_POST, [](AsyncWebServerRequest *request)
               {
        request->send(200, "text/plain", "Restarting");
        LOG_I("收到重启请求");
        delay(500);
        ESP.restart(); });

    // 启动服务器
    server->begin();
    LOG_I("Web服务器已启动");
    LOG_I("主页访问地址: http://%s", WiFi.localIP().toString().c_str());
    LOG_I("OTA升级地址: http://%s/ota", WiFi.localIP().toString().c_str());
    LOG_I("===========================");
}

// 检查Web服务器是否运行
bool isWebServerRunning()
{
    return (webserverMode == WS_MODE_RUNNING) && (server != nullptr);
}

// 停止Web服务器
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

// 处理OTA上传
void handleOTAUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
    if (!index)
    {
        LOG_I("开始OTA上传");
        LOG_I("文件名: %s", filename.c_str());
        LOG_I("文件大小: %u 字节", request->contentLength());

        // 开始OTA更新
        if (!Update.begin(request->contentLength()))
        {
            Update.printError(Serial);
            return;
        }

        // 设置OTA完成后重启
        Update.onProgress([](size_t progress, size_t total)
                          { LOG_I("OTA进度: %u%%", (progress * 100) / total); });
    }

    // 写入数据
    if (Update.write(data, len) != len)
    {
        Update.printError(Serial);
        return;
    }

    // 上传完成
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

// 处理文件上传（支持目标路径，包含空间验证和回滚）
void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
    if (!index)
    {
        // 首次调用，获取目标路径参数
        if (request->hasParam("path", true))
        {
            uploadPath = request->getParam("path", true)->value();
        }
        else
        {
            uploadPath = "/";
        }
        // 确保路径以 / 结尾
        if (!uploadPath.endsWith("/"))
            uploadPath += "/";

        String fullPath = uploadPath + filename;
        // 路径规范化
        while (fullPath.indexOf("//") != -1)
            fullPath.replace("//", "/");

        LOG_I("开始文件上传");
        LOG_I("目标路径: %s", fullPath.c_str());
        LOG_I("文件大小: %u 字节", request->contentLength());

        // 检查剩余空间
        uint64_t totalBytes = LittleFS.totalBytes();
        uint64_t usedBytes = LittleFS.usedBytes();
        uint64_t freeBytes = totalBytes - usedBytes;
        uint64_t fileSize = request->contentLength();

        LOG_I("存储空间: 总计 %u KB, 已用 %u KB, 剩余 %u KB", 
                     totalBytes / 1024, usedBytes / 1024, freeBytes / 1024);

        // 空间不足，拒绝上传
        if (fileSize > freeBytes)
        {
            LOG_W("空间不足！需要 %u KB，但只有 %u KB 可用", fileSize / 1024, freeBytes / 1024);
            LOG_W("拒绝上传");
            return;
        }

        // 打开文件准备写入
        uploadFile = LittleFS.open(fullPath, "w");
        if (!uploadFile)
        {
            LOG_E("无法创建文件");
            return;
        }

        uploadFileSize = 0;
    }

    if (uploadFile)
    {
        // 写入数据
        size_t written = uploadFile.write(data, len);
        if (written != len)
        {
            LOG_E("写入失败！期望 %u 字节，实际写入 %u 字节", len, written);
            // 写入失败，回滚：关闭文件并删除
            uploadFile.close();
            String fullPath = uploadPath + filename;
            while (fullPath.indexOf("//") != -1)
                fullPath.replace("//", "/");
            LittleFS.remove(fullPath);
            LOG_W("已回滚：删除不完整的文件");
            return;
        }
        uploadFileSize += len;
    }

    if (final)
    {
        if (uploadFile)
        {
            uploadFile.close();
            LOG_I("文件上传完成: %s (大小: %u KB)", filename.c_str(), uploadFileSize / 1024);
        }
        uploadPath = "";
    }
}