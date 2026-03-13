#ifndef WEBSERVER_CUS_H
#define WEBSERVER_CUS_H

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "logger.h"
#include <Preferences.h>
#include "esp_task_wdt.h"
#include <AsyncWebSocket.h>

// Web服务器状态枚举
enum WebServerMode
{
    WS_MODE_OFF,    // Web服务器关闭
    WS_MODE_RUNNING // Web服务器运行中
};

// Web服务器配置结构体
struct WebServerConfig
{
    const char *firmwareVersion;
    uint16_t serverPort;
};

// 文件上传状态结构体
struct FileUploadState
{
    File file;
    String path;
    uint64_t size;
    bool error;
    bool active;

    void reset()
    {
        if (file)
            file.close();
        path = "";
        size = 0;
        error = false;
        active = false;
    }
};

// API响应结构体（保留用于HTTP API）
struct ApiResponse
{
    bool success;
    int code;
    String message;
    JsonDocument data;
    bool hasData;

    ApiResponse() : success(false), code(200), message(""), hasData(false) {}

    String toJson() const;
};

// 初始化文件系统
bool initFileSystem();

// 初始化Web服务器
void initWebServer();

// 检查Web服务器是否运行
bool isWebServerRunning();

// 停止Web服务器
void stopWebServer();

// WiFi配置相关函数声明
bool loadWifiConfig(String &ssid, String &password);
bool saveWifiConfig(const String &ssid, const String &password);
bool clearWifiConfig();
void startAPMode();

// API处理器函数声明（仅保留非WiFi的HTTP API）
void handleGetBatteryInfo(AsyncWebServerRequest *request);
void handleGetSystemInfo(AsyncWebServerRequest *request);
void handleGetFileSystemInfo(AsyncWebServerRequest *request);
void handleListFiles(AsyncWebServerRequest *request);
void handleCreateDirectory(AsyncWebServerRequest *request);
void handleDeleteResource(AsyncWebServerRequest *request);
void handleDownloadFile(AsyncWebServerRequest *request);
void handleUploadFileComplete(AsyncWebServerRequest *request);
void handleUploadFile(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
void handleGetCardsList(AsyncWebServerRequest *request);
void handleAddCard(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleDeleteCard(AsyncWebServerRequest *request);
void handleReadCard(AsyncWebServerRequest *request);
void handleTestCard(AsyncWebServerRequest *request);
void handleGetCardLogicConfig(AsyncWebServerRequest *request);
void handleSetCardLogicConfig(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleGetServoConfig(AsyncWebServerRequest *request);
void handleServoUnlock(AsyncWebServerRequest *request);
void handleServoLock(AsyncWebServerRequest *request);
void handleRestartSystem(AsyncWebServerRequest *request);
void handleOtaUpdateComplete(AsyncWebServerRequest *request);
void handleOtaUpdate(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);

// 舵机控制函数声明
extern bool isservobusy;
void loadServoConfig();
void saveServoConfig(uint16_t unlock, uint16_t lock);
void getServoConfig(uint16_t &unlock, uint16_t &lock);
void executeUnlock();
void executeLock();
void executePosition(uint16_t position);

extern void addTolist(unsigned int in);

// 卡片逻辑控制变量声明
extern bool cardLogicEnabled;

// 电池电压函数声明
float read_battery_voltage();

// 日志缓存结构体
struct LogEntry
{
    uint32_t id;
    int level;
    String tag;
    String message;
    unsigned long timestamp;

    LogEntry(uint32_t logId, int l, const char *t, const char *m, unsigned long ts)
        : id(logId), level(l), tag(t), message(m), timestamp(ts) {}
};

// WebSocket 日志相关函数
void broadcastLogToWebSocket(int level, const char *tag, const char *message);
void initWebSocket();
void cleanupWebSocket();

// WebSocket WiFi 管理相关内部函数
void handleWsWifiGetInfo(AsyncWebSocketClient *client, const JsonDocument &req);
void handleWsWifiScan(AsyncWebSocketClient *client, const JsonDocument &req);
void handleWsWifiSaveConfig(AsyncWebSocketClient *client, const JsonDocument &req);
void handleWsWifiClearConfig(AsyncWebSocketClient *client, const JsonDocument &req);
void handleWsWifiTest(AsyncWebSocketClient *client, const JsonDocument &req);
void handleWsWifiTestStatus(AsyncWebSocketClient *client, const JsonDocument &req);
void handleWsLogReplay(AsyncWebSocketClient *client, const JsonDocument &req);

#endif
