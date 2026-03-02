#ifndef WEBSERVER_CUS_H
#define WEBSERVER_CUS_H

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// Web服务器状态枚举
enum WebServerMode {
    WS_MODE_OFF,      // Web服务器关闭
    WS_MODE_RUNNING   // Web服务器运行中
};

// Web服务器配置结构体
struct WebServerConfig {
    const char* ssid;
    const char* password;
    const char* firmwareVersion;
    uint16_t serverPort;
};

// 文件上传状态结构体
struct FileUploadState {
    File file;
    String path;
    uint64_t size;
    bool error;
    bool active;
    
    void reset() {
        if (file) file.close();
        path = "";
        size = 0;
        error = false;
        active = false;
    }
};

// API响应结构体
struct ApiResponse {
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

// API处理器函数声明
void handleGetWifiInfo(AsyncWebServerRequest *request);
void handleGetBatteryInfo(AsyncWebServerRequest *request);
void handleGetSystemInfo(AsyncWebServerRequest *request);
void handleGetFileSystemInfo(AsyncWebServerRequest *request);
void handleListFiles(AsyncWebServerRequest *request);
void handleCreateDirectory(AsyncWebServerRequest *request);
void handleDeleteResource(AsyncWebServerRequest *request);
void handleDownloadFile(AsyncWebServerRequest *request);
void handleUploadFileComplete(AsyncWebServerRequest *request);
void handleUploadFile(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
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

// 电池电压函数声明
float read_battery_voltage();

#endif // WEBSERVER_H