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

// 初始化Web服务器
void initWebServer();

// 检查Web服务器是否运行
bool isWebServerRunning();

// 停止Web服务器
void stopWebServer();

// 处理OTA上传
void handleOTAUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);

// 处理文件上传
void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);

// 舵机控制函数声明
extern bool isservobusy;
void loadServoConfig();
void saveServoConfig(uint16_t unlock, uint16_t lock);
void getServoConfig(uint16_t &unlock, uint16_t &lock);
void executeUnlock();
void executeLock();

#endif // WEBSERVER_H
