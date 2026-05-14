#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

// 日志级别：0=ERROR, 1=WARN, 2=INFO, 3=DEBUG, 4=VERBOSE
static constexpr int loglevel = 2;

#define LOG_E(...) logMessage(0, "ERROR", __VA_ARGS__)
#define LOG_W(...) logMessage(1, "WARN", __VA_ARGS__)
#define LOG_I(...) logMessage(2, "INFO", __VA_ARGS__)
#define LOG_D(...) logMessage(3, "DEBUG", __VA_ARGS__)
#define LOG_V(...) logMessage(4, "VERBOSE", __VA_ARGS__)

// 日志输出函数
void logMessage(const int level, const char *tag, const char *format, ...);

// WebSocket 日志广播函数
void broadcastLogToWebSocket(int level, const char *tag, const char *message);

#endif // LOGGER_H
