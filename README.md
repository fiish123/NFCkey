# DoorKey

基于 ESP32-C3 的 NFC 门禁固件，配合 AsyncWebServer 管理界面与 LittleFS 运行时资源。

## 项目结构

```text
doorkey/
├── src/                  # 固件源码：主程序、NFC 读卡、舵机控制、音频播放、Web 服务器
│   ├── main.cpp          #   启动流程、休眠、刷卡循环、舵机/音频调度
│   ├── web_server.cpp    #   HTTP 路由、REST API、WebSocket（WiFi 与日志通道）
│   ├── nfc.cpp / nfc.h   #   NFC 卡片结构与读卡逻辑
│   └── logger.h          #   日志宏（LOG_E/W/I/D/V）
├── data_src/             # 可编辑 Web 源码（gzip 打包前）
│   └── web/              #   HTML 页面、CSS、JS 及 vendor 资源
├── data/                 # 部署到设备 LittleFS 的运行时资源
│   ├── web/              #   预压缩的 .gz Web 资产
│   ├── sound/            #   AAC 语音提示音文件
│   └── cards.json        #   运行时卡片数据（gitignore，不随仓库发布）
├── scripts/              # 发布打包脚本（手动运行）
│   ├── release.py        #   完整流程：重建 Web 资产 + 打包 OTA zip
│   └── pack_ota.py       #   仅打包：复用已有 .gz，适用于仅改固件
├── hardware/             # PCB Gerber 文件与 BOM（参考用，非构建输入）
├── dist/                 # 打包输出：update-package.zip（用于 OTA 升级）
├── include/              # PlatformIO 脚手架头文件（非项目逻辑）
├── test/                 # PlatformIO 测试区（未维护）
├── lib/                  # 第三方库（audio、arduino-libhelix），非项目自有逻辑
├── platformio.ini        # PlatformIO 环境与构建配置
└── partitions.csv        # Flash 分区表（双 OTA 槽 + LittleFS）
```
