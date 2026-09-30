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
├── webui/                # Web 前端源码（Vue 3 + TypeScript + Vite，构建产出 .gz）
│   └── src/              #   页面视图、组件与 composables（WebSocket / 日志 / 对话框）
├── data/                 # 部署到设备 LittleFS 的运行时资源
│   ├── web/              #   预压缩的 .gz Web 资产
│   ├── sound/            #   AAC 语音提示音文件
│   └── cards.json        #   运行时卡片数据（gitignore，不随仓库发布）
├── scripts/              # 打包脚本（手动运行）
│   ├── ota_lib.py        #   共用实现：构建 webui / 同步 data/web / 打包 OTA
│   └── pack_ota.py       #   入口：构建网页 + 校验资源 + 打包 OTA（--skip-web 可只打包）
├── build_webui.sh        # 一键：构建网页 → 编译固件 → 构建 LittleFS（本地烧录用）
├── hardware/             # PCB Gerber 文件与 BOM（参考用，非构建输入）
├── dist/                 # 打包输出：update-package.zip（用于 OTA 升级）
├── include/              # PlatformIO 脚手架头文件（非项目逻辑）
├── test/                 # PlatformIO 测试区（未维护）
├── lib/helix_aac/        # 裁剪后的第三方 AAC 解码器（仅 AAC-LC/ADTS），非项目自有逻辑
├── platformio.ini        # PlatformIO 环境与构建配置
└── partitions.csv        # Flash 分区表（双 OTA 槽 + LittleFS）
```
