# PROJECT KNOWLEDGE BASE

ESP32-C3 (Arduino / PlatformIO) 门禁固件：AsyncWebServer 管理界面 + LittleFS 资源 + NFC / 舵机 / AAC 提示音。
固件在 `src`，网页源码在 `webui`（Vue 3 + Vite），烧进设备的资源在 `data`。

## LAYOUT
```text
src/            固件：main.cpp 主流程、web_server.cpp 界面与 API、nfc / 舵机 / audio 模块
data/           LittleFS 载荷：data/web/*.gz（Vite 构建产物，预压缩）、data/sound/*.aac、cards.json
webui/          网页源码（Vue 3 + TS + Vite）；`pnpm run build` 产出 dist/*.gz
lib/helix_aac/  裁剪版 libhelix AAC-LC（当前自写播放链使用）
scripts/        发布打包：pack_ota.py 构建网页 + 打包 OTA（ota_lib.py 为共用实现）
build_webui.sh  一键：构建 webui + 同步 data/web → 编译固件 → 构建 LittleFS
hardware/ dist/ test/ include/   参考件 / 产物 / 脚手架，非构建输入
```

## WHERE TO LOOK
| 位置 | 内容 |
|---|---|
| `src/main.cpp` | setup/loop、休眠、NFC 认证、舵机、提示音队列 `addTolist()`、`powermanager()` |
| `src/audio_player.cpp` | `audio::` 接口（`begin/enqueue/isPlaying/waitIdle/beforeLightSleep`）；内部是 AAC / 音量 / I2S 播放链；逐段复位解码器，队列空后排空 DMA 并关闭输出 |
| `src/web_server.cpp/.h` | 页面与 REST 路由、WebSocket `/ws`（WiFi 配置 + 日志） |
| `src/nfc.h/.cpp` | `NFCcard` 结构体 + `ReadCard()` |
| `src/logger.h` | `LOG_E/W/I/D/V`；日志同时广播到 WebSocket（`{action:"log",data:{…}}`） |
| `webui/src/views/*.vue` | 每页一个视图；WS/日志运行时在 `webui/src/composables/` |
| `partitions.csv` / `platformio.ini` | 双 OTA 槽各 1.25 MiB + 1.44 MiB SPIFFS；ESP32-C3 + Arduino |

## CONVENTIONS
- 网页只改 `webui/src`；`data/web/*.gz` 是构建产物，不要直接编辑。
- 日志用 `LOG_*` 宏（默认 INFO），不要用裸 `Serial.print`，否则网页端看不到。


## RUNTIME
- Web 服务启动条件：无有效卡片 / `webdebug=true`（当前源码为 true，现场应设 false）/ 连续刷两次卡。服务启动后不再停止，30 分钟后 `ESP.restart()`（debug 模式改为 `esp_deep_sleep_start()`）。
- 无 Web 服务时 `loop()` 进浅睡眠，IRQ（GPIO 4）拉高唤醒。
- Serial1 复用：读卡器 TX18@9600 / 舵机 TX0@115200，切换用 `switchconnect()`。

## NOTES
- 舵机范围 0–1280，默认 unlock=800 / lock=1180。
- 提示音 ID：1=ready 2=waiting 3=accept(随机 1–5) 4=denied 5=readerror 6=low 7=lowlow 8=connectingwifi 9=successwifi 10=failwifi；`audiounknow.aac` 未被引用。
- 电池：ADC1_CH1、分压比 1.4545。
