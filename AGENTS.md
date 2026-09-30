# PROJECT KNOWLEDGE BASE

ESP32-C3 (Arduino / PlatformIO) 门禁固件：AsyncWebServer 管理界面 + LittleFS 资源 + NFC / 舵机 / AAC 提示音。
固件在 `src`，网页源码在 `data_src/web`，烧进设备的资源在 `data`。

## LAYOUT
```text
src/            固件：main.cpp 主流程、web_server.cpp 界面与 API、nfc / 舵机 / audio 模块
data/           LittleFS 载荷：data/web/*.gz（预压缩网页）、data/sound/*.aac、cards.json
data_src/web/   网页源码（html + js/pages/*.js + css），打包后生成 data/web
lib/helix_aac/  裁剪过的第三方 AAC-LC/ADTS 解码器
scripts/        发布打包：release.py 重建网页 + 打包 OTA，pack_ota.py 只打包
hardware/ dist/ test/ include/   参考件 / 产物 / 脚手架，非构建输入
```

## WHERE TO LOOK
| 位置 | 内容 |
|---|---|
| `src/main.cpp` | setup/loop、休眠、NFC 认证、舵机、提示音队列 `addTolist()`、`powermanager()` |
| `src/web_server.cpp/.h` | 页面与 REST 路由、WebSocket `/ws`（WiFi 配置 + 日志） |
| `src/nfc.h/.cpp` | `NFCcard` 结构体 + `ReadCard()` |
| `src/logger.h` | `LOG_E/W/I/D/V`；日志同时广播到 WebSocket |
| `data_src/web/js/pages/*.js` | 每页一个脚本；共享运行时在 `common.js`（含 `sendWsRequest`） |
| `partitions.csv` / `platformio.ini` | 双 OTA 槽各 1.25 MiB + 1.44 MiB SPIFFS；ESP32-C3 + Arduino |

## CONVENTIONS
- 网页只改 `data_src/web`；`data/web/*.gz` 是生成物，不要直接编辑。
- 日志用 `LOG_*` 宏（默认 INFO），不要用裸 `Serial.print`，否则网页端看不到。
- WiFi 配置只走 WebSocket（`wifi/*`）；其余是 REST（`/api/battery|system|files|servo|cards`）。
- 持久化：卡片 → LittleFS `/cards.json`；舵机位置 → NVS `servo`。
- 提示音用 `getAudioPath(id)` 映射路径与音量（`data/sound/*.aac`，AAC-LC 44.1k 单声道），`web_server.cpp` 只通过 `addTolist()` 播报。
- `NFCcard` 在 `nfc.h` 与 `web_server.cpp` 各有一份定义，改动要同步。
- 版本号硬编码在 `serverConfig`（`"1.1.0"`）；`test/`、`include/` 是脚手架，无实际测试。

## AUDIO（`audio_player.*` + `audio_aac_decoder/volume/i2s_out`）
接口：`audio::begin/pin`、`enqueue(path,volume)`、`isPlaying()`、`waitIdle()`、`beforeLightSleep()/afterLightSleep()`、`end()`。
- I2S 与解码器只在播放时打开、播完即关；浅睡眠依赖这个性质（`loop()` 在 `esp_light_sleep_start()` 前调 `beforeLightSleep()`）。
- DAC/5V 供电归 `main.cpp`：`addTolist()` 上电、`loop()` 睡前断电；音频模块自己不管电源（`setPowerHook` 保留但未使用）。
- `audio::begin()` 必须早于任何 `addTolist()`（`setup()` 里排在 `initWebServer()` 之前，后者要播 8/9/10 号提示音）。
- `AacDecoder::feed()` 最多接收 `space()` 字节：读文件必须限制在 `space()` 内，否则多余的字节被丢弃、AAC 流出现空洞。

## RUNTIME
- Web 服务启动条件：无有效卡片 / `webdebug=true`（当前源码为 true，现场应设 false）/ 连续刷两次卡。服务启动后不再停止，30 分钟后 `ESP.restart()`（debug 模式改为 `esp_deep_sleep_start()`）。
- 无 Web 服务时 `loop()` 进浅睡眠，IRQ（GPIO 4）拉高唤醒。
- Serial1 复用：读卡器 TX18@9600 / 舵机 TX0@115200，切换用 `switchconnect()`。
- 上传：`/api/files` 需 `X-File-SHA256`，OTA 可选 `X-Firmware-SHA256`，`/api/files/sync-check` 返回 per-file 动作。

## NOTES
- 舵机范围 0–1280，默认 unlock=800 / lock=1180。
- 提示音 ID：1=ready 2=waiting 3=accept(随机 1–5) 4=denied 5=readerror 6=low 7=lowlow 8=connectingwifi 9=successwifi 10=failwifi；`audiounknow.aac` 未被引用。
- 电池：ADC1_CH1、分压比 1.4545；提示音阈值 ≤3.5V 低 / ≤3.4V 极低（`/api/battery` 用 >3.4 正常 / >3.2 低）。
