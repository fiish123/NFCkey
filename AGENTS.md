# PROJECT KNOWLEDGE BASE

ESP32-C3 (Arduino / PlatformIO) 门禁固件：AsyncWebServer 管理界面 + LittleFS 资源 + NFC / 舵机 / AAC 提示音。
固件在 `src`，网页源码在 `webui`（Vue 3 + Vite），烧进设备的资源在 `data`。

## LAYOUT
```text
src/            固件：main.cpp 主流程、web_server.cpp 界面与 API、nfc / 舵机 / audio 模块
data/           LittleFS 载荷：data/web/*.gz（Vite 构建产物，预压缩）、data/sound/*.aac、cards.json
webui/          网页源码（Vue 3 + TS + Vite）；`pnpm run build` 产出 dist/*.gz
lib/helix_aac/  裁剪过的第三方 AAC-LC/ADTS 解码器
scripts/        发布打包：pack_ota.py 构建网页 + 打包 OTA（ota_lib.py 为共用实现）
build_webui.sh  一键：构建 webui + 同步 data/web → 编译固件 → 构建 LittleFS
hardware/ dist/ test/ include/   参考件 / 产物 / 脚手架，非构建输入
```

## WHERE TO LOOK
| 位置 | 内容 |
|---|---|
| `src/main.cpp` | setup/loop、休眠、NFC 认证、舵机、提示音队列 `addTolist()`、`powermanager()` |
| `src/web_server.cpp/.h` | 页面与 REST 路由、WebSocket `/ws`（WiFi 配置 + 日志） |
| `src/nfc.h/.cpp` | `NFCcard` 结构体 + `ReadCard()` |
| `src/logger.h` | `LOG_E/W/I/D/V`；日志同时广播到 WebSocket（`{action:"log",data:{…}}`） |
| `webui/src/views/*.vue` | 每页一个视图；WS/日志运行时在 `webui/src/composables/` |
| `partitions.csv` / `platformio.ini` | 双 OTA 槽各 1.25 MiB + 1.44 MiB SPIFFS；ESP32-C3 + Arduino |

## CONVENTIONS
- 网页只改 `webui/src`；`data/web/*.gz` 是构建产物，不要直接编辑。
- 路由：ESPAsyncWebServer 3.x 未启用 `ASYNCWEBSERVER_REGEX`，`"^...$"` 写法不会当正则。
  静态路径用 `AsyncURIMatcher::exact()`，动态子路径用 `"/前缀/*"`（普通字符串是
  “精确 + 子路径前缀”语义，会让 `/api/cards` 吞掉 `/api/cards/read`）。
- WebUI 读取的 GET 接口返回**裸 JSON**（`sendRawJson`，如 `/api/battery|system/info|cards|files|servo`）；
  写操作才用带信封的 `sendSuccessResponse`（`{success,message,data}`），错误用 `sendErrorResponse`。
- 静态资源只回退 index.html 给“页面”路径：`/assets/**` 和带扩展名的路径一律 404，
  否则浏览器会把 text/html 当模块脚本，直接白屏。
- 日志用 `LOG_*` 宏（默认 INFO），不要用裸 `Serial.print`，否则网页端看不到。
- WiFi 全部走 WebSocket：`wifi/getInfo|scan|test|testStatus|saveConfig|clearConfig`，结果用
  `wifi/scanResult`、`wifi/testResult` 事件推送（扫描/测试都在独立任务里跑）。没有 REST 版本：
  阻塞式 `WiFi.scanNetworks()` 跑在 AsyncTCP 任务里会拖死网络栈并复位设备。
  `wifi/test` 期间设备会短暂断开当前 WiFi，前端用 `wifi/testStatus` 兜底取回结论。
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
- 上传：文件走 `/api/files/upload`（multipart，字段 `path` 必须排在 `file` 前面，固件在文件首块就读取目标目录）；
  `X-File-SHA256` 由 WebUI 提供（`webui/src/utils/sha256.ts` 是纯 JS 实现——http 明文下浏览器没有
  `crypto.subtle`），固件收到就校验，缺失则跳过校验；OTA 走 `/update`（字段 `firmware`），`X-Firmware-SHA256` 同理。
- OTA 升级包：设备**不接受 zip**。网页端（`webui/src/utils/{zip,updatePackage,sha256}.ts`）用浏览器内置
  `DecompressionStream('deflate-raw')` 解包 `pack_ota.py` 产出的 `update-package.zip`，然后：
  先 `POST /api/files/sync-check`（`{files:[{path,sha256}]}`）让固件逐个比对 LittleFS 里已有文件的
  SHA-256，返回 `skip/upload/preserve/invalid`，只上传需要更新的；最后才上传 `firmware.bin`——
  因为 `/update` 写完后设备 1 秒内自动重启。

## NOTES
- 舵机范围 0–1280，默认 unlock=800 / lock=1180。
- 提示音 ID：1=ready 2=waiting 3=accept(随机 1–5) 4=denied 5=readerror 6=low 7=lowlow 8=connectingwifi 9=successwifi 10=failwifi；`audiounknow.aac` 未被引用。
- 电池：ADC1_CH1、分压比 1.4545；提示音阈值 ≤3.5V 低 / ≤3.4V 极低（`/api/battery` 用 >3.4 正常 / >3.2 低）。
