#include <Arduino.h>
#include <Preferences.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "AudioTools.h"
#include "AudioTools/Disk/AudioSourceLittleFS.h"
#include "AudioTools/AudioCodecs/CodecAACHelix.h"
#include "web_server.h"
#include "logger.h"
#include "nfc.h"
#include <LittleFS.h>

// ======================================================================
//  Pin Definitions & Hardware Configuration
// ======================================================================

// 读卡器通信
#define UART1_RX_PIN 19
#define UART1_TX_reader_PIN 18
#define UART_reader_BAUDRATE 9600

// 唤醒中断
#define IRQ 4

// DAC通信
#define LRC1_PIN 5
#define CLK1_PIN 6
#define DATA1_PIN 7
#define DAC_EN 10
volatile float VOLUME1 = 1.0;

// 舵机通信
#define UART1_TX_servo_PIN 0
#define UART_servo_BAUDRATE 115200

// 5V电源EN (DAC、舵机电源)
#define EN_5V 2

// 舵机位置配置
Preferences servoPreferences;
uint16_t unlockPosition = 800; // 默认解锁位置
uint16_t lockPosition = 1180;  // 默认锁定位置
const char *PREF_NAMESPACE = "servo";
const char *PREF_UNLOCK_POS = "unlock_pos";
const char *PREF_LOCK_POS = "lock_pos";

// ======================================================================
//  Battery / ADC
// ======================================================================

#define BATTERY_ADC_CHANNEL ADC1_CHANNEL_1
#define ADC_ATTEN ADC_ATTEN_DB_12
#define VOLTAGE_DIVIDER_RATIO 1.4545f // 分压比 (R1+R2)/R2
static esp_adc_cal_characteristics_t adc_chars;

// Web服务器自动重启配置
unsigned long webServerStartTime = 0;
const unsigned long WEB_SERVER_RESTART_INTERVAL = 30 * 60 * 1000; // 30分钟

// ADC读取
float read_battery_voltage(void)
{
  uint32_t adc_reading = 0;

  // 采集16次取平均值
  for (int i = 0; i < 16; i++)
  {
    adc_reading += adc1_get_raw(BATTERY_ADC_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  adc_reading /= 16;

  // 将ADC值转换为电压(mV)
  uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);

  // 计算实际电池电压 = 分压后电压 × 分压比
  float battery_voltage = (voltage_mv / 1000.0f) * VOLTAGE_DIVIDER_RATIO;

  return battery_voltage;
}

void logMessage(const int level, const char *tag, const char *format, ...)
{

  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  // 广播到 WebSocket
  broadcastLogToWebSocket(level, tag, buffer);

  if (level > loglevel)
  {
    return;
  }

  Serial.print("[");
  Serial.print(tag);
  Serial.print("] ");
  Serial.println(buffer);
}

// ======================================================================
//  Power Management
// ======================================================================

bool cardLogicEnabled = true;
bool isdac_used = false, isservo_used = false, is5v_on = false;
void powermanager(u8_t pin, bool on)
{
  if (pin == 1)
  {
    if (on && !isdac_used)
    {
      LOG_D("DAC音频模块启用");
      isdac_used = true;
      gpio_hold_dis((gpio_num_t)DAC_EN);
      digitalWrite(DAC_EN, HIGH);
      gpio_hold_en((gpio_num_t)DAC_EN);
    }
    else if (!on && isdac_used)
    {
      LOG_D("DAC音频模块禁用");
      isdac_used = false;
      gpio_hold_dis((gpio_num_t)DAC_EN);
      digitalWrite(DAC_EN, LOW);
      gpio_hold_en((gpio_num_t)DAC_EN);
    }
  }
  else
  {
    if (on && !isservo_used)
    {
      LOG_D("舵机模块启用");
      isservo_used = true;
    }
    else if (!on && isservo_used)
    {
      LOG_D("舵机模块禁用");
      isservo_used = false;
    }
  }

  if ((isservo_used || isdac_used) && !on)
  {
    return;
  }

  if (on && !is5v_on)
  {
    LOG_D("5V电源开启");
    is5v_on = true;
    gpio_hold_dis((gpio_num_t)EN_5V);
    digitalWrite(EN_5V, HIGH);
    gpio_hold_en((gpio_num_t)EN_5V);
  }
  else if (!on && is5v_on)
  {
    LOG_D("5V电源关闭");
    is5v_on = false;
    gpio_hold_dis((gpio_num_t)EN_5V);
    digitalWrite(EN_5V, LOW);
    gpio_hold_en((gpio_num_t)EN_5V);
  }
}

// ======================================================================
//  UART Multiplexing (NFC Reader / Servo)
// ======================================================================

uint8_t connectchoice;
void switchconnect(uint8_t in)
{
  if (connectchoice == in)
  {
    return;
  }

  connectchoice = in;

  if (in == 1)
  {
    LOG_D("串口切换至读卡器模式 (9600 baud)");
    Serial1.end();
    vTaskDelay(pdMS_TO_TICKS(10));
    Serial1.begin(UART_reader_BAUDRATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_reader_PIN);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  else
  {
    LOG_D("串口切换至舵机模式 (115200 baud)");
    powermanager(2, true);
    Serial1.end();
    vTaskDelay(pdMS_TO_TICKS(10));
    Serial1.begin(UART_servo_BAUDRATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_servo_PIN);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ======================================================================
//  Audio System
// ======================================================================

// 全局音频流
I2SStream i2s;         // i2s
AACDecoderHelix helix; // aac decoder (for AudioPlayer)
AudioSourceLittleFS soundsource("/sound", "aac");
AudioPlayer player1(soundsource, i2s, helix);

// 音频文件路径映射
const char *getAudioPath(unsigned int in)
{
  VOLUME1 = 1.0;
  switch (in)
  {
  case 1:
    return "/sound/ready.aac";
  case 2:
    VOLUME1 = 0.9;
    return "/sound/wating.aac";
  case 3:
    // 随机选择 accept 音频
    randomSeed(millis());
    switch (random(1, 6))
    {
    case 1: //  Welcome to ARC
      return "/sound/accept_1.aac";
    case 2: // Ciallo～ (∠・ω< )⌒★
      VOLUME1 = 0.7;
      return "/sound/accept_2.aac";
    case 3: // 卡
      return "/sound/accept_3.aac";
    case 4: // ACCESS
      return "/sound/accept_4.aac";
    case 5: // 验证通过
      return "/sound/accept_5.aac";
    default: // ACCESS
      return "/sound/accept_4=.aac";
    }
  case 4:
    return "/sound/denied.aac";
  case 5:
    return "/sound/readerror.aac";
  case 6:
    return "/sound/low.aac";
  case 7:
    return "/sound/lowlow.aac";
  case 8:
    return "/sound/connectingwifi.aac";
  case 9:
    return "/sound/successwifi.aac";
  case 10:
    return "/sound/failwifi.aac";
  default:
    return "/sound/audiounknow.aac";
  }
}
// 播放音频
void playAudio(unsigned int in)
{
  const char *audioPath = getAudioPath(in);

  player1.setVolume(VOLUME1);

  LOG_I("开始播放音频: %s", audioPath);

  player1.playPath(audioPath);

  LOG_D("音频播放完成: %s", audioPath);
}
// 播放线程状态标记
TaskHandle_t playerListHandle = NULL;
unsigned char playlist[20] = {};
volatile unsigned int playlistcount = 0, playlistindex = 0;
volatile bool isplaying = false;
void playerList(void *parameter)
{
  while (1)
  {

    while (playlistcount - playlistindex > 0)
    {
      isplaying = true;
      playAudio(playlist[playlistindex]);
      playlistindex++;
    }

    if (isplaying)
    {
      powermanager(1, false);
      LOG_D("音频队列播放完成");

      player1.stop();

      playlistcount = 0;
      playlistindex = 0;
      isplaying = false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
// 添加播放任务到列表
void addTolist(unsigned int in)
{
  powermanager(1, true);

  playlist[playlistcount] = in;
  playlistcount++;
}

// 加载舵机配置
void loadServoConfig()
{
  servoPreferences.begin(PREF_NAMESPACE, false);
  unlockPosition = servoPreferences.getUShort(PREF_UNLOCK_POS, 800);
  lockPosition = servoPreferences.getUShort(PREF_LOCK_POS, 1180);
  servoPreferences.end();

  LOG_I("舵机配置加载成功: 解锁位置=%d, 锁定位置=%d", unlockPosition, lockPosition);
}

// 保存舵机配置
void saveServoConfig(uint16_t unlock, uint16_t lock)
{
  // 参数范围
  if (unlock > 1280)
    unlock = 1280;
  if (lock > 1280)
    lock = 1280;
  if (unlock < 0)
    unlock = 0;
  if (lock < 0)
    lock = 0;

  servoPreferences.begin(PREF_NAMESPACE, false);
  servoPreferences.putUShort(PREF_UNLOCK_POS, unlock);
  servoPreferences.putUShort(PREF_LOCK_POS, lock);
  servoPreferences.end();

  unlockPosition = unlock;
  lockPosition = lock;

  LOG_I("舵机配置已保存: 解锁位置=%d, 锁定位置=%d", unlockPosition, lockPosition);
}

// 获取舵机配置
void getServoConfig(uint16_t &unlock, uint16_t &lock)
{
  unlock = unlockPosition;
  lock = lockPosition;
}

// 发送舵机位置控制指令
void sendServoPosition(unsigned int in)
{
  // 切换uart通信
  switchconnect(2);

  uint16_t position = static_cast<uint16_t>(in);

  // 指令常量定义
  const uint8_t HEADER[2] = {0xFF, 0xFF}; // 固定包头
  const uint8_t SERVO_ID = 0x01;          // 舵机ID，单舵机控制
  const uint8_t CMD_WRITE = 0x03;         // 写入指令码
  const uint8_t POS_REGISTER = 0x1E;      // 位置寄存器地址
  const uint8_t DATA_LENGTH = 0x05;       // 长度字段：3个参数+2=5

  // 舵机位置范围限制
  const uint16_t MIN_POSITION = 0;      // 最低点
  const uint16_t MAX_POSITION = 0x0500; // 最高点（1280）

  // 限制位置值在有效范围内
  if (position < MIN_POSITION)
  {
    position = MIN_POSITION;
  }
  else if (position > MAX_POSITION)
  {
    position = MAX_POSITION;
  }

  // 拆分位置值为低字节和高字节（小端序）
  uint8_t pos_low = static_cast<uint8_t>(position & 0xFF);
  uint8_t pos_high = static_cast<uint8_t>((position >> 8) & 0xFF);

  // 构建数据部分（不包括包头）
  uint8_t data[6] = {
      SERVO_ID,     // 字节1：舵机ID
      DATA_LENGTH,  // 字节2：数据长度
      CMD_WRITE,    // 字节3：指令码
      POS_REGISTER, // 字节4：寄存器地址
      pos_low,      // 字节5：位置低字节（PL）
      pos_high      // 字节6：位置高字节（PH）
  };

  // 计算校验和
  uint16_t sum = 0;
  for (int i = 0; i < 6; i++)
  {
    sum += data[i];
  }
  uint8_t checksum = static_cast<uint8_t>(~(sum & 0xFF));

  // 发送完整指令
  Serial1.write(HEADER, 2); // 发送包头
  Serial1.write(data, 6);   // 发送数据部分
  Serial1.write(checksum);  // 发送校验和

  vTaskDelay(pdMS_TO_TICKS(100));
  switchconnect(1);
}

// 舵机动作
bool isservobusy = false;

void Switchlock()
{
  isservobusy = true;

  // UNLOCKING
  sendServoPosition(unlockPosition);

  vTaskDelay(pdMS_TO_TICKS(2000));

  // LOCKING
  sendServoPosition(lockPosition);
  vTaskDelay(pdMS_TO_TICKS(1000));

  LOG_D("舵机动作执行完成");

  powermanager(2, false);

  isservobusy = false;
}

// 执行解锁动作（供Web调用）
void executeUnlock()
{
  isservobusy = true;

  sendServoPosition(unlockPosition);
  vTaskDelay(pdMS_TO_TICKS(1000));

  isservobusy = false;

  powermanager(2, false);

  LOG_I("API调用: 舵机解锁");
}

// 执行锁定动作（供Web调用）
void executeLock()
{
  isservobusy = true;

  sendServoPosition(lockPosition);
  vTaskDelay(pdMS_TO_TICKS(1000));

  isservobusy = false;

  powermanager(2, false);

  LOG_I("API调用: 舵机锁定");
}

// 执行任意动作
void executePosition(uint16_t position)
{
  isservobusy = true;

  sendServoPosition(position);
  vTaskDelay(pdMS_TO_TICKS(1000));

  isservobusy = false;

  powermanager(2, false);

  LOG_I("API调用: 舵机移动至位置 %d", position);
}

bool webdebug = false;

// ======================================================================
//  Initialization (setup)
// ======================================================================

void setup()
{
  // 初始化调试串口
  Serial.begin(115200);
  AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Error);
  LOG_I("系统启动，开始初始化硬件与服务");

  // 初始化 UART1
  connectchoice = 2;
  Serial1.begin(UART_servo_BAUDRATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_servo_PIN);

  // 初始化ADC1
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTEN);
  // 字符化ADC用于电压转换
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
      ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  // 初始化5V电源控制
  pinMode(EN_5V, OUTPUT);

  // 初始化唤醒中断
  pinMode(IRQ, INPUT_PULLDOWN);
  gpio_wakeup_enable((gpio_num_t)IRQ, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  // 初始化DAC EN控制
  pinMode(DAC_EN, OUTPUT);

  delay(1000);

  // 加载舵机配置
  loadServoConfig();

  // 舵机复位
  sendServoPosition(lockPosition);

  while (!LittleFS.begin())
  {
    LOG_E("LittleFS文件系统挂载失败");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  LOG_I("LittleFS文件系统挂载成功");

  // 加载卡片数据
  if (!loadCardsDataFromFile() || webdebug)
  {
    LOG_W("授权卡片不可用，启动Web管理服务用于配置");
    initWebServer();
    webServerStartTime = millis();
  }

  // 初始化播放器
  auto cfg = i2s.defaultConfig();
  cfg.sample_rate = 44100;
  cfg.channels = 1;
  cfg.pin_bck = CLK1_PIN;
  cfg.pin_data = DATA1_PIN;
  cfg.pin_ws = LRC1_PIN;
  i2s.begin(cfg);
  player1.begin();
  xTaskCreatePinnedToCore(
      playerList,        // 任务函数
      "playerlist1",     // 任务名称
      1024 * 5,          // 堆栈大小（字节）
      NULL,              // 参数
      4,                 // 优先级
      &playerListHandle, // 任务句柄
      0                  // 核心编号
  );

  vTaskDelay(pdMS_TO_TICKS(100));

  // au:ready
  addTolist(1);
  vTaskDelay(pdMS_TO_TICKS(2000));

  // 切换读卡器通信
  switchconnect(1);

  vTaskDelay(pdMS_TO_TICKS(1000));

  LOG_I("系统初始化完成");
}

// ======================================================================
//  Main Loop
// ======================================================================

void loop()
{
  serviceScheduledRestart();

  // 如果Web服务器正在运行，不进入浅睡眠
  if (!isWebServerRunning())
  {
    // 进入浅睡眠
    LOG_D("进入浅睡眠");
    powermanager(1, false);
    powermanager(2, false);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_light_sleep_start();
    LOG_D("已唤醒");
  }
  else if (digitalRead(IRQ) != HIGH)
  {
    // 没有检测到卡，不执行后续逻辑
    vTaskDelay(pdMS_TO_TICKS(500));

    // 检查是否超过30分钟自动重启
    if (millis() - webServerStartTime >= WEB_SERVER_RESTART_INTERVAL)
    {
      if (webdebug)
      {
        LOG_I("Web服务器运行超时(30min)，系统即将进入深度睡眠");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_deep_sleep_start();
      }

      LOG_I("Web服务器运行超时(30min)，系统即将重启");
      vTaskDelay(pdMS_TO_TICKS(1000));
      ESP.restart();
    }
    return;
  }

  if (!cardLogicEnabled)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    return;
  }

  vTaskDelay(pdMS_TO_TICKS(10));

  // 读卡
  bool iscardkeep = true;
  uint8_t consecutiveCardCount = 0;
  while (iscardkeep)
  {
    iscardkeep = false;

    // au:wait
    addTolist(2);

    sendCardSearchCommand();

    // 读取标签
    NFCcard currentcard;
    currentcard = ReadCard();

    // 卡数据有效检查
    if (currentcard.uidLength != 0)
    {
      if (isCardAuthorized(currentcard))
      {
        // 匹配
        LOG_I("卡片验证通过，执行解锁动作");

        // 舵机动作
        if (!isservobusy)
        {

          if (consecutiveCardCount < 1)
          {
            // au:accept
            addTolist(3);
            vTaskDelay(pdMS_TO_TICKS(10));
          }
          Switchlock();
        }
      }
      else
      {
        // 不匹配
        LOG_I("卡片验证失败，拒绝访问");

        // au:denied
        addTolist(4);

        consecutiveCardCount = 0;
        ;
      }
    }
    else
    {
      // 卡数据无效

      // au:readerror
      addTolist(5);

      LOG_W("卡片数据读取异常");
      consecutiveCardCount = 0;
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 等待舵机完成动作
    while (isservobusy)
    {
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    uint32_t lasttime = millis();
    while (lasttime - millis() < 500 and digitalRead(IRQ) == LOW)
    {
      lasttime = millis();
    }

    if (digitalRead(IRQ) == HIGH)
    {
      iscardkeep = true;
      consecutiveCardCount++;
    }

    if (consecutiveCardCount >= 3)
    {
      if (!isWebServerRunning())
        LOG_I("检测到连续刷卡");
      initWebServer();
      consecutiveCardCount = 0;
      vTaskDelay(pdMS_TO_TICKS(10000));
      break;
    }
  }

  // 低电量提醒
  float voltage = read_battery_voltage();
  if (voltage <= 3.5 and voltage > 3.4)
  {
    LOG_W("电池电量低: %.2f V", voltage);
    // au:lowbat
    addTolist(6);
  }
  else if (voltage <= 3.4)
  {
    LOG_W("电池电量极低: %.2f V", voltage);
    // au:lowlowbat
    addTolist(7);
  }

  // 等待音频完成播放
  while (isplaying)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
