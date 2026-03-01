#include <Arduino.h>
#include <vector>
#include <Preferences.h>
// #include <FastLED.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecAACHelix.h"
#include "AudioTools/Disk/AudioSourceLittleFS.h"
#include "web_server.h"

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

// 舵机位置配置（使用Preferences存储）
Preferences servoPreferences;
uint16_t unlockPosition = 800;   // 默认解锁位置
uint16_t lockPosition = 1180;    // 默认锁定位置
const char* PREF_NAMESPACE = "servo";
const char* PREF_UNLOCK_POS = "unlock_pos";
const char* PREF_LOCK_POS = "lock_pos";

// LED参数
// #define NUM_LEDS 1   // LED数量
// #define DATA_PIN 8   // 数据引脚
// CRGB leds[NUM_LEDS]; // LED实例

// ADC配置
#define BATTERY_ADC_CHANNEL ADC1_CHANNEL_1
#define ADC_ATTEN ADC_ATTEN_DB_12
#define VOLTAGE_DIVIDER_RATIO 1.4545f // 分压比 (R1+R2)/R2
static esp_adc_cal_characteristics_t adc_chars;

// ADC读取
float read_battery_voltage(void)
{
  uint32_t adc_reading = 0;

  // 采集16次取平均值
  for (int i = 0; i < 16; i++)
  {
    adc_reading += adc1_get_raw(BATTERY_ADC_CHANNEL);
  }
  adc_reading /= 16;

  // 将ADC值转换为电压(mV)
  uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);

  // 计算实际电池电压 = 分压后电压 × 分压比
  float battery_voltage = (voltage_mv / 1000.0f) * VOLTAGE_DIVIDER_RATIO;

  return battery_voltage;
}

// 日志
const int loglevel = 2;
#define LOG_E(...) logMessage(0, "ERROR", __VA_ARGS__)
#define LOG_W(...) logMessage(1, "WARN", __VA_ARGS__)
#define LOG_I(...) logMessage(2, "INFO", __VA_ARGS__)
#define LOG_D(...) logMessage(3, "DEBUG", __VA_ARGS__)
#define LOG_V(...) logMessage(4, "V", __VA_ARGS__)
void logMessage(const int level, const char *tag, const char *format, ...)
{
  if (level > loglevel)
  {
    return;
  }

  Serial.print("[");
  Serial.print(tag);
  Serial.print("] ");

  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  Serial.println(buffer);
}

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
    switch (random(1, 4))
    {
    case 1: // Welcome to Rhinelab LLC, Internal Residence.
      return "/sound/accept_1.aac";
    case 2: // Ciallo～ (∠・ω< )⌒★
      VOLUME1 = 0.7;
      return "/sound/accept_2.aac";
    case 3: // 卡
      return "/sound/accept_3.aac";
    default: // Welcome to Rhinelab LLC, Internal Residence.
      return "/sound/accept_1.aac";
    }
  case 4:
    return "/sound/denied.aac";
  case 5:
    return "/sound/readerror.aac";
  case 6:
    return "/sound/low.aac";
  case 7:
    return "/sound/lowlow.aac";
  default:
    return "/sound/audiounknow.aac";
  }
}
// 播放音频
void playAudio(unsigned int in)
{
  const char *audioPath = getAudioPath(in);

  player1.setVolume(VOLUME1);

  // 播放音频文件（阻塞直到完成）
  player1.playPath(audioPath);

  LOG_I("播放音频: %s", audioPath);
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
      LOG_D("播放列表 %d/%d", playlistindex, playlistcount);
    }

    if (isplaying)
    {
      digitalWrite(DAC_EN, LOW);
      LOG_I("播放任务完成~");
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
  digitalWrite(DAC_EN, HIGH);

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
  
  Serial.printf("舵机配置加载 - 解锁: %d, 锁定: %d\n", unlockPosition, lockPosition);
}

// 保存舵机配置
void saveServoConfig(uint16_t unlock, uint16_t lock)
{
  // 参数范围校验
  if (unlock > 4095) unlock = 4095;
  if (lock > 4095) lock = 4095;
  
  servoPreferences.begin(PREF_NAMESPACE, false);
  servoPreferences.putUShort(PREF_UNLOCK_POS, unlock);
  servoPreferences.putUShort(PREF_LOCK_POS, lock);
  servoPreferences.end();
  
  unlockPosition = unlock;
  lockPosition = lock;
  
  Serial.printf("舵机配置已保存 - 解锁: %d, 锁定: %d\n", unlockPosition, lockPosition);
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
}

// 舵机动作
bool isservobusy = false;

void Switchlock()
{
  isservobusy = true;

  // UNLOCKING
  sendServoPosition(unlockPosition);

  // au:accept
  addTolist(3);
  vTaskDelay(pdMS_TO_TICKS(10));

  vTaskDelay(pdMS_TO_TICKS(2000));

  // LOCKING
  sendServoPosition(lockPosition);
  vTaskDelay(pdMS_TO_TICKS(1000));

  LOG_D("舵机完成动作");

  isservobusy = false;
}

// 执行解锁动作（供Web调用）
void executeUnlock()
{
  isservobusy = true;
  sendServoPosition(unlockPosition);
  vTaskDelay(pdMS_TO_TICKS(1000));
  isservobusy = false;
  LOG_I("Web控制: 舵机解锁");
}

// 执行锁定动作（供Web调用）
void executeLock()
{
  isservobusy = true;
  sendServoPosition(lockPosition);
  vTaskDelay(pdMS_TO_TICKS(1000));
  isservobusy = false;
  LOG_I("Web控制: 舵机锁定");
}

// NFC标签结构体
struct NFCcard
{
  uint8_t uid[4];
  uint8_t uidLength;
};

// 授权卡片列表
const NFCcard authorizedCards[] = {
    {{0xF1, 0xB3, 0x9A, 0x3E}, 4}, // A
    {{0x1, 0x23, 0x8, 0x72}, 4},   // L
    {{0xE8, 0xFB, 0xB4, 0xCD}, 4}, // Y
    {{0x2E, 0xE, 0xAD, 0xE0}, 4},  // H
    {{0xE, 0x23, 0x5D, 0x80}, 4},  // X
    {{0x41, 0x32, 0xE4, 0xBA}, 4}, // YH

};
const int Cardscount = 6;

// 匹配卡片
bool isCardAuthorized(const NFCcard &currentCard, const NFCcard authorizedList[], const int listSize)
{
  // 遍历授权列表中的每一张卡
  for (int i = 0; i < listSize; i++)
  {
    // 先检查长度，长度不同则直接跳过
    if (currentCard.uidLength != authorizedList[i].uidLength)
    {
      continue;
    }

    // 长度相同，再逐字节比较UID
    bool isMatch = true;
    for (int j = 0; j < currentCard.uidLength; j++)
    {
      if (currentCard.uid[j] != authorizedList[i].uid[j])
      {
        isMatch = false; // 发现一个字节不匹配
        break;           // 跳出内层循环，比较下一张授权卡
      }
    }

    // 匹配
    if (isMatch)
    {
      return true;
    }
  }

  // 遍历完所有授权卡都没找到匹配的
  return false;
}

// 读卡函数
NFCcard ReadCard()
{
  uint8_t rxBuffer[20];      // 缓冲区
  uint8_t bufferIndex = 0;   // 缓冲索引
  bool frameStarted = false; // 接收标志
  unsigned long lastReceiveTime = 0;
  const unsigned long TIMEOUT_MS = 100; // 超时时长

  NFCcard readdata;
  readdata.uidLength = 0; // 初始化为无效状态

  // 超时检查：如果已经开始接收帧但超过设定时间没有收到完整数据，则重置状态
  if (frameStarted && (millis() - lastReceiveTime > TIMEOUT_MS))
  {
    LOG_V("接收超时，重置接收状态");
    frameStarted = false;
    bufferIndex = 0;
    return readdata;
  }

  while (Serial1.available() > 0)
  {
    uint8_t incomingByte = Serial1.read();
    lastReceiveTime = millis(); // 更新最后接收时间

    LOG_V("收到字节: 0x%X", incomingByte);

    // 寻找帧起始符 0x20
    if (!frameStarted && incomingByte == 0x20)
    {

      frameStarted = true;
      bufferIndex = 0;
      rxBuffer[bufferIndex++] = incomingByte;
      lastReceiveTime = millis(); // 开始接收时记录时间
    }
    // 如果已经开始接收帧
    else if (frameStarted)
    {
      rxBuffer[bufferIndex++] = incomingByte;

      // 检查是否收到完整的帧 (14字节)
      if (bufferIndex >= 14)
      {

        // 检查帧结构是否正确 (起始符和结束符)
        if (rxBuffer[0] != 0x20 || rxBuffer[13] != 0x03)
        {

          LOG_V("帧结构错误: 起始符或结束符不正确");
          LOG_V("起始符: 0x%X, 结束符: 0x%X", rxBuffer[0], rxBuffer[13]);

          frameStarted = false;
          bufferIndex = 0;
          readdata.uidLength = 0;
          return readdata;
        }

        // 计算校验和验证数据完整性
        uint8_t checksum = 0;
        for (int i = 1; i <= 11; i++)
        {
          checksum ^= rxBuffer[i];
        }
        checksum = ~checksum;

        if (checksum != rxBuffer[12])
        {
          LOG_V("校验和错误");
          frameStarted = false;
          bufferIndex = 0;
          readdata.uidLength = 0;
          return readdata;
        }

        // 提取序列号 (第9-12字节)
        readdata.uidLength = 4;
        readdata.uid[0] = rxBuffer[8];
        readdata.uid[1] = rxBuffer[9];
        readdata.uid[2] = rxBuffer[10];
        readdata.uid[3] = rxBuffer[11];

        // 输出序列号
        char serialStr[9];
        sprintf(serialStr, "%02X%02X%02X%02X", readdata.uid[0], readdata.uid[1], readdata.uid[2], readdata.uid[3]);

        LOG_V("卡片序列号: %s", serialStr);

        frameStarted = false;
        bufferIndex = 0;
        return readdata;
      }

      // 防止缓冲区溢出
      if (bufferIndex >= sizeof(rxBuffer))
      {
        LOG_V("缓冲区溢出，重置接收状态");
        frameStarted = false;
        bufferIndex = 0;
        readdata.uidLength = 0;
        return readdata;
      }
    }
  }

  // 没有读取到完整数据时返回空数据
  readdata.uidLength = 0;
  return readdata;
}

// 读卡指令
void sendCardSearchCommand()
{
  // 寻卡指令
  uint8_t cardSearchCmd[] = {0x20, 0x00, 0x27, 0x00, 0xD8, 0x03};

  vTaskDelay(pdMS_TO_TICKS(50));
  while (Serial1.available() > 0)
  {
    Serial1.read();
  }

  // 通过Serial1发送指令
  Serial1.write(cardSearchCmd, sizeof(cardSearchCmd));

  unsigned long last, now;
  last = millis();
  now = last;
  while (Serial1.available() < 14 && now - last < 1000)
  {
    vTaskDelay(pdMS_TO_TICKS(1));
    now = millis();
  }
  LOG_V("读卡耗时 %dms ", now - last);
}
// 连续读卡计数器 - 用于激活Web服务器
volatile int consecutiveCardCount = 0;
const int CARD_COUNT_TO_ACTIVATE_WEBSERVER = 3;
unsigned long lastCardReadTime = 0;
const unsigned long CARD_READ_TIMEOUT_MS = 10000; // 10秒内连续读卡才计数



void setup()
{
  // 低功耗
  // setCpuFrequencyMhz(80);

  // 初始化调试串口
  Serial.begin(115200);
   AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Error);

  // 初始化 UART1
  Serial1.begin(UART_servo_BAUDRATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_servo_PIN);

  // 初始化ADC1
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTEN);
  // 字符化ADC用于电压转换
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
      ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, 1100, &adc_chars);





  // 初始化LED
  // FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  // FastLED.setBrightness(150);
  // leds[0] = CRGB::White;
  // FastLED.show();

  // 初始化5V电源控制
  pinMode(EN_5V, OUTPUT);
  digitalWrite(EN_5V, HIGH);

  // 初始化唤醒中断
  pinMode(IRQ, INPUT_PULLDOWN);
  gpio_wakeup_enable((gpio_num_t)IRQ, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  pinMode(DAC_EN, OUTPUT);
  digitalWrite(DAC_EN, HIGH);

  delay(1000);

  // 加载舵机配置
  loadServoConfig();

  // 舵机复位
  sendServoPosition(lockPosition);

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
  digitalWrite(EN_5V, LOW);
  digitalWrite(DAC_EN, LOW);

  // 切换读卡器通信
  Serial1.end();
  vTaskDelay(pdMS_TO_TICKS(200));
  Serial1.begin(UART_reader_BAUDRATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_reader_PIN);

  vTaskDelay(pdMS_TO_TICKS(1000));

  LOG_I("初始化完成");

  // leds[0] = CRGB::Black;
  // FastLED.show();
}

void loop()
{
  // 如果Web服务器正在运行，不进入浅睡眠
  if (!isWebServerRunning())
  {
    // 进入浅睡眠
    LOG_I("进入浅睡眠");
    gpio_hold_en((gpio_num_t)EN_5V);
    gpio_hold_en((gpio_num_t)DAC_EN);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_light_sleep_start();
    gpio_hold_dis((gpio_num_t)EN_5V);
    gpio_hold_dis((gpio_num_t)DAC_EN);
    digitalWrite(EN_5V, HIGH);
    LOG_I("已唤醒");
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  else
  {
    // Web服务器运行时，保持唤醒状态
    vTaskDelay(pdMS_TO_TICKS(1000));
    return;
  }

  // au:wait
  addTolist(2);

  // 读卡
  sendCardSearchCommand();

  // 检查 UART1 是否有数据可读
  if (Serial1.available() > 0)
  {
    LOG_I("检测到卡");

    // 读取标签
    NFCcard currentcard;
    currentcard = ReadCard();

    // 卡数据有效检查
    if (currentcard.uidLength != 0)
    {
      if (isCardAuthorized(currentcard, authorizedCards, Cardscount))
      {
        // 匹配
        LOG_I("卡授权");

        // 检查是否超时重置计数器
        if (millis() - lastCardReadTime > CARD_READ_TIMEOUT_MS)
        {
          consecutiveCardCount = 0;
        }

        // 增加计数器
        consecutiveCardCount++;
        lastCardReadTime = millis();
        LOG_I("连续读卡次数: %d/%d", consecutiveCardCount, CARD_COUNT_TO_ACTIVATE_WEBSERVER);

        // 检查是否达到激活Web服务器的次数
        if (consecutiveCardCount >= CARD_COUNT_TO_ACTIVATE_WEBSERVER)
        {
          if (!isWebServerRunning())
          {
            LOG_I("=== 激活Web服务器模式 ===");
            LittleFS.begin();
            initWebServer();
            consecutiveCardCount = 0; // 重置计数器
          }
        }

        // 切换舵机通信
        Serial1.end();
        vTaskDelay(pdMS_TO_TICKS(10));
        Serial1.begin(UART_servo_BAUDRATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_servo_PIN);
        vTaskDelay(pdMS_TO_TICKS(100));

        // leds[0] = CRGB::Green;
        // FastLED.show();

        // 舵机动作
        if (!isservobusy)
        {
          Switchlock();
        }
      }
      else
      {
        // 不匹配
        // leds[0] = CRGB::Red;
        // FastLED.show();
        LOG_I("卡拒绝");

        // au:denied
        addTolist(4);
        
        // 不匹配的卡重置计数器
        consecutiveCardCount = 0;
      }
    }
    else
    {
      // 卡数据无效
      // leds[0] = CRGB::Yellow;
      // FastLED.show();

      // au:readerror
      addTolist(5);

      LOG_W("卡数据异常");

      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 等待舵机完成动作
    while (isservobusy)
    {
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 切换读卡器通信
    Serial1.end();
    vTaskDelay(pdMS_TO_TICKS(50));
    Serial1.begin(UART_reader_BAUDRATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_reader_PIN);

    // leds[0] = CRGB::Black;
    // FastLED.show();
  }

  // 低电量提醒
  float voltage = read_battery_voltage();
  LOG_I("电池电压: %f", voltage);
  if (voltage <= 3.4 and voltage > 3.2)
  {
    LOG_W("电量低");
    // leds[0] = CRGB::Red;
    // au:lowbat
    addTolist(6);
    // leds[0] = CRGB::Black;
    // FastLED.show();
  }
  else if (voltage <= 3.2)
  {
    LOG_E("电量极低");
    // leds[0] = CRGB::Red;
    // au:lowlowbat
    addTolist(7);
    // leds[0] = CRGB::Black;
    // FastLED.show();
  }

  // 等待音频完成播放
  while (isplaying)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  digitalWrite(EN_5V, LOW);
}
