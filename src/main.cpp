#include <Arduino.h>
#include <vector>
#include <FastLED.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecAACHelix.h"
#include "MP3DATA.h"

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
float VOLUME1 = 1;

// 舵机通信
#define UART1_TX_servo_PIN 0
#define UART_servo_BAUDRATE 115200

// 5V电源EN (DAC、舵机电源)
#define EN_5V 2

// LED参数
#define NUM_LEDS 1   // LED数量
#define DATA_PIN 8   // 数据引脚
CRGB leds[NUM_LEDS]; // LED实例

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

// 全局音频流
I2SStream i2s;                           // i2s
AACDecoderHelix helix;                   // aac decoder
VolumeStream volume(i2s);                // volume to i2s
EncodedAudioStream out(&volume, &helix); // output to volume
StreamCopy copier;                       // copy in to decoder
// 播放线程状态标记
bool isplaying = false;
bool isplListrun = false;
unsigned int arrsize;
// 解码主线程
TaskHandle_t playerHandle = NULL;
void player(void *parameter)
{
  isplaying = true;

  // 音频FLASH数据流
  const unsigned char *DATA = (const unsigned char *)parameter;
  MemoryStream data(DATA, arrsize);

  // i2s配置
  auto cfg = i2s.defaultConfig();
  cfg.sample_rate = 44100;
  cfg.channels = 1;
  cfg.pin_bck = CLK1_PIN;
  cfg.pin_data = DATA1_PIN;
  cfg.pin_ws = LRC1_PIN;
  i2s.begin(cfg);
  // 音量控制配置
  volume.setVolume(VOLUME1);
  volume.begin();

  // 输出流配置
  out.begin();
  copier.begin(out, data);
  while (data.available())
  {
    // 解码线程
    copier.copy();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  helix.end(); // flush output
  auto info = out.decoder().audioInfo();
  LOGI("The audio rate from the mp3 file is %d", info.sample_rate);
  LOGI("The channels from the mp3 file is %d", info.channels);
  i2s.end();
  volume.end();
  copier.end();
  out.end();
  data.end();
  vTaskDelay(pdMS_TO_TICKS(100));
  isplaying = false;
  vTaskDelete(NULL);
}
// 切换音源
void switchaudio(unsigned int in)
{

  void *dataarr = NULL;
  String pname = "player";
  int choi = 0;
  VOLUME1 = 1;
  switch (in)
  {
  case 1:
    // ready

    arrsize = sizeof(ready_data);
    dataarr = (void *)ready_data;
    pname = "player";
    break;
  case 2:
    // reading
    arrsize = sizeof(reading_data);
    dataarr = (void *)reading_data;
    pname = "player2";

    break;
  case 3:
    // accept
    // copier.begin(out, mp3_accept);
    choi = random(1, 4);
    Serial.println(choi);
    switch (choi)
    {
    case 1:
      // VOLUME1=0.65;

    //卡
      arrsize = sizeof(accept_data);
      dataarr = (void *)accept_data;
      break;
    case 2:
    //Ciallo～ (∠・ω< )⌒★
      VOLUME1 = 0.9;

      arrsize = sizeof(accept_data_2);
      dataarr = (void *)accept_data_2;
      break;
    case 3:
    //Welcome to Rhinelab LLC, Internal Residence.
      arrsize = sizeof(accept_data_2);
      dataarr = (void *)accept_data_2;
      break;
    default:
      arrsize = sizeof(_data);
      dataarr = (void *)_data;
      break;
    }
    pname = "player3";

    break;
  case 4:
    // denied

    arrsize = sizeof(denied_data);
    dataarr = (void *)denied_data;
    pname = "player4";
    break;
  case 5:
    // readerror

    arrsize = sizeof(readerror_data);
    dataarr = (void *)readerror_data;

    pname = "player5";

    break;
  case 6:
    // lowBAT
    arrsize = sizeof(lowbat_data);
    dataarr = (void *)lowbat_data;
    pname = "player6";

    break;
  case 7:
    // pluslowBAT
    arrsize = sizeof(lowlowbat_data);
    dataarr = (void *)lowlowbat_data;
    pname = "player7";

    break;
  default:

    arrsize = sizeof(_data);
    dataarr = (void *)_data;
    pname = "player0";

    break;
  }

  // 主线程
  xTaskCreatePinnedToCore(
      player,        // 任务函数
      pname.c_str(), // 任务名称
      4096 * 1,      // 堆栈大小（字节）
      dataarr,       // 参数
      3,             // 优先级
      &playerHandle, // 任务句柄
      0              // 核心编号
  );
  // 创建任务
}
// 音源管理
TaskHandle_t playerListHandle = NULL;
unsigned char playlist[20];
unsigned char playlistcount = 0, playlistindex = 0;
void playerList(void *parameter)
{
  isplListrun = true;
  while (playlistcount > 0)
  {
    if (!isplaying)
    {
      vTaskDelay(pdMS_TO_TICKS(100));
      switchaudio(playlist[playlistindex]);
      playlistindex++;
      playlistcount--;
      Serial.print("播放列表:");
      Serial.print(playlistindex);
      Serial.print("/");
      Serial.println(playlistcount);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  isplListrun = false;

  vTaskDelete(NULL);
}
// 添加播放任务到列表
void addTolist(unsigned int in)
{
  playlist[playlistindex + playlistcount] = in;
  playlistcount++;
  if (!isplListrun)
  {
    xTaskCreatePinnedToCore(
        playerList,        // 任务函数
        "playerlist1",     // 任务名称
        1024,              // 堆栈大小（字节）
        NULL,              // 参数
        1,                 // 优先级
        &playerListHandle, // 任务句柄
        0                  // 核心编号
    );
  }
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

  // // 舵机位置范围限制
  // const uint16_t MIN_POSITION = 0;      // 最低点
  // const uint16_t MAX_POSITION = 0x0500; // 最高点（1280）

  // // 限制位置值在有效范围内
  // if (position < MIN_POSITION)
  // {
  //   position = MIN_POSITION;
  // }
  // else if (position > MAX_POSITION)
  // {
  //   position = MAX_POSITION;
  // }

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

  // 可选：调试输出
  // Serial.print("位置: ");
  // Serial.print(position);
  // Serial.print(" (0x");
  // Serial.print(position, HEX);
  // Serial.println(")");

  // Serial.print("指令: FF FF ");
  // for (int i = 0; i < 6; i++) {
  //   if (data[i] < 0x10) Serial.print("0");
  //   Serial.print(data[i], HEX);
  //   Serial.print(" ");
  // }
  // if (checksum < 0x10) Serial.print("0");
  // Serial.println(checksum, HEX);
}

// 舵机动作
bool isservobusy = false;
void Switchlock()
{

  isservobusy = true;

  // UNLOCKING
  sendServoPosition(800);

  // au:accept
  addTolist(3);
  vTaskDelay(pdMS_TO_TICKS(10));

  vTaskDelay(pdMS_TO_TICKS(2000));

  // LOCKING
  sendServoPosition(1180);
  vTaskDelay(pdMS_TO_TICKS(1000));

  Serial.println("Servo done");

  isservobusy = false;
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
  const unsigned long TIMEOUT_MS = 1000; // 超时时长

  NFCcard readdata;
  readdata.uidLength = 0; // 初始化为无效状态

  // 超时检查：如果已经开始接收帧但超过设定时间没有收到完整数据，则重置状态
  if (frameStarted && (millis() - lastReceiveTime > TIMEOUT_MS))
  {
    Serial.println("接收超时，重置接收状态");
    frameStarted = false;
    bufferIndex = 0;
    return readdata;
  }

  while (Serial1.available() > 0)
  {
    uint8_t incomingByte = Serial1.read();
    lastReceiveTime = millis(); // 更新最后接收时间

    Serial.print("收到字节: 0x");
    Serial.println(incomingByte, HEX);

    // 寻找帧起始符 0x20
    if (!frameStarted && incomingByte == 0x20)
    {
      Serial.println("检测到帧起始符 0x20 开始接收帧");
      frameStarted = true;
      bufferIndex = 0;
      rxBuffer[bufferIndex++] = incomingByte;
      lastReceiveTime = millis(); // 开始接收时记录时间
    }
    // 如果已经开始接收帧
    else if (frameStarted)
    {
      rxBuffer[bufferIndex++] = incomingByte;
      // Serial.print("缓冲索引: ");
      // Serial.println(bufferIndex);

      // 检查是否收到完整的帧 (14字节)
      if (bufferIndex >= 14)
      {
        Serial.println("收到完整帧(14字节)，开始解析");

        // 检查帧结构是否正确 (起始符和结束符)
        if (rxBuffer[0] != 0x20 || rxBuffer[13] != 0x03)
        {
          Serial.println("帧结构错误: 起始符或结束符不正确");
          Serial.print("起始符: 0x");
          Serial.print(rxBuffer[0], HEX);
          Serial.print(", 结束符: 0x");
          Serial.println(rxBuffer[13], HEX);

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

        // Serial.print("计算校验和: 0x");
        // Serial.print(checksum, HEX);
        // Serial.print(", 接收校验和: 0x");
        // Serial.println(rxBuffer[12], HEX);

        if (checksum != rxBuffer[12])
        {
          Serial.println("校验和错误");
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

        Serial.print("卡片序列号: ");
        Serial.println(serialStr);

        // 打印完整帧数据用于调试
        // Serial.print("完整帧数据: ");
        // for (int i = 0; i < 14; i++)
        // {
        //   Serial.print("0x");
        //   Serial.print(rxBuffer[i], HEX);
        //   Serial.print(" ");
        // }
        // Serial.println();

        frameStarted = false;
        bufferIndex = 0;
        Serial.println("帧处理完成");
        return readdata;
      }

      // 防止缓冲区溢出
      if (bufferIndex >= sizeof(rxBuffer))
      {
        Serial.println("缓冲区溢出，重置接收状态");
        frameStarted = false;
        bufferIndex = 0;
        readdata.uidLength = 0;
        return readdata;
      }
    }
    else
    {
      Serial.println("等待帧起始符...");
    }
  }

  // 没有读取到完整数据时返回空数据
  if (frameStarted)
  {
    Serial.print("正在接收数据，当前已接收 ");
    Serial.print(bufferIndex);
    Serial.println(" 字节");
  }

  readdata.uidLength = 0;
  return readdata;
}

// 读卡指令
void sendCardSearchCommand()
{

  Serial1.read();
  // 寻卡指令
  uint8_t cardSearchCmd[] = {0x20, 0x00, 0x27, 0x00, 0xD8, 0x03};

  // 通过Serial1发送指令
  Serial1.write(cardSearchCmd, sizeof(cardSearchCmd));

  // Serial.print("发送寻卡指令: ");
  // for (int i = 0; i < sizeof(cardSearchCmd); i++)
  // {
  //   Serial.print("0x");
  //   if (cardSearchCmd[i] < 0x10)
  //     Serial.print("0");
  //   Serial.print(cardSearchCmd[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  vTaskDelay(pdMS_TO_TICKS(100));

  // 检查是否有返回数据
  // if(Serial1.available()) {
  //   Serial.print("收到响应: ");
  //   while(Serial1.available()) {
  //     uint8_t response = Serial1.read();
  //     Serial.print("0x");
  //     if(response < 0x10) Serial.print("0");
  //     Serial.print(response, HEX);
  //     Serial.print(" ");
  //   }
  //   Serial.println();
  // }
}

void setup()
{
  // 低功耗
  // setCpuFrequencyMhz(80);

  // 初始化调试串口
  Serial.begin(115200);
  AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Warning);

  // 初始化 UART1
  Serial1.begin(UART_reader_BAUDRATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_reader_PIN);

  // 初始化ADC1
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTEN);
  // 字符化ADC用于电压转换
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
      ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  // 初始化LED
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  leds[0] = CRGB::White;
  FastLED.show();

  // 初始化5V电源控制
  pinMode(EN_5V, OUTPUT);
  digitalWrite(EN_5V, HIGH);

  // 初始化唤醒中断
  pinMode(IRQ, INPUT_PULLDOWN);
  gpio_wakeup_enable((gpio_num_t)IRQ, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  delay(1000);

  // au:ready
  addTolist(1);
  delay(2000);
  digitalWrite(EN_5V, LOW);
  delay(1000);

  Serial.println("Ready!");
  leds[0] = CRGB::Black;
  FastLED.show();
}

void loop()
{

  // 进入浅睡眠
  Serial.println("Sleeping!");
  vTaskDelay(pdMS_TO_TICKS(100));
  esp_light_sleep_start();
  Serial.println("Wake up!");

  digitalWrite(EN_5V, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));

  // 清空缓冲区
  sendCardSearchCommand();
  sendCardSearchCommand();

  // au:wait
  addTolist(2);
  vTaskDelay(pdMS_TO_TICKS(10));

  // 检查 UART1 是否有数据可读
  if (Serial1.available() > 0)
  {
    Serial.println("--------------------");
    Serial.println("Card detected");

    // 读取标签
    NFCcard currentcard;
    currentcard = ReadCard();

    // 卡数据有效检查
    if (currentcard.uidLength != 0)
    {
      if (isCardAuthorized(currentcard, authorizedCards, Cardscount))
      {
        // 匹配
        Serial.println("ACCEPTED");

        // 切换舵机通信

        Serial1.end();
        vTaskDelay(pdMS_TO_TICKS(50));
        Serial1.begin(UART_servo_BAUDRATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_servo_PIN);
        vTaskDelay(pdMS_TO_TICKS(300));

        leds[0] = CRGB::Green;
        FastLED.show();

        // 舵机动作
        if (!isservobusy)
        {
          Switchlock();
        }
      }
      else
      {
        // 不匹配
        leds[0] = CRGB::Red;
        FastLED.show();
        Serial.println("DENIED");

        // au:denied
        addTolist(4);
      }
    }
    else
    {
      // 卡数据无效
      leds[0] = CRGB::Yellow;
      FastLED.show();

      // au:readerror
      addTolist(5);

      Serial.println("NO DATA");
      vTaskDelay(pdMS_TO_TICKS(1000));
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

    leds[0] = CRGB::Black;
    FastLED.show();
    Serial.println("--------------------");
  }

  // 低电量提醒
  float voltage = read_battery_voltage();
  Serial.print("BAT voltage: ");
  Serial.println(voltage);
  if (voltage <= 3.4 and voltage > 3.2)
  {
    leds[0] = CRGB::Red;
    // au:lowbat
    addTolist(6);
    leds[0] = CRGB::Black;
    FastLED.show();
  }
  else if (voltage <= 3.2)
  {
    leds[0] = CRGB::Red;
    // au:lowlowbat
    addTolist(7);
    leds[0] = CRGB::Black;
    FastLED.show();
  }

  vTaskDelay(pdMS_TO_TICKS(3000));

  digitalWrite(EN_5V, LOW);

  vTaskDelay(pdMS_TO_TICKS(100));
}
