#include <Arduino.h>
#include <vector>
#include <FastLED.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

// UART1配置
#define UART1_RX_PIN 19
#define UART1_TX_PIN 18
#define UART_BAUDRATE 9600

// 唤醒中断
#define IRQ 4

// 震动
#define vibmotor 10

// 舵机
#define servopin 3//2（pcb ）1（mn）

// LED参数
#define NUM_LEDS 1   // LED数量
#define DATA_PIN 8   // 数据引脚
CRGB leds[NUM_LEDS]; // LED数组

// uart通信
uint8_t rxBuffer[20];
uint8_t bufferIndex = 0;
bool frameStarted = false;
unsigned long lastReceiveTime = 0;  // 用于超时机制
const unsigned long TIMEOUT_MS = 100;  // 100ms超时

// ADC配置
#define BATTERY_ADC_CHANNEL    ADC1_CHANNEL_1
#define ADC_ATTEN             ADC_ATTEN_DB_12  // 测量范围0-3.1V
#define VOLTAGE_DIVIDER_RATIO  1.4545f            // 分压比 (R1+R2)/R2

static esp_adc_cal_characteristics_t adc_chars;

float read_battery_voltage(void)
{
    uint32_t adc_reading = 0;
    
    // 采集16次取平均值
    for (int i = 0; i < 16; i++) {
        adc_reading += adc1_get_raw(BATTERY_ADC_CHANNEL);
    }
    adc_reading /= 16;
    
    // 将ADC值转换为电压(mV)
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
    
    // 计算实际电池电压 = 分压后电压 × 分压比
    float battery_voltage = (voltage_mv / 1000.0f) * VOLTAGE_DIVIDER_RATIO;
    
    return battery_voltage;
}

// 震动动作
void VIBaction(int func)
{
  switch (func)
  {
  case 1:
    digitalWrite(vibmotor, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(vibmotor, LOW);
    break;
  case 2:
    digitalWrite(vibmotor, HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(vibmotor, LOW);
    break;
  case 3:
    digitalWrite(vibmotor, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(vibmotor, LOW);
    vTaskDelay(pdMS_TO_TICKS(50));
    digitalWrite(vibmotor, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(vibmotor, LOW);
    break;
  case 4:
    digitalWrite(vibmotor, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(vibmotor, LOW);
    break;
  case 5:
    digitalWrite(vibmotor, HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(vibmotor, LOW);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(vibmotor, HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(vibmotor, LOW);
        digitalWrite(vibmotor, HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(vibmotor, LOW);
    break;
  case 6:
    digitalWrite(vibmotor, HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(vibmotor, LOW);
    vTaskDelay(pdMS_TO_TICKS(100));
    break;
  default:
    break;
  }
}

// 读卡提醒
// void vib_1(void *parameter)
// {
//   while (1)
//   {
//     digitalWrite(vibmotor, HIGH);
//     vTaskDelay(pdMS_TO_TICKS(100));
//     digitalWrite(vibmotor, LOW);
//     vTaskDelay(pdTICKS_TO_MS(900));
//   }
// }
// TaskHandle_t vib1 = NULL;

// 舵机动作时序
struct CRD
{

  // 0:free 1:busy
  bool servostatus = 0;
  // 0:lock 1:unlock
  bool keystatus = 0;

  //(ms)
  unsigned int opentime_1 = 900;
  unsigned int opentime_2 = 1000;
  unsigned int closetime_1 = 700;
  unsigned int closetime_2 = 50;
  //(ms)
  unsigned int openspeed_1 = 0;
  unsigned int openspeed_2 = 83;
  unsigned int closespeed_1 = 180;
  unsigned int closespeed_2 = 97;
};
CRD Data0;

// 定义舵机配置结构体
struct ServoConfig
{
  int pin;        // GPIO引脚
  int channel;    // PWM通道(0-15)
  int freq;       // PWM频率(Hz)
  int resolution; // PWM分辨率(位数)
  int minPulse;   // 最小脉冲宽度(微秒)
  int maxPulse;   // 最大脉冲宽度(微秒)
  int minAngle;   // 最小角度
  int maxAngle;   // 最大角度
};
const ServoConfig servo = {
    .pin = 0,         // 使用GPIO0控制舵机
    .channel = 0,     // PWM通道0
    .freq = 50,       // 标准50Hz
    .resolution = 12, // 12位分辨率
    .minPulse = 500,  // 0度脉冲宽度
    .maxPulse = 2500, // 180度脉冲宽度
    .minAngle = 0,    // 最小角度0度
    .maxAngle = 180   // 最大角度180度
};

// 舵机控制器类
class ServoController
{
private:
  ServoConfig config;
  const int maxDuty;
  const long pwmPeriod;

public:
  ServoController(const ServoConfig &cfg)
      : config(cfg), maxDuty((1 << config.resolution) - 1), pwmPeriod(1000000 / config.freq)
  {
    // 构造函数体
  }

  void begin()
  {
    ledcSetup(config.channel, config.freq, config.resolution);
    ledcAttachPin(config.pin, config.channel);
    Serial.println("舵机控制器初始化完成");
  }

  int angleToDuty(int angle)
  {
    angle = constrain(angle, config.minAngle, config.maxAngle);
    int pulseWidth = map(angle, config.minAngle, config.maxAngle,
                         config.minPulse, config.maxPulse);
    return (pulseWidth * maxDuty) / pwmPeriod;
  }

  void setAngle(int angle)
  {
    int duty = angleToDuty(angle);
    ledcWrite(config.channel, duty);
    Serial.print("角度: ");
    Serial.print(angle);
    Serial.print("°, 占空比: ");
    Serial.println(duty);
  }
};
ServoController servoCtrl(servo);

// 舵机动作
void Switchlock()
{

  Data0.servostatus = 1;

  // UNLOCKING

  // P1
  servoCtrl.setAngle(Data0.openspeed_1);
  vTaskDelay(pdMS_TO_TICKS(Data0.opentime_1));
  // P2
  servoCtrl.setAngle(Data0.openspeed_2);
  vTaskDelay(pdMS_TO_TICKS(Data0.opentime_2));

  // LOCKING

  // P1
  servoCtrl.setAngle(Data0.closespeed_1);
  vTaskDelay(pdMS_TO_TICKS(Data0.closetime_1));
  // P2
  servoCtrl.setAngle(Data0.closespeed_2);
  vTaskDelay(pdMS_TO_TICKS(Data0.closetime_2));
  servoCtrl.setAngle(90);
  Serial.println("Servo done");

  VIBaction(6);

  Data0.servostatus = 0;
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
bool isCardAuthorized(const NFCcard &currentCard,const NFCcard authorizedList[], const int listSize)
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
  NFCcard readdata; // 在函数开头定义一次
  readdata.uidLength = 0; // 初始化为无效状态
  
  // 超时检查：如果已经开始接收帧但超过设定时间没有收到完整数据，则重置状态
  if (frameStarted && (millis() - lastReceiveTime > TIMEOUT_MS)) {
    Serial.println("⚠️ 接收超时，重置接收状态");
    frameStarted = false;
    bufferIndex = 0;
    return readdata;
  }
  
  while (Serial1.available() > 0)
  {
    uint8_t incomingByte = Serial1.read();
    lastReceiveTime = millis();  // 更新最后接收时间

    Serial.print("收到字节: 0x");
    Serial.println(incomingByte, HEX);

    // 寻找帧起始符 0x20
    if (!frameStarted && incomingByte == 0x20)
    {
      Serial.println("🔍 检测到帧起始符 0x20，开始接收帧");
      frameStarted = true;
      bufferIndex = 0;
      rxBuffer[bufferIndex++] = incomingByte;
      lastReceiveTime = millis();  // 开始接收时记录时间
    }
    // 如果已经开始接收帧
    else if (frameStarted)
    {
      rxBuffer[bufferIndex++] = incomingByte;
      Serial.print("📥 缓冲索引: ");
      Serial.println(bufferIndex);

      // 检查是否收到完整的帧 (14字节)
      if (bufferIndex >= 14)
      {
        Serial.println("✅ 收到完整帧(14字节)，开始解析");
        
        // 检查帧结构是否正确 (起始符和结束符)
        if (rxBuffer[0] != 0x20 || rxBuffer[13] != 0x03)
        {
          Serial.println("❌ 帧结构错误: 起始符或结束符不正确");
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

        Serial.print("🔢 计算校验和: 0x");
        Serial.print(checksum, HEX);
        Serial.print(", 接收校验和: 0x");
        Serial.println(rxBuffer[12], HEX);

        if (checksum != rxBuffer[12])
        {
          Serial.println("❌ 校验和错误");
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
        sprintf(serialStr, "%02X%02X%02X%02X",
                readdata.uid[0], readdata.uid[1], readdata.uid[2], readdata.uid[3]);

        Serial.print("🎫 卡片序列号: ");
        Serial.println(serialStr);

        // 打印完整帧数据用于调试
        Serial.print("📋 完整帧数据: ");
        for (int i = 0; i < 14; i++) {
          Serial.print("0x");
          Serial.print(rxBuffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();

        frameStarted = false;
        bufferIndex = 0;
        Serial.println("✅ 帧处理完成");
        return readdata;
      }

      // 防止缓冲区溢出
      if (bufferIndex >= sizeof(rxBuffer))
      {
        Serial.println("⚠️ 缓冲区溢出，重置接收状态");
        frameStarted = false;
        bufferIndex = 0;
        readdata.uidLength = 0;
        return readdata;
      }
    }
    else
    {
      Serial.println("⏳ 等待帧起始符...");
    }
  }
  
  // 没有读取到完整数据时返回空数据
  if (frameStarted) {
    Serial.print("⏳ 正在接收数据，当前已接收 ");
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
  // 硬编码寻卡指令
  uint8_t cardSearchCmd[] = {0x20, 0x00, 0x27, 0x00, 0xD8, 0x03};

  // 通过Serial1发送指令
  Serial1.write(cardSearchCmd, sizeof(cardSearchCmd));

  Serial.print("发送寻卡指令: ");
  for (int i = 0; i < sizeof(cardSearchCmd); i++)
  {
    Serial.print("0x");
    if (cardSearchCmd[i] < 0x10)
      Serial.print("0");
    Serial.print(cardSearchCmd[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

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

//关闭指令
void sendCardResetCommand()
{

  // 指令
  uint8_t cardSearchCmd[] = {0x20,0x00,0x3A,0x00,0xC5,0x03};

  // 通过Serial1发送指令
  Serial1.write(cardSearchCmd, sizeof(cardSearchCmd));

  Serial.print("发送指令: ");
  for (int i = 0; i < sizeof(cardSearchCmd); i++)
  {
    Serial.print("0x");
    if (cardSearchCmd[i] < 0x10)
      Serial.print("0");
    Serial.print(cardSearchCmd[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  vTaskDelay(pdMS_TO_TICKS(100));
}


void setup()
{
  // 低功耗
  setCpuFrequencyMhz(80);

  // 初始化调试串口
  Serial.begin(115200);

  // 初始化 UART1
  Serial1.begin(UART_BAUDRATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);

      // 初始化ADC1
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTEN);
    
    // 字符化ADC用于电压转换
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
        ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  // 初始化LED
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(200);
  leds[0] = CRGB::White;
  FastLED.show();

  // 初始化舵机控制
  servoCtrl.begin();
  servoCtrl.setAngle(90);
  pinMode(servopin, OUTPUT);
  //digitalWrite(servopin, HIGH);
 // Switchlock();
  digitalWrite(servopin, LOW);

  // 初始化震动
  pinMode(vibmotor, OUTPUT);
  VIBaction(1);

  // 初始化中断
  pinMode(IRQ, INPUT);
  gpio_wakeup_enable((gpio_num_t)IRQ, GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  Serial.println("Ready! Bring NFC tag close to reader...");
  delay(1000);
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
 
  // 清空缓冲区
  //sendCardResetCommand();
  sendCardSearchCommand();
  sendCardSearchCommand();

  VIBaction(2);
  vTaskDelay(pdMS_TO_TICKS(100));

  // 检查 UART1 是否有数据可读
  if (Serial1.available() > 0)
  {
    Serial.println("--------------------");
    Serial.println("Card detected");

    // 读取标签
    NFCcard currentcard;
    currentcard = ReadCard();

    if (currentcard.uidLength != 0)
    {
      if (isCardAuthorized(currentcard, authorizedCards, Cardscount))
      {
        Serial.println("ACCEPTED");

        digitalWrite(servopin, HIGH);

        leds[0] = CRGB::Green;
        FastLED.show();

        VIBaction(3);

        // 舵机动作
        if (Data0.servostatus == 0)
        {
          Switchlock();
        }
      }
      else
      {
        leds[0] = CRGB::Red;
        FastLED.show();
        Serial.println("DENIED");

        VIBaction(4);
      }
    }
    else
    {

      leds[0] = CRGB::Yellow;
      FastLED.show();

      VIBaction(5);

      Serial.println("NO DATA");
    }

    // 等待舵机完成动作
    while (Data0.servostatus == 1)
    {
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    digitalWrite(servopin, LOW);

    vTaskDelay(pdMS_TO_TICKS(1000));

    // 关闭状态指示灯
    leds[0] = CRGB::Black;
    FastLED.show();
    Serial.println("--------------------");
  }


  Serial.println(read_battery_voltage());

  //vTaskDelay(pdMS_TO_TICKS(1000));
}
