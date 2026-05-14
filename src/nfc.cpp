#include "nfc.h"
#include "logger.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

// Forward declarations (defined in main.cpp)
void switchconnect(uint8_t in);

// 授权卡片列表（动态数组）
std::vector<NFCcard> authorizedCards;

// 卡片文件路径
const char *CARDS_FILE_PATH = "/cards.json";

// 从文件加载卡片数据
bool loadCardsDataFromFile()
{
    if (!LittleFS.exists(CARDS_FILE_PATH))
    {
        LOG_D("卡片配置文件不存在，创建默认文件");

        // 创建空卡片数据
        JsonDocument doc;
        JsonArray cardsArray = doc["cards"].to<JsonArray>();

        // 保存空数组到文件
        File file = LittleFS.open(CARDS_FILE_PATH, "w");
        if (!file)
        {
            LOG_E("创建卡片配置文件失败");
            return false;
        }

        serializeJson(doc, file);
        file.close();

        LOG_D("卡片配置文件创建成功");
    }

    // 读取文件
    File file = LittleFS.open(CARDS_FILE_PATH, "r");
    if (!file)
    {
        LOG_E("打开卡片配置文件失败");
        return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error)
    {
        LOG_E("卡片配置JSON解析失败: %s", error.c_str());
        return false;
    }

    // 清空当前列表
    authorizedCards.clear();

    // 解析卡片数据
    JsonArray cardsArray = doc["cards"];
    for (JsonObject cardObj : cardsArray)
    {
        NFCcard card;
        card.uidLength = cardObj["length"];
        card.name = cardObj["name"] | ""; // 获取名字段，如果不存在则为空字符串

        String uidStr = cardObj["uid"];
        if (uidStr.length() >= 8)
        {
            // 将十六进制字符串转换为字节数组
            for (int i = 0; i < 4; i++)
            {
                String byteStr = uidStr.substring(i * 2, i * 2 + 2);
                card.uid[i] = (uint8_t)strtol(byteStr.c_str(), NULL, 16);
            }

            authorizedCards.push_back(card);
        }
    }

    LOG_D("授权卡片加载完成: %d 张", authorizedCards.size());
    if (authorizedCards.size() <= 0)
    {
        return false;
    }

    return true;
}

// 保存卡片数据到文件
bool saveCardsToFile()
{
    JsonDocument doc;
    JsonArray cardsArray = doc["cards"].to<JsonArray>();

    for (const auto &card : authorizedCards)
    {
        JsonObject cardObj = cardsArray.add<JsonObject>();
        cardObj["length"] = card.uidLength;
        cardObj["name"] = card.name; // 保存名字字段

        // 将字节数组转换为十六进制字符串
        char uidStr[9];
        sprintf(uidStr, "%02X%02X%02X%02X", card.uid[0], card.uid[1], card.uid[2], card.uid[3]);
        cardObj["uid"] = uidStr;
    }

    File file = LittleFS.open(CARDS_FILE_PATH, "w");
    if (!file)
    {
        LOG_E("打开卡片配置文件失败");
        return false;
    }

    serializeJson(doc, file);
    file.close();

    LOG_I("授权卡片数据保存成功");
    return true;
}

// 匹配卡片
bool isCardAuthorized(const NFCcard &currentCard)
{
    // 遍历授权列表中的每一张卡
    for (size_t i = 0; i < authorizedCards.size(); i++)
    {
        // 先检查长度，长度不同则直接跳过
        if (currentCard.uidLength != authorizedCards[i].uidLength)
        {
            continue;
        }

        // 长度相同，再逐字节比较UID
        bool isMatch = true;
        for (int j = 0; j < currentCard.uidLength; j++)
        {
            if (currentCard.uid[j] != authorizedCards[i].uid[j])
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
    unsigned long frameStartTime = 0;
    unsigned long lastReceiveTime = 0;
    const unsigned long TIMEOUT_MS = 100; // 超时时长

    NFCcard readdata;
    readdata.uidLength = 0; // 初始化为无效状态

    while (1)
    {
        if (Serial1.available() <= 0)
        {
            if (frameStarted && (millis() - lastReceiveTime > TIMEOUT_MS))
            {
                LOG_W("NFC数据帧接收超时: 已接收=%u字节, 持续=%lu ms", bufferIndex, millis() - frameStartTime);
                frameStarted = false;
                bufferIndex = 0;
                readdata.uidLength = 0;
                return readdata;
            }

            if (!frameStarted)
            {
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint8_t incomingByte = Serial1.read();
        lastReceiveTime = millis(); // 更新最后接收时间

        // 寻找帧起始符 0x20
        if (!frameStarted && incomingByte == 0x20)
        {

            frameStarted = true;
            bufferIndex = 0;
            rxBuffer[bufferIndex++] = incomingByte;
            frameStartTime = lastReceiveTime;
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

                    LOG_W("NFC数据帧格式错误: 起始=0x%X, 结束=0x%X", rxBuffer[0], rxBuffer[13]);

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
                    LOG_W("NFC数据帧校验失败");
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

                frameStarted = false;
                bufferIndex = 0;
                return readdata;
            }

            // 防止缓冲区溢出
            if (bufferIndex >= sizeof(rxBuffer))
            {
                LOG_W("NFC接收缓冲区溢出，重置接收状态");
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
    // 切换通信
    switchconnect(1);

    // 寻卡指令
    uint8_t cardSearchCmd[] = {0x20, 0x00, 0x27, 0x00, 0xD8, 0x03};

    // 等待读卡器初始化
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

    if (Serial1.available() < 14)
    {
        LOG_W("NFC寻卡响应超时: 等待=%lu ms, 已接收=%d/14字节", now - last, Serial1.available());
    }
}
