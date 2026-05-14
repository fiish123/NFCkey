#ifndef NFC_MODULE_H
#define NFC_MODULE_H

#include <vector>
#include <Arduino.h>

// NFC标签结构体
struct NFCcard
{
    uint8_t uid[4];
    uint8_t uidLength;
    String name;
};

// 授权卡片列表（动态数组）
extern std::vector<NFCcard> authorizedCards;

// 卡片文件路径
extern const char *CARDS_FILE_PATH;

// 从文件加载卡片数据
bool loadCardsDataFromFile();

// 保存卡片数据到文件
bool saveCardsToFile();

// 匹配卡片
bool isCardAuthorized(const NFCcard &currentCard);

// 读卡函数
NFCcard ReadCard();

// 读卡指令
void sendCardSearchCommand();

#endif // NFC_MODULE_H
