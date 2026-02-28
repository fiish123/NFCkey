#include "AudioTools.h"
#include "AudioTools/Disk/AudioSourceLittleFS.h"
#include "AudioTools/AudioCodecs/CodecMP3Helix.h"

AudioSourceLittleFS source("/", "mp3");
I2SStream i2s;
MP3DecoderHelix decoder;
AudioPlayer player(source, i2s, decoder);

void setup() {
  Serial.begin(115200);
  AudioToolsLogger.begin(Serial, AudioToolsLogLevel::Info);

  // 配置 I2S
  auto cfg = i2s.defaultConfig(TX_MODE);
  i2s.begin(cfg);

  // 初始化播放器（内部也会初始化 source）
  player.begin();

  // 播放指定文件（阻塞直到播放完成）
  if (!player.playPath("/music/my_song.mp3")) {
    Serial.println("文件不存在或播放失败");
  }
}

void loop() {
  // 在阻塞模式下，playPath 会自己调用 copy 直到结束，loop 中无需操作
}