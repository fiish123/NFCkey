#include "audio_i2s_out.h"

#include <Arduino.h>

#include "driver/i2s.h"
#include "esp_err.h"

#include "logger.h"

namespace audio {
namespace {

// Matches the AudioTools defaults that produced the known-good output:
// I2S_BUFFER_COUNT = 6 descriptors of I2S_BUFFER_SIZE = 512 frames.
constexpr int kBufferCount = 6;
constexpr int kBufferLen = 512;

// i2s_write() is given this many bytes per call so a stop request can be
// observed with a bounded latency instead of blocking for a whole frame.
constexpr size_t kWriteSlice = 2048;
constexpr TickType_t kWriteTimeout = pdMS_TO_TICKS(200);

bool s_open = false;
int s_sampleRate = 0;

}  // namespace

bool i2sOpen(int pinBclk, int pinLrc, int pinData, int sampleRate, int channels) {
  if (s_open) i2sClose();

  if (sampleRate <= 0) {
    LOG_E("I2S采样率无效: %d", sampleRate);
    return false;
  }
  if (channels != 1 && channels != 2) {
    LOG_E("I2S声道数无效: %d", channels);
    return false;
  }

  i2s_config_t cfg = {};
  // Master TX. RIGHT_LEFT slot layout means the driver sends the stereo pair we
  // hand it; mono sources are expanded by the caller.
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = (uint32_t)sampleRate;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = kBufferCount;
  cfg.dma_buf_len = kBufferLen;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;  // avoids noise when no data is available
  cfg.fixed_mclk = 0;
  cfg.mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT;
  cfg.bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT;

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
  if (err != ESP_OK) {
    LOG_E("I2S驱动安装失败: %s", esp_err_to_name(err));
    return false;
  }

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = pinBclk;
  pins.ws_io_num = pinLrc;
  pins.data_out_num = pinData;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  err = i2s_set_pin(I2S_NUM_0, &pins);
  if (err != ESP_OK) {
    LOG_E("I2S引脚配置失败: %s", esp_err_to_name(err));
    i2s_driver_uninstall(I2S_NUM_0);
    return false;
  }

  // Start from silence instead of whatever the descriptor memory held.
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_start(I2S_NUM_0);

  s_open = true;
  s_sampleRate = sampleRate;
  LOG_I("I2S输出已开启: %d Hz, %d 声道源, BCK=%d LRC=%d DIN=%d", sampleRate, channels,
        pinBclk, pinLrc, pinData);
  return true;
}

void i2sClose() {
  if (!s_open) return;
  i2s_stop(I2S_NUM_0);
  i2s_driver_uninstall(I2S_NUM_0);
  s_open = false;
  s_sampleRate = 0;
  LOG_D("I2S输出已关闭");
}

bool i2sIsOpen() { return s_open; }

int i2sSampleRate() { return s_sampleRate; }

bool i2sSetSampleRate(int sampleRate) {
  if (!s_open || sampleRate <= 0 || sampleRate == s_sampleRate) return true;

  // Stop the transmitter, reconfigure and restart. Calling this before the
  // first write of a prompt is safe: nothing was queued for the old rate.
  i2s_stop(I2S_NUM_0);
  if (i2s_set_sample_rates(I2S_NUM_0, (uint32_t)sampleRate) != ESP_OK) {
    LOG_W("I2S采样率调整失败: %d Hz", sampleRate);
    i2s_start(I2S_NUM_0);
    return false;
  }
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_start(I2S_NUM_0);
  s_sampleRate = sampleRate;
  LOG_I("I2S采样率调整为 %d Hz", sampleRate);
  return true;
}

bool i2sWrite(const int16_t *stereoFrames, size_t frames) {
  if (!s_open || stereoFrames == nullptr || frames == 0) return false;

  const char *bytes = reinterpret_cast<const char *>(stereoFrames);
  size_t total = frames * 2 * sizeof(int16_t);
  size_t written = 0;

  while (written < total) {
    size_t slice = total - written;
    if (slice > kWriteSlice) slice = kWriteSlice;

    size_t done = 0;
    esp_err_t err = i2s_write(I2S_NUM_0, bytes + written, slice, &done, kWriteTimeout);
    if (err != ESP_OK) {
      LOG_W("I2S写入失败: %s", esp_err_to_name(err));
      return false;
    }
    // A zero-length write means the driver is shutting down; bail out instead
    // of spinning. The caller stops on the next isPlaying() check.
    if (done == 0) return false;
    written += done;
  }
  return true;
}

}  // namespace audio