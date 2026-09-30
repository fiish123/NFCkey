#include "audio_player.h"

#include <Arduino.h>
#include <LittleFS.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "audio_aac_decoder.h"
#include "audio_i2s_out.h"
#include "audio_volume.h"
#include "logger.h"

namespace audio {
namespace {

// ----------------------------------------------------------------------
//  Tunables
// ----------------------------------------------------------------------

constexpr int kQueueDepth = 24;           // queued prompts; the old playlist held 20
constexpr int kReadChunk = 1024;          // AAC bytes pulled from LittleFS per read
constexpr int kDefaultSampleRate = 44100; // matches the prompt files
constexpr int kTaskStack = 5120;
constexpr UBaseType_t kTaskPriority = 4;  // same as the old playerList task
constexpr uint32_t kStopTimeoutMs = 2000; // bounded stop latency for sleep
// Enough to flush the DMA queue (6 x 512 frames at 44.1 kHz is ~70 ms) so the
// tail of a prompt is heard before the port is closed.
constexpr uint32_t kDrainDelayMs = 120;

// ----------------------------------------------------------------------
//  State
// ----------------------------------------------------------------------

struct Item {
  const char *path;  // string literal owned by the caller, e.g. getAudioPath()
  float volume;      // 0.0 .. 1.0
};

QueueHandle_t s_queue = nullptr;
SemaphoreHandle_t s_stateLock = nullptr;
TaskHandle_t s_task = nullptr;
PowerHook s_powerHook = nullptr;

volatile bool s_playing = false;        // something is queued or playing
volatile bool s_stopRequested = false;  // set by beforeLightSleep() / end()
volatile bool s_outputActive = false;   // I2S + decoder + analog rail are live
volatile bool s_decoderFailed = false;

int s_pinBclk = -1, s_pinLrc = -1, s_pinData = -1;

// Owned by the playback task. Sized from the trimmed decoder limits
// (1024 samples * 2 channels) so any decoded frame always fits: s_pcm holds up
// to 2048 interleaved samples, s_stereo the same number of duplicated frames
// (2 slots each).
AacDecoder s_decoder;
int16_t s_pcm[2048];
int16_t s_stereo[4096];

// ----------------------------------------------------------------------
//  Output lifecycle
// ----------------------------------------------------------------------

void startOutput() {
  s_outputActive = true;

  // If a power hook is installed, switch the DAC and the shared 5V rail on
  // before the decoder allocates, so a failed allocation still leaves the rail
  // in a defined state.
  if (s_powerHook != nullptr) s_powerHook(true);

  s_decoderFailed = !s_decoder.begin();

  if (!i2sOpen(s_pinBclk, s_pinLrc, s_pinData, kDefaultSampleRate, 1)) {
    LOG_E("I2S输出开启失败");
  }
}

void stopOutput() {
  i2sClose();
  s_decoder.end();
  if (s_powerHook != nullptr) s_powerHook(false);
  s_outputActive = false;
}

/// Decodes and plays one prompt file. Sets @p wasStopped when interrupted.
void playFile(const Item &item, bool &wasStopped) {
  wasStopped = false;

  File file = LittleFS.open(item.path, "r");
  if (!file) {
    LOG_W("音频文件打开失败: %s", item.path);
    return;
  }

  LOG_I("开始播放音频: %s", item.path);

  const float gain = volumeFactor(item.volume);
  int fadeInFrames = kFadeFrames;

  // One frame of look-ahead: a decoded frame is only written once the following
  // frame is available, so the tail can be faded out instead of cut off.
  bool haveFrame = false;
  bool wroteToPort = false;
  int frameSamples = 0;
  int frameChannels = 1;

  bool eof = false;
  uint8_t chunk[kReadChunk];

  for (;;) {
    if (s_stopRequested) {
      wasStopped = true;
      haveFrame = false;
      break;
    }

    // Keep the decoder staging buffer topped up. Reading while bytes are still
    // buffered is intentional: a single ADTS frame is larger than one read.
    //
    // Never read more than the staging buffer can take: feed() keeps at most
    // space() bytes and drops the rest of the chunk. Those lost bytes would
    // punch a hole into the AAC stream, after which the decoder resyncs onto
    // garbage and playback dies after the first few frames.
    int room = s_decoder.space();
    if (!eof && room > 0) {
      if (room > kReadChunk) room = kReadChunk;
      size_t got = file.read(chunk, (size_t)room);
      if (got > 0) {
        s_decoder.feed(chunk, (int)got);
      } else {
        eof = true;
      }
    }

    int samples = 0, channels = 1, sampleRate = 0;
    int rc = s_decoder.decodeFrame(&samples, &channels, &sampleRate);

    if (rc == 0) {
      // Follow the bitstream rate. Only possible before the first write of this
      // prompt; every shipped file is 44.1 kHz anyway, so this normally only
      // confirms the rate the port was opened with.
      if (!wroteToPort) i2sSetSampleRate(sampleRate);

      if (haveFrame) {
        int frames = applyVolume(s_pcm, frameSamples, frameChannels, gain, false,
                                 fadeInFrames, s_stereo);
        if (!i2sWrite(s_stereo, (size_t)frames)) {
          haveFrame = false;
          break;
        }
        wroteToPort = true;
      }
      memcpy(s_pcm, s_decoder.pcm(), (size_t)samples * sizeof(int16_t));
      frameSamples = samples;
      frameChannels = channels;
      haveFrame = true;
      continue;
    }

    if (rc < 0) {
      // Corrupt data was skipped; only give up once nothing can be recovered.
      if (s_decoder.buffered() > 0 || !eof) continue;
      break;
    }

    // rc > 0: the decoder needs more input.
    if (eof) break;
    if (s_decoder.space() == 0) {
      // The staging buffer is full but still holds no complete frame: the file
      // is corrupt or not ADTS. Give up rather than spin.
      LOG_W("无法从 %s 中解析出完整的AAC帧", item.path);
      break;
    }
  }

  // Fade the last frame out instead of cutting the DAC mid-sample.
  if (haveFrame && !s_stopRequested) {
    int frames = applyVolume(s_pcm, frameSamples, frameChannels, gain, true,
                             fadeInFrames, s_stereo);
    i2sWrite(s_stereo, (size_t)frames);
  }

  // Let the DMA buffers drain before the caller can close the port, so the
  // last samples (and the fade-out) are actually audible.
  if (!s_stopRequested) vTaskDelay(pdMS_TO_TICKS(kDrainDelayMs));

  file.close();
  LOG_D("音频播放完成: %s", item.path);
}

/// Plays every prompt currently queued. Returns true if a stop was requested.
bool playQueued() {
  Item item;
  bool stopped = false;
  while (xQueueReceive(s_queue, &item, 0) == pdTRUE) {
    if (s_stopRequested) {
      stopped = true;
      break;
    }
    if (s_decoderFailed) {
      LOG_W("跳过音频: 解码器不可用");
      continue;
    }
    playFile(item, stopped);
    if (stopped) break;
  }
  return stopped;
}

/// Drops whatever is still queued and powers the output back down.
void drainAndIdle() {
  Item drop;
  while (xQueueReceive(s_queue, &drop, 0) == pdTRUE) {
  }
  xSemaphoreTake(s_stateLock, portMAX_DELAY);
  s_playing = false;
  xSemaphoreGive(s_stateLock);
  stopOutput();
}

/// True when nothing is queued, nothing is being played and the output is
/// powered down. Taken under the lock so an enqueue that happens right now is
/// never mistaken for a finished burst and dropped.
bool isFullyIdle() {
  xSemaphoreTake(s_stateLock, portMAX_DELAY);
  bool idle = uxQueueMessagesWaiting(s_queue) == 0 && !s_playing && !s_outputActive;
  xSemaphoreGive(s_stateLock);
  return idle;
}

/// Retires the burst if the queue is still empty. Checked under the lock so a
/// prompt enqueued at this instant keeps the output alive.
bool queueEmpty() {
  bool empty = false;
  xSemaphoreTake(s_stateLock, portMAX_DELAY);
  if (uxQueueMessagesWaiting(s_queue) == 0 && !s_stopRequested) {
    s_playing = false;
    empty = true;
  }
  xSemaphoreGive(s_stateLock);
  return empty;
}

/// Waits for the idle state, without holding any lock while sleeping.
bool waitUntilIdle(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (!isFullyIdle()) {
    if (millis() - start >= timeoutMs) return false;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  return true;
}

void playbackTask(void *) {
  Item item;
  for (;;) {
    // Idle here: no polling, no DMA, no decoder heap, no powered analog stage.
    // This is the light-sleep safe state the audio module guarantees.
    if (xQueueReceive(s_queue, &item, portMAX_DELAY) != pdTRUE) continue;

    // A sleep request that arrived while this task was blocked must not cause
    // a pointless DAC/I2S power-up.
    if (s_stopRequested) {
      drainAndIdle();
      continue;
    }

    startOutput();
    bool stopped = false;

    if (!s_decoderFailed) {
      playFile(item, stopped);

      // Re-read the queue after every prompt so a burst (for example accept +
      // unlock) plays without power cycling the DAC.
      while (!stopped && !queueEmpty()) {
        stopped = playQueued();
      }
    }

    if (stopped || s_stopRequested) {
      drainAndIdle();
    } else {
      // Nothing left to play. Retire the idle state under the lock so a prompt
      // enqueued at this instant either keeps the output alive or is picked up
      // by the next queue receive without isPlaying() ever reporting false
      // while work is pending. The shutdown itself runs outside the lock
      // because it uninstalls the I2S driver.
      bool idle = false;
      xSemaphoreTake(s_stateLock, portMAX_DELAY);
      if (uxQueueMessagesWaiting(s_queue) == 0 && !s_stopRequested) {
        s_playing = false;
        idle = true;
      }
      xSemaphoreGive(s_stateLock);

      if (idle) stopOutput();
    }
  }
}

}  // namespace

// ----------------------------------------------------------------------
//  Public API
// ----------------------------------------------------------------------

void setPowerHook(PowerHook hook) { s_powerHook = hook; }

bool begin(int pinBclk, int pinLrc, int pinData) {
  if (s_task != nullptr) return true;

  s_pinBclk = pinBclk;
  s_pinLrc = pinLrc;
  s_pinData = pinData;

  s_queue = xQueueCreate(kQueueDepth, sizeof(Item));
  s_stateLock = xSemaphoreCreateMutex();
  if (s_queue == nullptr || s_stateLock == nullptr) {
    LOG_E("音频模块初始化失败: 队列/互斥量分配失败");
    return false;
  }

  BaseType_t created = xTaskCreatePinnedToCore(playbackTask, "audioPlayer",
                                               kTaskStack, nullptr,
                                               kTaskPriority, &s_task, 0);
  if (created != pdPASS) {
    LOG_E("音频播放任务创建失败");
    s_task = nullptr;
    return false;
  }
  return true;
}

void end() {
  s_stopRequested = true;
  // Wait for the queue to be dropped and the output to be powered down before
  // clearing the request, otherwise the task could restart playback with an
  // item that was still queued.
  if (!waitUntilIdle(kStopTimeoutMs)) {
    LOG_W("音频模块未能在超时内停止");
  }
  s_stopRequested = false;
}

void enqueue(const char *path, float volume) {
  if (path == nullptr) return;
  if (s_queue == nullptr) {
    // begin() was not called yet (or failed): dropping this silently used to
    // hide a boot-order bug, so keep the trace.
    LOG_W("音频模块尚未初始化，丢弃提示音: %s", path);
    return;
  }

  Item item{path, volume};
  xSemaphoreTake(s_stateLock, portMAX_DELAY);
  if (xQueueSend(s_queue, &item, 0) == pdTRUE) {
    s_playing = true;
  } else {
    LOG_W("音频队列已满，丢弃提示音: %s", path);
  }
  xSemaphoreGive(s_stateLock);
}

bool isPlaying() { return s_playing || s_outputActive; }

bool waitIdle(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (isPlaying()) {
    if (timeoutMs != 0 && millis() - start >= timeoutMs) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return true;
}

void beforeLightSleep() {
  // The request stays set for the whole sleep window: if a prompt is enqueued
  // from an ISR-driven path the playback task drains it instead of starting the
  // I2S driver while the system is asleep. afterLightSleep() clears it.
  s_stopRequested = true;

  // Wait until the task dropped the queue, closed I2S, released the decoder and
  // switched the DAC rail off.
  if (!waitUntilIdle(kStopTimeoutMs)) {
    LOG_W("音频模块未能在超时内停止，浅睡眠可能受到影响");
  }
}

void afterLightSleep() { s_stopRequested = false; }

int pending() {
  int queued = s_queue == nullptr ? 0 : (int)uxQueueMessagesWaiting(s_queue);
  return queued + (s_outputActive ? 1 : 0);
}

}  // namespace audio