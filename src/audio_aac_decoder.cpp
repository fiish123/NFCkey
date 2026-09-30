#include "audio_aac_decoder.h"

#include <string.h>

#include "logger.h"

extern "C" {
#include "aacdec.h"
}

namespace audio {

namespace {

/// samplesOut / channelsOut / sampleRateOut live in the C AACFrameInfo struct.
struct FrameFormat {
  int samples;
  int channels;
  int sampleRate;
};

FrameFormat readFormat(void *decoder) {
  AACFrameInfo info;
  AACGetLastFrameInfo((HAACDecoder)decoder, &info);
  FrameFormat f;
  f.samples = info.outputSamps;
  f.channels = info.nChans > 0 ? info.nChans : 1;
  f.sampleRate = info.sampRateOut;
  return f;
}

}  // namespace

AacDecoder::~AacDecoder() { end(); }

int AacDecoder::maxSamples() {
  // AAC_MAX_NSAMPS (1024) * AAC_MAX_NCHANS (2); SBR is compiled out, so one
  // decoded frame never exceeds this.
  return (int)(sizeof(((AacDecoder *)nullptr)->pcm_) / sizeof(int16_t));
}

bool AacDecoder::begin() {
  if (decoder_ != nullptr) return true;
  decoder_ = AACInitDecoder();
  pos_ = 0;
  len_ = 0;
  if (decoder_ == nullptr) {
    LOG_E("AAC解码器内存分配失败");
    return false;
  }
  return true;
}

void AacDecoder::end() {
  if (decoder_ != nullptr) {
    AACFreeDecoder((HAACDecoder)decoder_);
    decoder_ = nullptr;
  }
  pos_ = 0;
  len_ = 0;
}

int AacDecoder::feed(const uint8_t *data, int len) {
  if (data == nullptr || len <= 0) return 0;

  if (pos_ > 0) {
    if (len_ > 0) memmove(buf_, buf_ + pos_, (size_t)len_);
    pos_ = 0;
  }

  int space = kBufferBytes - len_;
  if (space <= 0) return 0;
  if (len > space) {
    // The caller handed over more than fits. The tail is dropped, which leaves
    // a hole in the AAC stream, so callers must limit their reads to space().
    LOG_W("AAC输入缓冲区空间不足，丢弃 %d 字节输入", len - space);
    len = space;
  }

  memcpy(buf_ + len_, data, (size_t)len);
  len_ += len;
  return len;
}

void AacDecoder::resync() {
  if (len_ <= 1) {
    // Not even a full sync word left; keep the trailing byte because the sync
    // word may straddle this read and the next one.
    if (len_ == 1) {
      buf_[0] = buf_[pos_];
      pos_ = 0;
      // len_ stays 1
    } else {
      pos_ = 0;
      len_ = 0;
    }
    return;
  }

  int offset = AACFindSyncWord(buf_ + pos_ + 1, len_ - 1);
  if (offset < 0) {
    buf_[0] = buf_[pos_ + len_ - 1];
    pos_ = 0;
    len_ = 1;
    return;
  }
  int drop = offset + 1;
  pos_ += drop;
  len_ -= drop;
}

int AacDecoder::decodeFrame(int *samplesOut, int *channelsOut, int *sampleRateOut) {
  if (samplesOut != nullptr) *samplesOut = 0;
  if (channelsOut != nullptr) *channelsOut = 1;
  if (sampleRateOut != nullptr) *sampleRateOut = 0;

  if (decoder_ == nullptr || len_ <= 0) return 1;

  unsigned char *in = buf_ + pos_;
  int left = len_;
  int rc = AACDecode((HAACDecoder)decoder_, &in, &left, (short *)pcm_);

  if (rc == 0) {
    int used = (int)(in - (buf_ + pos_));
    pos_ += used;
    len_ -= used;
    if (len_ == 0) pos_ = 0;

    FrameFormat f = readFormat(decoder_);
    if (f.sampleRate <= 0 || f.samples <= 0) {
      // Header parsed but no audio yet (for example an ADIF/PCE-only prefix).
      return 1;
    }
    if (f.samples > maxSamples()) {
      LOG_W("AAC帧超出PCM缓冲区: %d 采样", f.samples);
      return -1;
    }

    if (samplesOut != nullptr) *samplesOut = f.samples;
    if (channelsOut != nullptr) *channelsOut = f.channels;
    if (sampleRateOut != nullptr) *sampleRateOut = f.sampleRate;
    return 0;
  }

  if (rc == ERR_AAC_INDATA_UNDERFLOW) {
    // Need more bytes: the C API leaves the pointers untouched.
    return 1;
  }

  LOG_W("AAC解码错误 %d，跳过损坏数据", rc);
  resync();
  return -1;
}

}  // namespace audio