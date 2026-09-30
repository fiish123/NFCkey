#include "audio_volume.h"

namespace audio {

float volumeFactor(float volume) {
  if (volume < 0.0f) volume = 0.0f;
  if (volume > 1.0f) volume = 1.0f;

  // Piecewise taper: slow rise up to the midpoint, steep after it.
  if (volume <= 0.5f) {
    return volume * 0.2f;  // (0, 0) -> (0.5, 0.1)
  }
  return 0.1f + (volume - 0.5f) * 1.8f;  // (0.5, 0.1) -> (1.0, 1.0)
}

int applyVolume(const int16_t *src, int samples, int channels, float gain,
                bool fadeOut, int &fadeInFrames, int16_t *dst) {
  if (src == nullptr || dst == nullptr || samples <= 0) return 0;
  if (channels < 1) channels = 1;
  if (channels > 2) channels = 2;

  const int frames = samples / channels;
  const float fadeOutStep = fadeOut && frames > 0 ? 1.0f / (float)frames : 0.0f;

  int out = 0;
  for (int i = 0; i < frames; i++) {
    float f = gain;
    if (fadeOut) {
      f *= 1.0f - fadeOutStep * (float)i;
    } else if (fadeInFrames > 0) {
      f *= 1.0f - (float)fadeInFrames / (float)kFadeFrames;
      fadeInFrames--;
    }

    if (channels == 1) {
      // Duplicate the mono prompt to both I2S slots.
      float v = f * (float)src[i];
      if (v > 32767.0f) v = 32767.0f;
      if (v < -32768.0f) v = -32768.0f;
      int16_t s = (int16_t)v;
      dst[out++] = s;
      dst[out++] = s;
    } else {
      for (int c = 0; c < 2; c++) {
        float v = f * (float)src[i * channels + c];
        if (v > 32767.0f) v = 32767.0f;
        if (v < -32768.0f) v = -32768.0f;
        dst[out++] = (int16_t)v;
      }
    }
  }
  return frames;
}

}  // namespace audio