#ifndef AUDIO_VOLUME_H
#define AUDIO_VOLUME_H

#include <stdint.h>

/**
 * @file audio_volume.h
 * @brief Volume curve and click-free gain application for the prompt player.
 *
 * The curve reproduces what the firmware used before this module existed: the
 * AudioTools VolumeStream defaulted to a SimulatedAudioPot with
 * x = 0.5, y = 0.1, so a requested volume v mapped to
 *
 *     v in [0.0 .. 0.5] -> factor in [0.0 .. 0.1]
 *     v in [0.5 .. 1.0] -> factor in [0.1 .. 1.0]
 *
 * Existing prompt volumes (VOLUME1 = 1.0, 0.9, 0.7) therefore keep their
 * previous loudness ratios.
 */
namespace audio {

/// Multiplication factor for a requested volume in [0.0 .. 1.0].
float volumeFactor(float volume);

/// Number of frames used for the linear fade at the start and the end of a
/// prompt. The AudioTools FadeStream ramped over a whole 1024 frame buffer;
/// this is the equivalent anti-click ramp scaled down.
constexpr int kFadeFrames = 256;

/**
 * @brief Applies volume and fade to one decoded AAC frame, producing the
 *        interleaved 16 bit stereo buffer handed to I2S.
 *
 * @param src            decoded PCM, interleaved per @p channels
 * @param samples        total sample count in @p src (frames * channels)
 * @param channels       source channel count, 1 or 2
 * @param gain           factor from volumeFactor()
 * @param fadeOut        true for the final frame: ramps from @p gain to 0
 * @param fadeInFrames   remaining fade-in frames, decremented in place
 * @param dst            output buffer, must hold frames*2 int16_t where
 *                       frames = samples / channels (a mono frame is expanded
 *                       to both I2S slots)
 * @return number of interleaved stereo frames written
 */
int applyVolume(const int16_t *src, int samples, int channels, float gain,
                bool fadeOut, int &fadeInFrames, int16_t *dst);

}  // namespace audio

#endif  // AUDIO_VOLUME_H