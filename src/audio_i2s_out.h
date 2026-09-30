#ifndef AUDIO_I2S_OUT_H
#define AUDIO_I2S_OUT_H

#include <stddef.h>
#include <stdint.h>

/**
 * @file audio_i2s_out.h
 * @brief Thin I2S transmit wrapper for the external DAC.
 *
 * This is the small replacement for the Arduino AudioTools I2SStream plus the
 * channel expansion logic that used to run inside AudioTools. It is
 * intentionally not a Stream: the audio player feeds interleaved 16 bit stereo
 * frames directly, and PCM data that arrives as mono is duplicated to both
 * slots before it reaches the driver.
 *
 * I2S is opened closed on demand. Nothing is transmitted during light sleep,
 * which is what the lifecycle in audio_player.cpp relies on.
 */
namespace audio {

/// Opens the I2S port in master TX mode.
/// @param pinBclk  bit clock pin (BCK)
/// @param pinLrc   word select pin (WS / LRCK)
/// @param pinData  serial data out pin (DIN of the DAC)
/// @param sampleRate output sample rate in Hz
/// @param channels number of source channels, 1 or 2; mono is duplicated
/// @return true on success
bool i2sOpen(int pinBclk, int pinLrc, int pinData, int sampleRate, int channels);

/// Uninstalls the I2S driver and releases the DMA buffers.
void i2sClose();

/// True while the port is installed and transmitting.
bool i2sIsOpen();

/// Current output sample rate in Hz, or 0 when the port is closed.
int i2sSampleRate();

/// Reconfigures the output rate. Must be called while nothing has been written
/// for the current prompt yet (the port is briefly stopped). Returns false when
/// the driver rejected the rate.
bool i2sSetSampleRate(int sampleRate);

/// Writes interleaved 16 bit stereo frames, blocking until the driver consumed
/// them. Returns false if the write failed (for example after i2sClose()).
bool i2sWrite(const int16_t *stereoFrames, size_t frames);

}  // namespace audio

#endif  // AUDIO_I2S_OUT_H