#ifndef AUDIO_PLAYER_H
#define AUDIO_PLAYER_H

#include <stddef.h>
#include <stdint.h>

/**
 * @file audio_player.h
 * @brief Minimal AAC prompt player: decoder + volume + I2S output + task and
 *        light-sleep lifecycle.
 *
 * This is a self-contained AAC prompt chain. Only the pieces this firmware
 * actually uses are kept:
 *
 *   - AAC-LC / ADTS decoding through the trimmed vendored Helix decoder
 *     (lib/helix_aac, see lib/helix_aac/README.md)
 *   - per-prompt volume using the same piecewise "audio taper" curve the
 *     AudioTools VolumeStream used, plus the short fade-in that removed the
 *     start click
 *   - I2S output on the external DAC (16 bit, mono source duplicated to both
 *     slots, 6 x 512 frame DMA buffers)
 *   - one playback task that owns every hardware transition, so the I2S
 *     driver and the decoder allocation only exist while a prompt burst is
 *     actually playing. That is what makes light sleep safe: when the queue is
 *     empty there is no DMA activity, no decoder heap and no powered analog
 *     stage (main.cpp switches the DAC/5V rail off with powermanager() before
 *     sleeping), so the caller can call beforeLightSleep() and go straight into
 *     esp_light_sleep_start().
 */
namespace audio {

/// Optional hook to switch the shared DAC/5V rail while audio is active.
/// Deliberately optional: in this firmware main.cpp owns that rail itself
/// (addTolist() powers it up, loop() powers it down before light sleep) and
/// never installs a hook, because powermanager() already de-duplicates the
/// switch-on. Wire it to powermanager(1, on) only if the caller wants the rail
/// to follow the playback burst instead.
using PowerHook = void (*)(bool on);
void setPowerHook(PowerHook hook);

/// Creates the queue/mutex, records the I2S pins and starts the playback task.
/// Call once from setup() after LittleFS is mounted. The I2S port itself is
/// opened lazily on the first prompt, so nothing is transmitting while idle.
/// Returns false on allocation failure.
bool begin(int pinBclk, int pinLrc, int pinData);

/// Stops playback (interrupting the current prompt), releases the decoder and
/// closes the I2S output. The playback task keeps running and will restart the
/// output on the next enqueue().
void end();

/// Queues a prompt file for playback. @p path must outlive the playback
/// (string literals are expected, as produced by getAudioPath()).
/// @p volume is 0.0 .. 1.0 and is applied with the AudioTools taper curve.
/// The call is cheap and safe from any task.
void enqueue(const char *path, float volume);

/// True from the moment a prompt is queued until the queue has drained and the
/// output has been powered down again.
bool isPlaying();

/// Blocks (while yielding) until isPlaying() becomes false.
/// @param timeoutMs 0 means "wait forever".
/// @return true if playback finished, false on timeout.
bool waitIdle(uint32_t timeoutMs = 0);

/// Light-sleep lifecycle. beforeLightSleep() interrupts/waits for any playback
/// and closes the I2S driver and the decoder, so the caller may enter
/// esp_light_sleep_start() without DMA or DAC activity (the DAC/5V rail itself
/// is switched off by the caller or by the power hook, whichever owns it). The
/// output is re-opened lazily on the next enqueue(), so afterLightSleep() only
/// has to clear the stop request.
void beforeLightSleep();
void afterLightSleep();

/// Number of prompts currently queued plus the one being played.
int pending();

}  // namespace audio

#endif  // AUDIO_PLAYER_H