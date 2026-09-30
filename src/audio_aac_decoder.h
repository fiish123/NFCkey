#ifndef AUDIO_AAC_DECODER_H
#define AUDIO_AAC_DECODER_H

#include <stddef.h>
#include <stdint.h>

/**
 * @file audio_aac_decoder.h
 * @brief AAC-LC / ADTS decoder with its own input staging buffer.
 *
 * Wraps the trimmed vendored Helix decoder (lib/helix_aac) directly, without
 * the Arduino AudioTools layers that used to sit in between
 * (AACDecoderHelix -> CommonHelix -> Vector/SingleBuffer -> Allocator).
 *
 * Responsibilities kept here:
 *   - staging the AAC bytes until at least one complete ADTS frame is present
 *   - resynchronising after a corrupt frame instead of aborting playback
 *   - reporting the PCM format of the decoded frame (the prompt files are all
 *     44.1 kHz mono LC, but the player follows what the bitstream says)
 *
 * The caller owns the file and feeds bytes with feed(); a decode attempt has no
 * allocation side effects, which keeps the light-sleep transition predictable.
 */
namespace audio {

class AacDecoder {
 public:
  AacDecoder() = default;
  ~AacDecoder();

  AacDecoder(const AacDecoder &) = delete;
  AacDecoder &operator=(const AacDecoder &) = delete;

  /// Allocates the decoder state and resets the staging buffer.
  bool begin();

  /// Releases the decoder state and clears the staging buffer.
  void end();

  /// True once begin() succeeded.
  bool isOpen() const { return decoder_ != nullptr; }

  /// Appends AAC bytes. Returns the number of bytes accepted, which is 0 when
  /// the staging buffer is full and decodeFrame() has to run first.
  ///
  /// At most space() bytes are kept: a caller that hands over more than that
  /// has the tail dropped, which leaves an unrecoverable hole in the AAC
  /// stream. Either limit the write to space() or keep the rejected tail.
  int feed(const uint8_t *data, int len);

  /// Number of staged bytes not yet consumed.
  int buffered() const { return len_; }

  /// Free space in the staging buffer, in bytes.
  int space() const { return kBufferBytes - len_; }

  /// True when more input should be read before attempting a decode.
  bool needsMore() const { return len_ < kMinFrameBytes; }

  /// Decodes the next complete ADTS frame from the staging buffer.
  ///
  /// @param samplesOut   interleaved sample count of the decoded frame
  /// @param channelsOut  channel count of the decoded frame
  /// @param sampleRateOut PCM sample rate of the decoded frame
  /// @return  0  a frame was decoded (read pcm())
  ///          1  not enough data yet, feed() more
  ///         -1  the bitstream ended or a corrupt frame was skipped; when the
  ///             return value is -1 and buffered() == 0 the caller should stop
  ///             feeding
  int decodeFrame(int *samplesOut, int *channelsOut, int *sampleRateOut);

  /// Interleaved 16 bit PCM produced by the last successful decodeFrame().
  const int16_t *pcm() const { return pcm_; }

  /// Maximum number of interleaved samples pcm() can hold.
  static int maxSamples();

 private:
  /// Drops staged bytes that cannot start a frame.
  void resync();

  static constexpr int kBufferBytes = 4096;  // > largest ADTS frame (2100)
  static constexpr int kMinFrameBytes = 16;

  void *decoder_ = nullptr;
  uint8_t buf_[kBufferBytes] = {};
  int pos_ = 0;  // read offset inside buf_
  int len_ = 0;  // valid bytes from pos_
  int16_t pcm_[2048] = {};  // AAC_MAX_NSAMPS * AAC_MAX_NCHANS
};

}  // namespace audio

#endif  // AUDIO_AAC_DECODER_H