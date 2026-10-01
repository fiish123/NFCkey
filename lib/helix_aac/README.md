# helix_aac - trimmed AAC decoder for NFCkey

Minimal, self-contained copy of the AAC decoder used by the door access
firmware. Only AAC-LC decoding is included. The former AudioTools and full
arduino-libhelix dependencies have been removed from the repository.

## Provenance and licence

All files here are copied from
<https://github.com/pschatzmann/arduino-libhelix> (version 0.9.2), which is
itself derived from the RealNetworks Helix fixed-point MPEG-4 AAC decoder
(Jon Recker, February 2005). Both upstream projects are GPLv3 - see
`License.txt` in the original library directory in the repository history.

Files are kept byte-identical to upstream except for the additions noted below,
so re-syncing with upstream stays mechanical.

## What is kept

| File | Purpose |
|------|---------|
| `aacdec.c` / `aacdec.h` | public C API: `AACInitDecoder`, `AACDecode`, `AACGetLastFrameInfo`, `AACFreeDecoder`, `AACFindSyncWord` |
| `aaccommon.h`, `bitstream.[ch]`, `coder.h`, `assembly.h`, `statname.h` | decoder state, bit reader, platform macros |
| `aactabs.c`, `hufftabs.c`, `trigtabs.c` | constant tables in flash |
| `decelmnt.c`, `noiseless.c`, `huffman.c`, `dequant.c`, `stproc.c`, `pns.c`, `tns.c` | raw data block decoding |
| `dct4.c`, `fft.c`, `imdct.c` | transform and windowing |
| `filefmt.c`, `buffers.c` | ADTS/ADIF parsing, decoder state allocation |
| `helix_memory.cpp`, `utils/helix_memory.h` | `calloc`/`free` backing for `buffers.c` |
| `ConfigHelix.h` | trimmed configuration (replaces the 200 line upstream version) |
| `utils/helix_pgm.h` | `PROGMEM` shim |

## What was removed and why

| Removed | Reason |
|---------|--------|
| `libhelix-mp3/*` | only AAC prompts are shipped |
| `sbr*.c`, `sbr.h` | SBR is not used: all prompts are 44.1 kHz AAC-LC. Also removes the `int[2][1024]` SBR work buffer from the decoder state and halves the PCM buffer |
| `sbrimdct.c` | only reachable from the SBR path in `imdct.c` |
| `CommonHelix.h/.cpp`, `AACDecoderHelix.h`, `MP3DecoderHelix.h` | Arduino `Stream`/`Print` glue and `Vector`/`SingleBuffer` heap buffers; the firmware now decodes through the plain C API |
| `utils/Allocator.h`, `utils/Buffers.h`, `utils/Vector.h` | only used by the removed glue |
| `utils/helix_log*.h` | logging was disabled; errors surface through `AACDecode()` return codes |
| `examples/`, `docs/`, `CMakeLists.txt`, `Doxyfile` | build/documentation scaffolding |

`ConfigHelix.h` no longer defines `ALLOCATOR`, so `buffers.c` pulls in
`utils/helix_memory.h` and `helix_memory.cpp` provides the two C entry points
with `calloc`/`free`. That is what upstream's `AllocatorExt` does on a target
without PSRAM, minus the PSRAM probe.

## Deliberately not enabled

* `HELIX_FEATURE_AUDIO_CODEC_AAC_SBR` - see above. Enabling it again requires
  restoring `sbr*.c`/`sbr.h`/`sbrimdct.c` and the matching `#define`.
* `HELIX_CONFIG_AAC_GENERATE_TRIGTABS_FLOAT` - keeps the trig tables in RAM.

## Footprint

Cross-compiled for `rv32imc` with `-Os` the 17 objects total about 46 KiB of
`.text` (of which ~22 KiB is `trigtabs.c`). The decoder state allocated at
runtime is roughly 30 KiB, and is only allocated while a prompt is playing.

## Consumers

`src/audio_aac_decoder.cpp` wraps this C API. Nothing else in the firmware
includes these headers directly.