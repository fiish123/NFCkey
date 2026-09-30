#include "utils/helix_memory.h"

// Minimal backing allocator for the trimmed libhelix AAC decoder.
//
// The upstream arduino-libhelix library funnels its allocations through a
// C++ allocator object so that PSRAM can be preferred on ESP32. The
// ESP32-C3 has no PSRAM, and the decoder state is only ~30 KiB, so plain
// zeroed calloc() is exactly equivalent and avoids pulling in the
// allocator/logging/buffer utility headers entirely.

extern "C" void *helix_malloc(int size) {
  return calloc(1, size <= 0 ? 1 : (size_t)size);
}

extern "C" void helix_free(void *ptr) { free(ptr); }