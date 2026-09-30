#pragma once
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Allocates zeroed memory for the AAC decoder. On the ESP32 this is
/// deliberately plain calloc(): the ESP32-C3 has no PSRAM, and the
/// decoder state is small enough (~30 KiB) for the internal heap.
void *helix_malloc(int size);

/// Releases memory obtained from helix_malloc()
void helix_free(void *ptr);

#ifdef __cplusplus
}
#endif