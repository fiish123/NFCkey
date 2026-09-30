#pragma once

// ======================================================================
//  Minimal libhelix configuration for the trimmed AAC-only decoder
// ======================================================================
//  This directory is a reduced copy of the MPEG-4 LC / ADTS AAC decoder
//  from arduino-libhelix (https://github.com/pschatzmann/arduino-libhelix,
//  derived from the RealNetworks Helix fixed-point decoder, GPLv3).
//
//  Only the files listed below are kept, and only these switches are
//  supported. Nothing in this directory depends on the Arduino AudioTools
//  library any more.
//
//      aacdec.c aactabs.c bitstream.c buffers.c dct4.c decelmnt.c
//      dequant.c fft.c filefmt.c huffman.c hufftabs.c imdct.c
//      noiseless.c pns.c stproc.c tns.c trigtabs.c helix_memory.cpp
//
//  Deliberately removed relative to upstream:
//    * libhelix-mp3            (MP3 decoding - unused)
//    * sbr*.c / sbr.h          (SBR extension of HE-AAC)
//    * sbrimdct.c              (only called from the SBR path)
//    * CommonHelix.h/.cpp      (AudioTools Stream/Print glue and buffers)
//    * utils/Allocator.h, Buffers.h, Vector.h, helix_log*.h
//
//  Feature macros intentionally NOT defined:
//    HELIX_FEATURE_AUDIO_CODEC_AAC_SBR
//        Enables SBR. All shipped prompt files are 44.1 kHz AAC-LC
//        without SBR; enabling it doubles the PCM output buffer and adds
//        an int[2][1024] work buffer plus the full SBR state to the
//        decoder.
//    HELIX_CONFIG_AAC_GENERATE_TRIGTABS_FLOAT
//        Would keep the trig tables in RAM instead of flash.
// ======================================================================

#include <stddef.h>
#include <stdint.h>

// Unused by the remaining sources (kept so the C sources stay untouched),
// but defined for completeness.
#ifndef SYNCH_WORD_LEN
#  define SYNCH_WORD_LEN 4
#endif
#ifndef AAC_MAX_OUTPUT_SIZE
#  define AAC_MAX_OUTPUT_SIZE (AAC_MAX_NCHANS * AAC_MAX_NSAMPS * 2)
#endif
#ifndef AAC_MAX_FRAME_SIZE
#  define AAC_MAX_FRAME_SIZE 2100
#endif
#ifndef AAC_MIN_FRAME_SIZE
#  define AAC_MIN_FRAME_SIZE 16
#endif