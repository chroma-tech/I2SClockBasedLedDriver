#pragma once

#include "esp_attr.h"

#ifndef NUMSTRIPS
#define NUMSTRIPS 16
#endif

#ifndef SNAKEPATTERN
#define SNAKEPATTERN 0
#endif

#ifndef ALTERNATEPATTERN
#define ALTERNATEPATTERN 0
#endif

#define AA (0x00AA00AAL)
#define CC (0x0000CCCCL)
#define FF (0xF0F0F0F0L)
#define FF2 (0x0F0F0F0FL)

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#ifndef HARDWARESPRITES
#define HARDWARESPRITES 0
#endif
#ifndef NUM_LEDS_PER_STRIP
#define NUM_LEDS_PER_STRIP 50
#endif

#if HARDWARESPRITES == 1
#include "hardwareSprite.h"
#endif
#define I2S_BASE_CLK (80000000L)
typedef union {
  uint8_t bytes[16];
  uint32_t shorts[8];
  uint32_t raw[2];
} Lines;

struct OffsetDisplay {
  int offsetx;
  int offsety;
  int panel_height;
  int panel_width;
};

enum colorarrangment {
  ORDER_GRBW,
  ORDER_RGB,
  ORDER_RBG,
  ORDER_GRB,
  ORDER_GBR,
  ORDER_BRG,
  ORDER_BGR,
};

enum displayMode {
  NO_WAIT,
  WAIT,
  LOOP,
  LOOP_INTERUPT,
};

int MOD(int a, int b) {
  /* if (b == 1)
  {
      if (a < 0)
          return -a;
      else
          return a;
  }*/
  if (a < 0) {
    if (-a % b == 0)
      return 0;
    else
      return b - (-a) % b;
  } else
    return a % b;
}

struct LedTiming {

  // led timing
  uint32_t T0;
  uint32_t T1;
  uint32_t T2;

  // compileled
  uint8_t f1;
  uint8_t f2;
  uint8_t f3;
};

static void IRAM_ATTR transpose16x1_noinline2(unsigned char *A, uint8_t *B) {

  uint32_t x, y, x1, y1, t;

  y = *(unsigned int *)(A);
#if NUMSTRIPS > 4
  x = *(unsigned int *)(A + 4);
#else
  x = 0;
#endif

#if NUMSTRIPS > 8
  y1 = *(unsigned int *)(A + 8);
#else
  y1 = 0;
#endif
#if NUMSTRIPS > 12
  x1 = *(unsigned int *)(A + 12);
#else
  x1 = 0;
#endif

  // pre-transform x
#if NUMSTRIPS > 4
  t = (x ^ (x >> 7)) & AA;
  x = x ^ t ^ (t << 7);
  t = (x ^ (x >> 14)) & CC;
  x = x ^ t ^ (t << 14);
#endif
#if NUMSTRIPS > 12
  t = (x1 ^ (x1 >> 7)) & AA;
  x1 = x1 ^ t ^ (t << 7);
  t = (x1 ^ (x1 >> 14)) & CC;
  x1 = x1 ^ t ^ (t << 14);
#endif
  // pre-transform y
  t = (y ^ (y >> 7)) & AA;
  y = y ^ t ^ (t << 7);
  t = (y ^ (y >> 14)) & CC;
  y = y ^ t ^ (t << 14);
#if NUMSTRIPS > 8
  t = (y1 ^ (y1 >> 7)) & AA;
  y1 = y1 ^ t ^ (t << 7);
  t = (y1 ^ (y1 >> 14)) & CC;
  y1 = y1 ^ t ^ (t << 14);
#endif
  // final transform
  t = (x & FF) | ((y >> 4) & FF2);
  y = ((x << 4) & FF) | (y & FF2);
  x = t;

  t = (x1 & FF) | ((y1 >> 4) & FF2);
  y1 = ((x1 << 4) & FF) | (y1 & FF2);
  x1 = t;

  *((uint16_t *)(B + 2)) =
      (uint16_t)(((x & 0xff000000) >> 8 | ((x1 & 0xff000000))) >> 16);
  *((uint16_t *)(B)) =
      (uint16_t)(((x & 0xff0000) >> 16 | ((x1 & 0xff0000) >> 8)));
  *((uint16_t *)(B + 6)) =
      (uint16_t)(((x & 0xff00) | ((x1 & 0xff00) << 8)) >> 8);
  *((uint16_t *)(B + 4)) = (uint16_t)((x & 0xff) | ((x1 & 0xff) << 8));
  *((uint16_t *)(B + 10)) =
      (uint16_t)(((y & 0xff000000) >> 8 | ((y1 & 0xff000000))) >> 16);
  *((uint16_t *)(B + 8)) =
      (uint16_t)(((y & 0xff0000) | ((y1 & 0xff0000) << 8)) >> 16);
  *((uint16_t *)(B + 14)) =
      (uint16_t)(((y & 0xff00) | ((y1 & 0xff00) << 8)) >> 8);
  *((uint16_t *)(B + 12)) = (uint16_t)((y & 0xff) | ((y1 & 0xff) << 8));
}
