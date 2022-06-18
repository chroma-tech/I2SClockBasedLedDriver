#pragma once
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
typedef union
{
    uint8_t bytes[16];
    uint32_t shorts[8];
    uint32_t raw[2];
} Lines;

struct OffsetDisplay
{
    int offsetx;
    int offsety;
    int panel_height;
    int panel_width;
};

enum colorarrangment
{
    ORDER_GRBW,
    ORDER_RGB,
    ORDER_RBG,
    ORDER_GRB,
    ORDER_GBR,
    ORDER_BRG,
    ORDER_BGR,
};

enum displayMode
{
    NO_WAIT,
    WAIT,
    LOOP,
    LOOP_INTERUPT,
};

int MOD(int a, int b)
{
    /* if (b == 1)
    {
        if (a < 0)
            return -a;
        else
            return a;
    }*/
    if (a < 0)
    {
        if (-a % b == 0)
            return 0;
        else
            return b - (-a) % b;
    }
    else
        return a % b;
}

struct LedTiming
{

    // led timing
    uint32_t T0;
    uint32_t T1;
    uint32_t T2;

    // compileled
    uint8_t f1;
    uint8_t f2;
    uint8_t f3;
};
