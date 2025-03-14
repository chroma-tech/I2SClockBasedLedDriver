#pragma once
#include "I2SLedDriver.h"

#define BA0 0
#define BA1 3
#define BA2 2
#define BA3 1
#define BA4 0
#define BA5 0
#define BA6 0
#define BA7 0
#define BA8 0
#define BA9 0
#define BA10 0
#define BA11 0
#define BA12 0
#define BA13 0
#define BA14 0
#define BA15 0
#define BA16 0
#define BA17 0
#define BA18 0
#define BA19 0

#define DATA_SIZE 1
#define BRIGHTNESS 1
#define NUMBER_OF_BLOCK 4
#define START_FRAME_SIZE 4
#define END_FRAME 1
namespace I2SClockBased {

static void IRAM_ATTR _I2SClockBasedLedDriverinterruptHandler(void *arg);
static void IRAM_ATTR loadAndTranspose(uint8_t *ledt, int led_per_strip,
                                       int num_stripst, OffsetDisplay offdisp,
                                       uint8_t *buffer, int ledtodisp, int pg,
                                       int pr, int pb) {
  Lines secondPixel[NUMBER_OF_BLOCK];
  // uint8_t *poli=ledt+ledtodisp*NUMBER_OF_BLOCK;
  uint32_t offp, offi, offsetled;
  uint8_t _g, _r, _b;
  int x, y, X, Y, deltaY;
  deltaY =
      led_per_strip / offdisp.panel_width; // calculation of height per strip
  // printf("li%d:\n",line_width);
  y = ledtodisp / offdisp.panel_width;
  x = ledtodisp % offdisp.panel_width;
  Y = y;
  X = x;

  offi =
      ((x + offdisp.offsetx) % offdisp.panel_width) + y * offdisp.panel_width;
  offp = offi;

  // uint8_t *poli = ledt + ((ledtodisp+startleds)%led_per_strip) *
  // nbcomponents;
  offi = offi * NUMBER_OF_BLOCK;
  offp = offp * NUMBER_OF_BLOCK;
  uint8_t *poli = ledt + offi;
  offsetled = ledtodisp;
  for (int i = 0; i < num_stripst; i++) {

    if (poli >= ledt + (uint32_t)offdisp.panel_width * offdisp.panel_height *
                           NUMBER_OF_BLOCK) {
      // Serial.printf("%d %d %d\n",i,ledtodisp,((uint32_t)poli -
      // (uint32_t)offdisp.panel_width * offdisp.panel_height *
      // nbcomponents-(uint32_t)ledt))/3;
      poli = poli - (uint32_t)offdisp.panel_width * offdisp.panel_height *
                        NUMBER_OF_BLOCK;
    } else {
      if (poli < ledt) {
        // Serial.printf("neg %d %d %d\n",i,ledtodisp,(ledt-poli)/3);
        poli += (uint32_t)offdisp.panel_width * offdisp.panel_height *
                NUMBER_OF_BLOCK;
      }
    }

    _g = *(poli + 1);
    _r = *(poli);
    _b = *(poli + 2);

#if NUMBER_OF_BLOCK >= 1
    secondPixel[BA0].bytes[i] = *(poli);
#endif
#if NUMBER_OF_BLOCK >= 2
#if DATA_SIZE == 1
    secondPixel[BA1].bytes[i] = *(poli + 1); // mapr[*(poli+1)];
#else
    secondPixel[BA1].bytes[i] = *(poli + 1);
#endif

#endif
#if NUMBER_OF_BLOCK >= 3
#if DATA_SIZE == 1
    secondPixel[BA2].bytes[i] = *(poli + 2); // mapg[*(poli+2)];
#else
    f = (*((uint16_t *)(poli + 2))) / brightness;
    p1 = f >> 8;
    p2 = f & 255;
    secondPixel[BA2].bytes[i] = p1;
#endif
#endif
#if NUMBER_OF_BLOCK >= 4
#if DATA_SIZE == 1
    secondPixel[BA3].bytes[i] = *(poli + 3); // mapb[*(poli+3)];
#else
    secondPixel[BA3].bytes[i] = p2;
#endif
#endif

    poli += led_per_strip * NUMBER_OF_BLOCK;

    Y += deltaY;

    if (i % 2 == 0) {
      poli = poli - offi + offp;
    } else {
      poli = poli - offp + offi;
    }
  }
#if NUMBER_OF_BLOCK >= 1
  transpose16x1_noinline2(secondPixel[0].bytes, (uint8_t *)(buffer + 16 * 0));
#endif
#if NUMBER_OF_BLOCK >= 2
  transpose16x1_noinline2(secondPixel[1].bytes, (uint8_t *)(buffer + 16));
#endif
#if NUMBER_OF_BLOCK >= 3
  transpose16x1_noinline2(secondPixel[2].bytes, (uint8_t *)(buffer + 16 * 2));
#endif
#if NUMBER_OF_BLOCK >= 4
  transpose16x1_noinline2(secondPixel[3].bytes, (uint8_t *)(buffer + 16 * 3));
#endif
}

class I2SClockBasedLedDriver {
  bool _has_i2s_init = false;

  struct I2SClockBasedLedDriverDMABuffer {
    lldesc_t descriptor;
    uint8_t *buffer;
  };

  const int deviceBaseIndex[2] = {I2S0O_DATA_OUT0_IDX, I2S1O_DATA_OUT0_IDX};
  const int deviceClockIndex[2] = {I2S0O_BCK_OUT_IDX, I2S1O_BCK_OUT_IDX};
  const int deviceWordSelectIndex[2] = {I2S0O_WS_OUT_IDX, I2S1O_WS_OUT_IDX};
  const periph_module_t deviceModule[2] = {PERIPH_I2S0_MODULE,
                                           PERIPH_I2S1_MODULE};

public:
  i2s_dev_t *i2s;
  uint8_t _i2s_num;

  intr_handle_t _gI2SClocklessDriver_intr_handle;
  volatile SemaphoreHandle_t I2SClockBasedLedDriver_sem = NULL;
  volatile SemaphoreHandle_t I2SClockBasedLedDriver_semSync = NULL;
  volatile SemaphoreHandle_t I2SClockBasedLedDriver_semDisp = NULL;
  volatile int dmaBufferActive = 0;
  volatile bool wait;
  displayMode __displayMode;
  volatile int ledToDisplay;
  OffsetDisplay _offsetDisplay, _defaultOffsetDisplay;
  // volatile int oo=0;
  uint8_t *leds;
  int linewidth;
  int dmaBufferCount = 2; // we use two buffers
  volatile bool transpose = false;

  volatile int num_strips = 0;
  volatile int num_led_per_strip;
  // int clock_pin;
  int p_r, p_g, p_b;
  int i2s_base_pin_index;
  /*
   This flag is used when using the NO_WAIT modeÒÒ
   */
  volatile bool isDisplaying = false;
  volatile bool isWaiting = true;
  volatile bool framesync = false;
  volatile int counti;

  I2SClockBasedLedDriver(uint8_t i2s_num = 0) : _i2s_num(i2s_num){};
  void setPins(int *Pins, int clock_pin) {

    for (int i = 0; i < num_strips; i++) {

      PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[Pins[i]], PIN_FUNC_GPIO);
      gpio_set_direction((gpio_num_t)Pins[i],
                         (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
      gpio_matrix_out(Pins[i], deviceBaseIndex[_i2s_num] + i + 8, false, false);
    }
    gpio_matrix_out(clock_pin, deviceClockIndex[_i2s_num], false, false);
  }

  void i2sInit(int clockMHz) {
    int interruptSource;
    int cA = clockMHz;
    int cN = (uint8_t)80 / cA;
    int cB = 80 % clockMHz;
    if (_i2s_num == 0) {
      i2s = &I2S0;
      periph_module_enable(PERIPH_I2S0_MODULE);
      interruptSource = ETS_I2S0_INTR_SOURCE;
      i2s_base_pin_index = I2S0O_DATA_OUT0_IDX;
    } else {
      i2s = &I2S1;
      periph_module_enable(PERIPH_I2S1_MODULE);
      interruptSource = ETS_I2S1_INTR_SOURCE;
      i2s_base_pin_index = I2S1O_DATA_OUT0_IDX;
    }

    i2sReset();
    i2sReset_DMA();
    i2sReset_FIFO();
    i2s->conf.tx_right_first = 0;

    // -- Set parallel mode
    i2s->conf2.val = 0;
    i2s->conf2.lcd_en = 1;
    i2s->conf2.lcd_tx_wrx2_en = 1; // 0 for 16 or 32 parallel output
    i2s->conf2.lcd_tx_sdx2_en = 0; // HN

    // -- Set up the clock rate and sampling
    i2s->sample_rate_conf.val = 0;
    i2s->sample_rate_conf.tx_bits_mod = 16; // Number of parallel bits/pins
    i2s->clkm_conf.val = 0;

    i2s->clkm_conf.clka_en = 0;

    i2s->clkm_conf.clkm_div_a = cA;   // CLOCK_DIVIDER_A;
    i2s->clkm_conf.clkm_div_b = cB;   // CLOCK_DIVIDER_B;
    i2s->clkm_conf.clkm_div_num = cN; // CLOCK_DIVIDER_N;
    i2s->fifo_conf.val = 0;
    i2s->fifo_conf.tx_fifo_mod_force_en = 1;
    i2s->fifo_conf.tx_fifo_mod = 1;  // 16-bit single channel data
    i2s->fifo_conf.tx_data_num = 32; // 32; // fifo length
    i2s->fifo_conf.dscr_en = 1;      // fifo will use dma
    i2s->sample_rate_conf.tx_bck_div_num = 1;
    i2s->conf1.val = 0;
    i2s->conf1.tx_stop_en = 0;
    i2s->conf1.tx_pcm_bypass = 1;

    i2s->conf_chan.val = 0;
    i2s->conf_chan.tx_chan_mod =
        1; // Mono mode, with tx_msb_right = 1, everything goes to right-channel

    i2s->timing.val = 0;
    i2s->int_ena.val = 0;
    /*
    // -- Allocate i2s interrupt
    SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_DEVICE), I2S_OUT_EOF_INT_ENA_V,1,
    I2S_OUT_EOF_INT_ENA_S); SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_DEVICE),
    I2S_OUT_TOTAL_EOF_INT_ENA_V, 1, I2S_OUT_TOTAL_EOF_INT_ENA_S);
    SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_DEVICE), I2S_OUT_TOTAL_EOF_INT_ENA_V,
    1, I2S_OUT_TOTAL_EOF_INT_ENA_S);
    */
    // esp_err_t e =
    esp_intr_alloc(interruptSource,
                   ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL3 |
                       ESP_INTR_FLAG_IRAM,
                   &_I2SClockBasedLedDriverinterruptHandler, this,
                   &_gI2SClocklessDriver_intr_handle);

    // -- Create a semaphore to block execution until all the controllers are
    // done

    if (I2SClockBasedLedDriver_sem == NULL) {
      I2SClockBasedLedDriver_sem = xSemaphoreCreateBinary();
    }

    if (I2SClockBasedLedDriver_semSync == NULL) {
      I2SClockBasedLedDriver_semSync = xSemaphoreCreateBinary();
    }
    if (I2SClockBasedLedDriver_semDisp == NULL) {
      I2SClockBasedLedDriver_semDisp = xSemaphoreCreateBinary();
    }
  }

  void initDMABuffers() {
    DMABuffersTampon[0] =
        allocateDMABuffer(NUMBER_OF_BLOCK * 8 * 2); // the buffers for the
    DMABuffersTampon[1] = allocateDMABuffer(NUMBER_OF_BLOCK * 8 * 2);
    DMABuffersTampon[2] = allocateDMABuffer(START_FRAME_SIZE * 8 * 16 * 2 * 2);
    DMABuffersTampon[3] = allocateDMABuffer((NUM_LEDS_PER_STRIP * 2));
    memset(DMABuffersTampon[3]->buffer, 255, (NUM_LEDS_PER_STRIP * 2));

#ifdef FULL_DMA_BUFFER
    /*
     We do create n+2 buffers
     the first buffer is to be sure that everything is 0
     the last one is to put back the I2S at 0 the last bufffer is longer because
     when using the loop display mode the time between two frames needs to be
     longh enough.
     */
    DMABuffersTransposed = (I2SClockBasedLedDriverDMABuffer **)malloc(
        sizeof(I2SClockBasedLedDriverDMABuffer *) * (num_led_per_strip + 2));
    for (int i = 0; i < num_led_per_strip + 2; i++) {
      if (i > 0 and i < num_led_per_strip + 1)
        DMABuffersTransposed[i] = allocateDMABuffer(NUMBER_OF_BLOCK * 8 * 2);
      else {
        DMABuffersTransposed[i] = allocateDMABuffer(NUM_LEDS_PER_STRIP * 2);
        memset(DMABuffersTransposed[i]->buffer, 255, NUM_LEDS_PER_STRIP * 2);
      }
      if (i == 0)
        DMABuffersTransposed[i] =
            allocateDMABuffer(START_FRAME_SIZE * 8 * 2 * 16);
      if (i < num_led_per_strip)
        DMABuffersTransposed[i]->descriptor.eof = 0;
      if (i) {
        DMABuffersTransposed[i - 1]->descriptor.qe.stqe_next =
            &(DMABuffersTransposed[i]->descriptor);
        if (i < num_led_per_strip + 1) {
          // putdefaultones((uint16_t *)DMABuffersTransposed[i]->buffer);
        }
      }
    }
#endif
  }

#ifdef FULL_DMA_BUFFER

  void stopDisplayLoop() {
    DMABuffersTransposed[num_led_per_strip + 1]->descriptor.qe.stqe_next = 0;
  }

  void showPixelsFromBuffer() { showPixelsFromBuffer(NO_WAIT); }

  void showPixelsFromBuffer(displayMode dispmode) {
    /*
     We cannot launch twice when in loopmode
     */
    if (__displayMode == LOOP && isDisplaying) {
      ESP_LOGE(TAG,
               "The loop mode is activated execute stopDisplayLoop() first");
      return;
    }
    /*
     We wait for the display to be stopped before launching a new one
     */
    if (__displayMode == NO_WAIT && isDisplaying == true)
      xSemaphoreTake(I2SClockBasedLedDriver_semDisp, portMAX_DELAY);
    __displayMode = dispmode;
    isWaiting = false;
    if (dispmode == LOOP or dispmode == LOOP_INTERUPT) {
      DMABuffersTransposed[num_led_per_strip + 1]->descriptor.qe.stqe_next =
          &(DMABuffersTransposed[0]->descriptor);
    }
    transpose = false;
    i2sStart(DMABuffersTransposed[0]);

    if (dispmode == WAIT) {
      isWaiting = true;
      xSemaphoreTake(I2SClockBasedLedDriver_sem, portMAX_DELAY);
    }
  }

  void showPixelsFirstTranspose(OffsetDisplay offdisp) {
    _offsetDisplay = offdisp;
    showPixelsFirstTranspose();
    _offsetDisplay = _defaultOffsetDisplay;
  }

  void showPixelsFirstTranspose(OffsetDisplay offdisp, uint8_t *temp_leds) {
    _offsetDisplay = offdisp;
    showPixelsFirstTranspose(temp_leds);
    _offsetDisplay = _defaultOffsetDisplay;
  }

  void showPixelsFirstTranspose(uint8_t *new_leds) {
    uint8_t *tmp_leds;
    tmp_leds = leds;
    leds = new_leds;
    showPixelsFirstTranspose();
    leds = tmp_leds;
  }
  void showPixelsFirstTranspose() { showPixelsFirstTranpose(NO_WAIT); }
  void showPixelsFirstTranpose(displayMode dispmode) {
    if (leds == NULL) {
      printf("no leds buffer defined");
      return;
    }
    transposeAll();
    showPixelsFromBuffer(dispmode);
  }

  void transposeAll() {
    ledToDisplay = 0;
    // Lines secondPixel[NUMBER_OF_BLOCK];
    for (int j = 0; j < NUM_LEDS_PER_STRIP; j++) {
      /*uint8_t *poli = leds + ledToDisplay * NUMBER_OF_BLOCK;
      for (int i = 0; i < NUMSTRIPS; i++)
      {

          secondPixel[p_g].bytes[i] = __green_map[*(poli + 1)];
          secondPixel[p_r].bytes[i] = __red_map[*(poli + 0)];
          secondPixel[p_b].bytes[i] = __blue_map[*(poli + 2)];
          if (nb_components > 3)
              secondPixel[3].bytes[i] = __white_map[*(poli + 3)];
          //#endif
          poli += num_led_per_strip * nb_components;
      }
      ledToDisplay++;
      transpose16x1_noinline2(secondPixel[0].bytes, (uint16_t
      *)DMABuffersTransposed[j + 1]->buffer);
      transpose16x1_noinline2(secondPixel[1].bytes, (uint16_t
      *)DMABuffersTransposed[j + 1]->buffer + 3 * 8);
      transpose16x1_noinline2(secondPixel[2].bytes, (uint16_t
      *)DMABuffersTransposed[j + 1]->buffer + 2 * 3 * 8); if (nb_components > 3)
          transpose16x1_noinline2(secondPixel[3].bytes, (uint16_t
      *)DMABuffersTransposed[j + 1]->buffer + 3 * 3 * 8);
          */
      loadAndTranspose(leds, num_led_per_strip, num_strips, _offsetDisplay,
                       DMABuffersTransposed[j + 1]->buffer, ledToDisplay,
                       __green_map, __red_map, __blue_map, __white_map,
                       nb_components, p_g, p_r, p_b, _brightness);
      ledToDisplay++;
    }
  }

  void initled(int *Pinsq, int num_strips, int num_led_per_strip,
               colorarrangment cArr) {
    initled(NULL, Pinsq, num_strips, num_led_per_strip, cArr);
  }
  void waitSync() {
    xSemaphoreTake(I2SClockBasedLedDriver_semSync, portMAX_DELAY);
  }
#endif

  /*
Show pixels classiques
*/
  void showPixels(uint8_t *newleds) {
    uint8_t *tmp_leds;
    tmp_leds = leds;
    leds = newleds;
    showPixels();
    leds = tmp_leds;
  }

  void showPixels() {
    if (num_strips == 0) {
      return;
    }

    if (leds == NULL) {
      ESP_LOGE(TAG, "no leds buffer defined");
      return;
    }
    ledToDisplay = 0;
    transpose = true;
    DMABuffersTampon[0]->descriptor.qe.stqe_next =
        &(DMABuffersTampon[1]->descriptor);
    DMABuffersTampon[1]->descriptor.qe.stqe_next =
        &(DMABuffersTampon[0]->descriptor);
    DMABuffersTampon[2]->descriptor.qe.stqe_next =
        &(DMABuffersTampon[0]->descriptor);
    DMABuffersTampon[3]->descriptor.qe.stqe_next = 0;
    dmaBufferActive = 0;
    loadAndTranspose(leds, num_led_per_strip, num_strips, _offsetDisplay,
                     DMABuffersTampon[0]->buffer, ledToDisplay, p_g, p_r, p_b);

    dmaBufferActive = 1;
    i2sStart(DMABuffersTampon[2]);

    isWaiting = true;
    xSemaphoreTake(I2SClockBasedLedDriver_sem, portMAX_DELAY);
  }

  void initled(uint8_t *leds, int *Pinsq, int clock_pin, int num_strips,
               int num_led_per_strip, int clockMHz = 4) {
    dmaBufferCount = 2;
    this->leds = leds;
    this->num_led_per_strip = num_led_per_strip;
    _offsetDisplay.offsetx = 0;
    _offsetDisplay.offsety = 0;
    _offsetDisplay.panel_width = num_led_per_strip;
    _offsetDisplay.panel_height = 9999;
    _defaultOffsetDisplay = _offsetDisplay;
    linewidth = num_led_per_strip;
    this->num_strips = num_strips;
    this->dmaBufferCount = dmaBufferCount;
    setPins(Pinsq, clock_pin);
    if (!_has_i2s_init) {
      i2sInit(clockMHz);
      initDMABuffers();
      _has_i2s_init = true;
    }
  }

  // private:

  // intr_handle_t I2SClockBasedLedDriver_intr_handle;// = NULL;
  //    xSemaphoreHandle I2SClockBasedLedDriver_sem = NULL;
  //   xSemaphoreHandle I2SClockBasedLedDriver_semSync = NULL;
  //   xSemaphoreHandle I2SClockBasedLedDriver_semDisp= NULL;
  // buffer array for the transposed leds
  I2SClockBasedLedDriverDMABuffer **DMABuffersTransposed = NULL;
  // buffer array for the regular way
  I2SClockBasedLedDriverDMABuffer *DMABuffersTampon[4];

  I2SClockBasedLedDriverDMABuffer *allocateDMABuffer(int bytes) {
    I2SClockBasedLedDriverDMABuffer *b =
        (I2SClockBasedLedDriverDMABuffer *)heap_caps_malloc(
            sizeof(I2SClockBasedLedDriverDMABuffer), MALLOC_CAP_DMA);
    if (!b) {
      ESP_LOGE(TAG, "No more memory\n");
      return NULL;
    }

    b->buffer = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_DMA);
    if (!b->buffer) {
      ESP_LOGE(TAG, "No more memory\n");
      return NULL;
    }
    memset(b->buffer, 0, bytes);

    b->descriptor.length = bytes;
    b->descriptor.size = bytes;
    b->descriptor.owner = 1;
    b->descriptor.sosf = 1;
    b->descriptor.buf = b->buffer;
    b->descriptor.offset = 0;
    b->descriptor.empty = 0;
    b->descriptor.eof = 1;
    b->descriptor.qe.stqe_next = 0;

    return b;
  }

  void i2sReset_DMA() {

    i2s->lc_conf.out_rst = 1;
    i2s->lc_conf.out_rst = 0;
  }

  void i2sReset_FIFO() {

    i2s->conf.tx_fifo_reset = 1;
    i2s->conf.tx_fifo_reset = 0;
  }

  void i2sStop() {

    ets_delay_us(16);

    xSemaphoreGive(I2SClockBasedLedDriver_semDisp);
    esp_intr_disable(_gI2SClocklessDriver_intr_handle);
    i2sReset();

    i2s->conf.tx_start = 0;
    isDisplaying = false;
    /*
     We have finished to display the strips
     */

    // xSemaphoreGive(I2SClockBasedLedDriver_semDisp);
  }

  /*
   Transpose the pixel, as the function is static and all the variables are not
   static or global, we need to provide all of them.
   */

  //    void transpose16x1_noinline2(uint8_t y,uint16_t *B,uint16_t
  //    mask,uint16_t mask2,int stripNumber) {
  //
  //        *((uint16_t*)(B)) =   (*((uint16_t*)(B))& mask) | ((uint16_t)((y &
  //        128)>>7) <<stripNumber);
  //        *((uint16_t*)(B+5)) =   (*((uint16_t*)(B+5))& mask) | ((uint16_t)((y
  //        & 64)>>6) <<stripNumber);
  //        *((uint16_t*)(B+6)) =   (*((uint16_t*)(B+6))& mask) | ((uint16_t)((y
  //        & 32)>>5) <<stripNumber);
  //        *((uint16_t*)(B+11)) =   (*((uint16_t*)(B+11))& mask) |
  //        ((uint16_t)((y& 16)>>4)<<stripNumber);
  //        *((uint16_t*)(B+12)) =   (*((uint16_t*)(B+12))& mask) |
  //        ((uint16_t)((y& 8)>>3) <<stripNumber);
  //        *((uint16_t*)(B+17)) =   (*((uint16_t*)(B+17))& mask) |
  //        ((uint16_t)((y& 4)>>2) <<stripNumber);
  //        *((uint16_t*)(B+18)) =   (*((uint16_t*)(B+18))& mask) |
  //        ((uint16_t)((y& 2)>>1) <<stripNumber);
  //        *((uint16_t*)(B+23)) =   (*((uint16_t*)(B+23))& mask) |
  //        ((uint16_t)(y & 1) <<stripNumber);
  //
  //    }

  void i2sStart(I2SClockBasedLedDriverDMABuffer *startBuffer) {

    i2sReset();
    framesync = false;
    counti = 0;

    i2s->lc_conf.val =
        I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;

    i2s->out_link.addr = (uint32_t) & (startBuffer->descriptor);

    i2s->out_link.start = 1;

    i2s->int_clr.val = i2s->int_raw.val;

    i2s->int_clr.val = i2s->int_raw.val;
    i2s->int_ena.val = 0;

    /*
     If we do not use the regular showpixels, then no need to activate the
     interupt at the end of each pixels
     */
    // if(transpose)
    i2s->int_ena.out_eof = 1;

    i2s->int_ena.out_total_eof = 1;
    esp_intr_enable(_gI2SClocklessDriver_intr_handle);

    // We start the I2S
    i2s->conf.tx_start = 1;

    // Set the mode to indicate that we've started
    isDisplaying = true;
  }

  void IRAM_ATTR i2sReset() {
    i2s->lc_conf.val |=
        I2S_IN_RST_M | I2S_OUT_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
    i2s->lc_conf.val &=
        ~(I2S_IN_RST_M | I2S_OUT_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M);
    i2s->conf.val |= I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M |
                     I2S_TX_FIFO_RESET_M;
    i2s->conf.val &= ~(I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M |
                       I2S_TX_FIFO_RESET_M);
  }

  // static void IRAM_ATTR interruptHandler(void *arg);
};

static void IRAM_ATTR _I2SClockBasedLedDriverinterruptHandler(void *arg) {
  auto *cont = (I2SClockBasedLedDriver *)arg;

#ifdef DO_NOT_USE_INTERUPT
  cont->i2s->int_clr.val = cont->i2s->int_raw.val & 0xFFFFFFC0 | 0x3F;
  return;
#else

  if (GET_PERI_REG_BITS(I2S_INT_ST_REG(cont->_i2s_num), I2S_OUT_EOF_INT_ST_V,
                        I2S_OUT_EOF_INT_ST_S)) {
    cont->framesync = !cont->framesync;

    if (cont->transpose) {
      cont->ledToDisplay++;
      if (cont->ledToDisplay < NUM_LEDS_PER_STRIP) {
        loadAndTranspose(cont->leds, cont->num_led_per_strip, cont->num_strips,
                         cont->_offsetDisplay,
                         cont->DMABuffersTampon[cont->dmaBufferActive]->buffer,
                         cont->ledToDisplay, cont->p_g, cont->p_r, cont->p_b);
        if (cont->ledToDisplay ==
            cont->num_led_per_strip -
                4) // here it's not -1 because it takes time top have the change
                   // into account and it reread the buufer
        {
          cont->DMABuffersTampon[cont->dmaBufferActive]
              ->descriptor.qe.stqe_next =
              &(cont->DMABuffersTampon[3]->descriptor);
        }
        cont->dmaBufferActive = (cont->dmaBufferActive + 1) % 2;
      }
    } else {
      if (cont->framesync) {
        portBASE_TYPE HPTaskAwoken = 0;
        xSemaphoreGiveFromISR(cont->I2SClockBasedLedDriver_semSync,
                              &HPTaskAwoken);
        if (HPTaskAwoken == pdTRUE)
          portYIELD_FROM_ISR();
      }
    }
  }

  if (GET_PERI_REG_BITS(I2S_INT_ST_REG(cont->_i2s_num),
                        I2S_OUT_TOTAL_EOF_INT_ST_V,
                        I2S_OUT_TOTAL_EOF_INT_ST_S)) {

    //        portBASE_TYPE HPTaskAwoken = 0;
    //            xSemaphoreGiveFromISR(((I2SClockBasedLedDriver
    //            *)arg)->I2SClockBasedLedDriver_semDisp, &HPTaskAwoken);
    //            if(HPTaskAwoken == pdTRUE) portYIELD_FROM_ISR();
    cont->i2sStop();
    if (cont->isWaiting) {
      portBASE_TYPE HPTaskAwoken = 0;
      xSemaphoreGiveFromISR(cont->I2SClockBasedLedDriver_sem, &HPTaskAwoken);
      if (HPTaskAwoken == pdTRUE)
        portYIELD_FROM_ISR();
    }
  }
  cont->i2s->int_clr.val = cont->i2s->int_raw.val & 0xFFFFFFC0 | 0x3F;
#endif
}

}; // namespace I2SClockBased
