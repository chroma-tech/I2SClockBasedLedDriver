

/*

 */

#pragma once

#include "esp_heap_caps.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "rom/lldesc.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <rom/ets_sys.h>
#include <math.h>

#include "ledtypes.h"
#include "ledcommon.h"

#include "I2SClockBasedLedDriver.h"
#include "I2SClocklessLedDriver.h"

using I2SClockBasedLedDriver = I2SClockBased::I2SClockBasedLedDriver;
using I2SClocklessLedDriver = I2SClockless::I2SClocklessLedDriver;
