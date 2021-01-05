#pragma once

#include "esp32-hal.h"
#include "../shared/timers.h"

struct ESP32_timer_t {
  hw_timer_t * timer;
  portMUX_TYPE mux;
  void (* serviceFunPtr)(void);
  };
