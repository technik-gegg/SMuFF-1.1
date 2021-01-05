#pragma once

#include "../shared/timers.h"

struct STM32_timer_t {
  HardwareTimer timer;
  void (* serviceFunPtr)(void);
  };
