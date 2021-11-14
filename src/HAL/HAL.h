/**
 * SMuFF Firmware
 * Copyright (C) 2019-2021 Technik Gegg
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#if defined(__STM32F1__)
  #include "STM32F1/i2c.h"
  #include "STM32F1/timers.h"
#elif defined(__STM32F4__)
  #include "STM32F4/i2c.h"
  #include "STM32F4/timers.h"
#elif defined(__ESP32__)
  #include "ESP32/i2c.h"
  #include "ESP32/timers.h"
#endif
