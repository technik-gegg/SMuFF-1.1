/**
 * SMuFF Firmware
 * Copyright (C) 2019-2022 Technik Gegg
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
 * 
 * STM32 code originated from: https://github.com/stm32duino/STM32Examples/examples/Benchmarking/MemoryAllocationStatistics/MemoryAllocationStatistics.ino
 */

#include "Memtest.h"

#if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)

extern "C" char *sbrk(int i);
/* Use linker definition */
extern char _end;
extern char _sdata;
extern char _estack;
extern char _Min_Stack_Size;

static char *ramstart = &_sdata;
static char *ramend = &_estack;
static char *minSP = (char*)(ramend - &_Min_Stack_Size);

void showFreeMemory() {
  char* heapend = (char*)sbrk(0);
  char* stack_ptr = (char*)__get_MSP();
  struct mallinfo mi = mallinfo();
  long free = ((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks;
  long used = &_end - ramstart;
  char ramFree[40];
  char ramUsed[40];

  if(free > 2048)
    snprintf_P(ramFree, ArraySize(ramFree)-1, PSTR("%s KB"), String(((double)free/1024)).c_str());
  else
    snprintf_P(ramFree, ArraySize(ramFree)-1, PSTR("%ld B"), free);
  if(used > 2048)
    snprintf_P(ramUsed, ArraySize(ramUsed)-1, PSTR("%s KB"), String(((double)used/1024)).c_str());
  else
    snprintf_P(ramUsed, ArraySize(ramUsed)-1, PSTR("%ld B"), used);
  __debugS(DEV4, PSTR("Used: [ Heap: %ld B  RAM: %s  Stack: %ld B ]  Available: [ RAM: %s ]"), mi.uordblks, ramUsed, ramend - stack_ptr, ramFree);
}

void getFreeMemory(char* buffer, size_t maxlen) {
  char* heapend = (char*)sbrk(0);
  char* stack_ptr = (char*)__get_MSP();
  struct mallinfo mi = mallinfo();
  long free = ((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks;

  snprintf_P(buffer, maxlen, P_FreeMemory, free);
}

#elif defined(__ESP32__)
void showFreeMemory() {
  __debugS(DEV4, PSTR("Heap: %ld, Fragmentation: %d"), ESP.getFreeHeap(), ESP.getHeapFragmentation());
}

void getFreeMemory(char* buffer, size_t maxlen) {
  snprintf_P(buffer, maxlen, P_FreeMemory, ESP.getFreeHeap());
}
#endif