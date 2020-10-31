#ifdef __AVR__
  typedef unsigned int timerInterval_t;
  typedef uint8_t WiringPinMode;
  typedef unsigned int servotick_t;

  #define noInterrupts  cli
  #define interrupts    sei


  #ifndef INPUT_PULLDOWN
    #define INPUT_PULLDOWN  INPUT
  #endif

#elif defined(__STM32F1__)
  typedef unsigned int timerInterval_t;
  typedef uint32_t servotick_t;

#elif defined(__ESP32__)
  typedef uint64_t timerInterval_t;
  typedef uint8_t WiringPinMode;
  typedef unsigned int servotick_t;
#endif
