//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#ifndef TIMER_H_
#define TIMER_H_

#include "cmsis_device.h"
#include <stdint.h>

// ----------------------------------------------------------------------------

class Timer
{
public:
  typedef uint64_t ticks_t;
  static constexpr ticks_t FREQUENCY_HZ = 1000u;

//private:
  static volatile ticks_t currentTicks;

public:
  // Default constructor
  Timer() = default;

  static inline void
  start(void)
  {
    // Use SysTick as reference for the delay loops.
    SysTick_Config(SystemCoreClock / FREQUENCY_HZ);
  }

  inline static ticks_t GetTick(void)
  {
	  return currentTicks;
  }

  static void sleep(ticks_t ticks);

  inline static void tick(void)
  {
	  currentTicks++;
  }
};

// ----------------------------------------------------------------------------

#endif // TIMER_H_
