//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#include "Timer.h"
#include "cortexm/ExceptionHandlers.h"

// ----------------------------------------------------------------------------

#if defined(USE_HAL_DRIVER)
extern "C" void HAL_IncTick(void);
#endif

// ----------------------------------------------------------------------------

volatile Timer::ticks_t Timer::currentTicks;

// ----------------------------------------------------------------------------

void Timer::sleep(ticks_t sleep_ms)
{
	ticks_t targetTicks = currentTicks + sleep_ms;

  // Busy wait until the SysTick decrements the counter to zero.
  while (currentTicks != targetTicks)
    ;
}

// ----- SysTick_Handler() ----------------------------------------------------

extern "C" void
SysTick_Handler(void)
{
#if defined(USE_HAL_DRIVER)
  HAL_IncTick();
#endif
  Timer::tick();
}

// ----------------------------------------------------------------------------
