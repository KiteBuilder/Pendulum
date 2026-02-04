/*
 * time.cpp
 *
 *  Created on: Dec 20, 2023
 *      Author: Kitebuilder
 */

#include "stm32f4xx_hal.h"
#include "times.h"
#include "atomic.h"

using namespace std;

static volatile timeMs_t sysTickUptime = 0;
static volatile uint32_t sysTickValStamp = 0;

static volatile int sysTickPending = 0;

uint32_t usTicks = 0;

/**
  * @brief
  * @retval None
  */
void HAL_SYSTICK_Callback()
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX)
    {
        sysTickUptime++;
        sysTickValStamp = SysTick->VAL;
        sysTickPending = 0;
        (void)(SysTick->CTRL);
    }
}

/**
  * @brief Return system uptime in microseconds
  * @retval None
  */
timeUs_t microsISR(void)
{
    uint32_t ms, pending, cycle_cnt;

    ATOMIC_BLOCK(NVIC_PRIO_MAX)
    {
        cycle_cnt = SysTick->VAL;

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            // Update pending.
            // Record it for multiple calls within the same rollover period
            // (Will be cleared when serviced).
            // Note that multiple rollovers are not considered.

            sysTickPending = 1;

            // Read VAL again to ensure the value is read after the rollover.

            cycle_cnt = SysTick->VAL;
        }

        ms = sysTickUptime;
        pending = sysTickPending;
    }

    const uint32_t partial = (usTicks * 1000U - cycle_cnt) / usTicks;
    return ((timeUs_t)(ms + pending) * 1000LL) + ((timeUs_t)partial);
}

/**
  * @brief Return system uptime in microseconds
  * @retval None
  */
timeUs_t micros(void)
{
    volatile uint32_t ms, cycle_cnt;

    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI())) {
        return microsISR();
    }

    do
    {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime || cycle_cnt > sysTickValStamp);

    const uint32_t partial = (usTicks * 1000U - cycle_cnt) / usTicks;
    return ((timeUs_t)ms * 1000LL) + ((timeUs_t)partial);
}

/**
  * @brief Delay in microseconds
  * @param us: time in microseconds
  * @retval None
  */
void delayUs(timeUs_t us)
{
    timeUs_t now = micros();
    while (micros() - now < us);
}

/**
  * @brief Delay in milliseconds
  * @param ms: time in mlliseconds
  * @retval None
  */
void delayMs(timeMs_t ms)
{
    while (ms--)
        delayUs(1000);
}
