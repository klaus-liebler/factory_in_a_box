//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include "TaskScheduler.h"
#include "main.h"

#include <cstring>

#if defined(STM32G4xx)
#define SCHEDULER_TIMER TIM7
#define SCHEDULER_TIMER_IRQn TIM7_DAC_IRQn
#define SCHEDULER_TIMER_IRQ_HANDLER TIM7_DAC_IRQHandler
#elif defined(STM32L4xx)
#define SCHEDULER_TIMER TIM7
#define SCHEDULER_TIMER_IRQn TIM7_IRQn
#define SCHEDULER_TIMER_IRQ_HANDLER TIM7_IRQHandler
#elif defined(STM32G0xx)
#define SCHEDULER_TIMER TIM7
#define SCHEDULER_TIMER_IRQn TIM7_LPTIM2_IRQn
#define SCHEDULER_TIMER_IRQ_HANDLER TIM7_LPTIM2_IRQHandler
#elif defined(STM32F103xB) || defined(STM32F4xx)
#define SCHEDULER_TIMER TIM3
#define SCHEDULER_TIMER_IRQn TIM3_IRQn
#define SCHEDULER_TIMER_IRQ_HANDLER TIM3_IRQHandler
#endif

#define TIMER SCHEDULER_TIMER


#define _countof(a) (sizeof(a) / sizeof(*(a)))

static TIM_HandleTypeDef schedulerTimerHandle{};
static uint32_t coreClockHz = 0;
static uint32_t lastCycleCount = 0;
static uint64_t accumulatedMicros = 0;

static void enableTimerClock() {
#if defined(STM32L4xx) || defined(STM32G0xx) || defined(STM32G4xx)
    __HAL_RCC_TIM7_CLK_ENABLE();
#elif defined(STM32F103xB) || defined(STM32F4xx)
    __HAL_RCC_TIM3_CLK_ENABLE();
#endif
}

static uint32_t getTimerClockHz() {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
#ifdef RCC_CFGR_PPRE1
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        pclk1 *= 2;
#endif
    return pclk1;
}

static void ensureCycleCounterStarted() {
    if (coreClockHz == 0)
        coreClockHz = SystemCoreClock;

    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        lastCycleCount = 0;
        accumulatedMicros = 0;
    }
}

uint32_t pdMicros() {
    ensureCycleCounterStarted();
    uint32_t cycleCount = DWT->CYCCNT;
    uint32_t deltaCycles = cycleCount - lastCycleCount;
    lastCycleCount = cycleCount;

    accumulatedMicros += (static_cast<uint64_t>(deltaCycles) * 1000000ULL) / coreClockHz;
    return static_cast<uint32_t>(accumulatedMicros);
}

inline static uint32_t timeDifference(uint32_t time, uint32_t now) {
    return time - now;
}

inline static bool hasExpired(uint32_t time, uint32_t now) {
    return (time - now) > 0xfff0000;
}

TaskScheduler Scheduler{};

TaskScheduler::TaskScheduler() : numScheduledTasks(-1) { }

void TaskScheduler::start() {
    numScheduledTasks = 0;

    ensureCycleCounterStarted();

    enableTimerClock();

    uint32_t prescaler = (getTimerClockHz() + 500000) / 1000000;
    schedulerTimerHandle.Instance = TIMER;
    schedulerTimerHandle.Init.Prescaler = prescaler - 1;
    schedulerTimerHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    schedulerTimerHandle.Init.Period = 0xffff;
    schedulerTimerHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    schedulerTimerHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&schedulerTimerHandle);

    // one-pulse mode
    TIMER->CR1 |= TIM_CR1_OPM;
    // disable ARR buffering
    TIMER->CR1 &= ~TIM_CR1_ARPE_Msk;

    __HAL_TIM_CLEAR_IT(&schedulerTimerHandle, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&schedulerTimerHandle, TIM_IT_UPDATE);

    HAL_NVIC_SetPriority(SCHEDULER_TIMER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SCHEDULER_TIMER_IRQn);
}

void TaskScheduler::scheduleTaskAfter(TaskFunction task, uint32_t delay) {
    scheduleTaskAt(task, pdMicros() + delay);
}

void TaskScheduler::scheduleTaskAt(TaskFunction task, uint32_t time) {
    if (numScheduledTasks == -1)
        start();

    if (numScheduledTasks >= static_cast<int>(_countof(scheduledTimes)))
        __builtin_trap();

    // pause timer
    TIMER->CR1 &= ~TIM_CR1_CEN_Msk;
    uint32_t now = pdMicros();

    // find insertion index
    // (tasks are sorted by time but time wraps around)
    int index = 0;
    uint32_t delay = timeDifference(time, now);
    while (index < numScheduledTasks) {
        if (delay < timeDifference(scheduledTimes[index], now))
            break;
        index += 1;
    }

    // move elements after insertion point if needed
    if (index < numScheduledTasks) {
        memmove(&scheduledTimes[index + 1], &scheduledTimes[index],
                sizeof(scheduledTimes[0]) * (numScheduledTasks - index));
        memmove(&scheduledFunctions[index + 1], &scheduledFunctions[index],
                sizeof(scheduledFunctions[0]) * (numScheduledTasks - index));
    }

    numScheduledTasks += 1;

    // set values
    scheduledTimes[index] = time;
    scheduledFunctions[index] = task;

    checkPendingTasks();
}

void TaskScheduler::cancelTask(TaskFunction task) {
    if (numScheduledTasks == -1)
        return;

    // pause timer
    TIMER->CR1 &= ~TIM_CR1_CEN_Msk;

    // find task to remove
    int index;
    for (index = 0; index < numScheduledTasks; index += 1) {
        if (scheduledFunctions[index] == task)
            break;
    }

    if (index < numScheduledTasks) {
        numScheduledTasks -= 1;

        // move remaining elements if needed
        if (index < numScheduledTasks) {
            memmove(&scheduledTimes[index], &scheduledTimes[index + 1],
                    sizeof(scheduledTimes[0]) * (numScheduledTasks - index));
            memmove(&scheduledFunctions[index], &scheduledFunctions[index + 1],
                    sizeof(scheduledFunctions[0]) * (numScheduledTasks - index));
        }
    }

    checkPendingTasks();
}

void TaskScheduler::cancelAllTasks() {
    if (numScheduledTasks == -1)
        return;

    // pause timer
    TIMER->CR1 &= ~TIM_CR1_CEN_Msk;
    numScheduledTasks = 0;
}

void TaskScheduler::checkPendingTasks() {

    uint32_t now = 0;

    while (true) {
        if (numScheduledTasks == 0)
            return; // no pending tasks

        now = pdMicros();
        if (!hasExpired(scheduledTimes[0], now))
            break; // next task has not yet expired

        TaskFunction task = scheduledFunctions[0];
        numScheduledTasks -= 1;

        // move remaining elements if needed
        if (numScheduledTasks > 0) {
            memmove(&scheduledTimes[0], &scheduledTimes[1],
                sizeof(scheduledTimes[0]) * numScheduledTasks);
            memmove(&scheduledFunctions[0], &scheduledFunctions[1],
                sizeof(scheduledFunctions[0]) * numScheduledTasks);
        }

        // execute task
        task();
    }

    uint32_t delayToFirstTask = timeDifference(scheduledTimes[0], pdMicros());
    if (delayToFirstTask > 0xffff)
        delayToFirstTask = 0xffff;

    // restart timer
    TIMER->CNT = 0;
    TIMER->ARR = delayToFirstTask;
    TIMER->CR1 |= TIM_CR1_CEN;
}

void TaskScheduler::onInterrupt() {
    Scheduler.checkPendingTasks();
}

#ifdef SCHEDULER_TIMER_IRQ_HANDLER
extern "C" void SCHEDULER_TIMER_IRQ_HANDLER(void) {
    if (__HAL_TIM_GET_FLAG(&schedulerTimerHandle, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&schedulerTimerHandle, TIM_IT_UPDATE);
        Scheduler.onInterrupt();
    } else {
        __HAL_TIM_CLEAR_IT(&schedulerTimerHandle, TIM_IT_UPDATE);
    }
}
#endif
