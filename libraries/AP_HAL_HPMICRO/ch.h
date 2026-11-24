/*
 * Minimal FreeRTOS shims for ChibiOS APIs used by AP_IOMCU.
 * Only the event-wait/signal surface is provided.
 */
#pragma once

#include <stdint.h>
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"

#ifndef EVENT_MASK
#define EVENT_MASK(n) ((eventmask_t)(1U << (n)))
#endif

// Completes the incomplete type declared in AP_IOMCU.h
struct ch_thread {
    EventGroupHandle_t eg;
};

static inline uint32_t chTimeMS2I(uint32_t ms)
{
    return pdMS_TO_TICKS(ms);
}

static inline thread_t *chThdGetSelfX(void)
{
    // IOMCU owns a single dedicated thread; one event group is enough.
    static thread_t self;
    if (self.eg == nullptr) {
        self.eg = xEventGroupCreate();
    }
    return &self;
}

static inline void chEvtSignal(thread_t *tp, eventmask_t mask)
{
    if (tp == nullptr || tp->eg == nullptr || mask == 0) {
        return;
    }
    if (xPortIsInsideInterrupt()) {
        BaseType_t woken = pdFALSE;
        xEventGroupSetBitsFromISR(tp->eg, mask, &woken);
        portYIELD_FROM_ISR(woken);
    } else {
        xEventGroupSetBits(tp->eg, mask);
    }
}

static inline eventmask_t chEvtWaitAnyTimeout(eventmask_t mask, uint32_t timeout)
{
    thread_t *self = chThdGetSelfX();
    if (self->eg == nullptr) {
        return 0;
    }
    return (eventmask_t)xEventGroupWaitBits(self->eg, mask, pdTRUE, pdFALSE, timeout);
}
