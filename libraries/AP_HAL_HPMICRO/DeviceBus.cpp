/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "DeviceBus.h"

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

#include "Scheduler.h"
#include "Semaphores.h"
#include "MemGuard.h"

#include "FreeRTOS.h"
#include "task.h"

// #define BUSDEBUG    1
using namespace HPMicro;
static const AP_HAL::HAL& hpmhal = AP_HAL::get_HAL();

DeviceBus::DeviceBus(uint8_t _thread_priority) :
    semaphore(),
    thread_priority(_thread_priority)
{
#ifdef BUSDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
}

/*
  per-bus callback thread
*/
void  DeviceBus::bus_thread(void *arg)
{
#ifdef BUSDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    struct DeviceBus *binfo = (struct DeviceBus *)arg;

    while (true) {
        uint64_t now = AP_HAL::micros64();
        DeviceBus::callback_info *callback;

        // find a callback to run
        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (now >= callback->next_usec) {
                while (now >= callback->next_usec) {
                    callback->next_usec += callback->period_usec;
                }
                // call it with semaphore held
                if (binfo->semaphore.take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
                    callback->cb();
                    binfo->semaphore.give();
                }
            }
        }

        // work out when next loop is needed
        uint64_t next_needed = 0;
        now = AP_HAL::micros64();

        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (next_needed == 0 ||
                callback->next_usec < next_needed) {
                next_needed = callback->next_usec;
                if (next_needed < now) {
                    next_needed = now;
                }
            }
        }

        // delay for at most 50ms, to handle newly added callbacks
        uint32_t delay_us = 50000;
        if (next_needed >= now && next_needed - now < delay_us) {
            delay_us = next_needed - now;
        }
        // don't delay for less than 1ms (FreeRTOS tick); avoid
        // Scheduler::delay_microseconds(uint16_t) busy paths from bus threads
        uint32_t delay_ms = delay_us / 1000U;
        if (delay_ms < 1) {
            delay_ms = 1;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb, AP_HAL::Device *_hal_device)
{
#ifdef BUSDEBUG
    hpmhal.console->printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (!thread_started) {
        thread_started = true;
        hal_device = _hal_device;
        // setup a name for the thread
        char name[configMAX_TASK_NAME_LEN];
        switch (hal_device->bus_type()) {
        case AP_HAL::Device::BUS_TYPE_I2C:
            snprintf(name, sizeof(name), "APM_I2C:%u",
                     hal_device->bus_num());
            break;

        case AP_HAL::Device::BUS_TYPE_SPI:
            snprintf(name, sizeof(name), "APM_SPI:%u",
                     hal_device->bus_num());
            break;
        default:
            break;
        }
#ifdef BUSDEBUG
        hpmhal.console->printf("%s:%d Thread Start\n", __PRETTY_FUNCTION__, __LINE__);
#endif
        /* FreeRTOS stack depth in words (DEVICE_SS). 2048 words overflowed. */
        if (xTaskCreate(DeviceBus::bus_thread, name, Scheduler::DEVICE_SS,
                        this, thread_priority, &bus_thread_handle) != pdPASS) {
            thread_started = false;
            bus_thread_handle = nullptr;
            return nullptr;
        }
        MemGuard::note_task_created(name, Scheduler::DEVICE_SS);
    }
    DeviceBus::callback_info *callback = NEW_NOTHROW DeviceBus::callback_info;
    if (callback == nullptr) {
        return nullptr;
    }
    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    // add to linked list of callbacks on thread
    callback->next = callbacks;
    callbacks = callback;

    return callback;
}

/*
 * Adjust the timer for the next call: it needs to be called from the bus
 * thread, otherwise it will race with it
 */
bool DeviceBus::adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    if (xTaskGetCurrentTaskHandle() != bus_thread_handle) {
        return false;
    }
    DeviceBus::callback_info *callback = static_cast<DeviceBus::callback_info *>(h);
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;
    return true;
}
