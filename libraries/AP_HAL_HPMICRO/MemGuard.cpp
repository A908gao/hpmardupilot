/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "MemGuard.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

namespace HPMicro {

static uint32_t last_heap_warn_ms;
static uint32_t last_stack_warn_ms;
static bool boot_announced;

bool MemGuard::boot_heap_ok()
{
    return xPortGetFreeHeapSize() >= HAL_HPM_HEAP_BOOT_MIN_BYTES;
}

void MemGuard::note_task_created(const char *name, uint32_t stack_words)
{
    const uint32_t free_b = xPortGetFreeHeapSize();
    if (hal.console) {
        hal.console->printf("MemGuard: +task %s stack=%luW free=%lu min=%lu\n",
                            name ? name : "?",
                            (unsigned long)stack_words,
                            (unsigned long)free_b,
                            (unsigned long)xPortGetMinimumEverFreeHeapSize());
    }
    if (free_b < HAL_HPM_HEAP_WARN_BYTES) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "MemGuard: low heap after %s free=%u",
                      name ? name : "?", unsigned(free_b));
    }
}

void MemGuard::periodic()
{
    const uint32_t now = AP_HAL::millis();
    const uint32_t free_b = xPortGetFreeHeapSize();
    const uint32_t min_b = xPortGetMinimumEverFreeHeapSize();

    if (!boot_announced && now > 5000) {
        boot_announced = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "MemGuard: heap free=%u min=%u total=%u",
                      unsigned(free_b), unsigned(min_b),
                      unsigned(configTOTAL_HEAP_SIZE));
        if (!boot_heap_ok()) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                          "MemGuard: boot heap %u < %u — do not add features",
                          unsigned(free_b), unsigned(HAL_HPM_HEAP_BOOT_MIN_BYTES));
        }
    }

    if (free_b < HAL_HPM_HEAP_CRIT_BYTES || min_b < HAL_HPM_HEAP_CRIT_BYTES) {
        if (now - last_heap_warn_ms > 10000) {
            last_heap_warn_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                          "MemGuard: heap CRIT free=%u min=%u",
                          unsigned(free_b), unsigned(min_b));
        }
    } else if (free_b < HAL_HPM_HEAP_WARN_BYTES || min_b < HAL_HPM_HEAP_WARN_BYTES) {
        if (now - last_heap_warn_ms > 30000) {
            last_heap_warn_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "MemGuard: heap low free=%u min=%u",
                          unsigned(free_b), unsigned(min_b));
        }
    }

    /* Stack high-water scan (words remaining). Skip if called too often. */
    if (now - last_stack_warn_ms < 5000) {
        return;
    }

    const UBaseType_t ntasks = uxTaskGetNumberOfTasks();
    if (ntasks == 0 || ntasks > 64) {
        return;
    }
    TaskStatus_t status[64];
    configRUN_TIME_COUNTER_TYPE runtime = 0;
    const UBaseType_t got = uxTaskGetSystemState(status, ntasks, &runtime);
    const char *worst_name = nullptr;
    UBaseType_t worst_hwm = UINT32_MAX;
    for (UBaseType_t i = 0; i < got; i++) {
        const UBaseType_t hwm = status[i].usStackHighWaterMark;
        if (hwm < worst_hwm) {
            worst_hwm = hwm;
            worst_name = status[i].pcTaskName;
        }
    }
    if (worst_name == nullptr) {
        return;
    }
    if (worst_hwm < HAL_HPM_STACK_CRIT_WORDS) {
        last_stack_warn_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "MemGuard: stack CRIT %s free=%uW",
                      worst_name, unsigned(worst_hwm));
    } else if (worst_hwm < HAL_HPM_STACK_WARN_WORDS) {
        last_stack_warn_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "MemGuard: stack low %s free=%uW",
                      worst_name, unsigned(worst_hwm));
    }
}

}  // namespace HPMicro
