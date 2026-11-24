/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Runtime heap/stack gates so new features cannot silently starve existing tasks.
 */
#pragma once

#include <stdint.h>

#ifndef HAL_HPM_HEAP_WARN_BYTES
#define HAL_HPM_HEAP_WARN_BYTES        (64U * 1024U)
#endif
#ifndef HAL_HPM_HEAP_CRIT_BYTES
#define HAL_HPM_HEAP_CRIT_BYTES        (32U * 1024U)
#endif
/* FreeRTOS high-water mark is in words; warn when remaining < this. */
#ifndef HAL_HPM_STACK_WARN_WORDS
#define HAL_HPM_STACK_WARN_WORDS       128U
#endif
#ifndef HAL_HPM_STACK_CRIT_WORDS
#define HAL_HPM_STACK_CRIT_WORDS       64U
#endif
/* After system init, refuse optional bring-up if free heap below this. */
#ifndef HAL_HPM_HEAP_BOOT_MIN_BYTES
#define HAL_HPM_HEAP_BOOT_MIN_BYTES    (96U * 1024U)
#endif

namespace HPMicro {

class MemGuard {
public:
    /* Call ~1Hz from IO thread once GCS/scheduler are up. */
    static void periodic();

    /* Log after a successful xTaskCreate / thread_create. */
    static void note_task_created(const char *name, uint32_t stack_words);

    /* True if free heap is above the boot floor. */
    static bool boot_heap_ok();
};

}  // namespace HPMicro
