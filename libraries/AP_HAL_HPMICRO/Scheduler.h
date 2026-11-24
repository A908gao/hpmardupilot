/*
 * Copyright (c) 2025,2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#pragma once
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include "HAL_HPM_Namespace.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MemMap.h"

#define HPM_SCHEDULER_MAX_TIMER_PROCS 10
#define HPM_SCHEDULER_MAX_IO_PROCS 10

/* Scheduler implementation: */
class HPMicro::Scheduler : public AP_HAL::Scheduler
{

public:
    Scheduler();
    ~Scheduler() {};
    /* AP_HAL::Scheduler methods */
    void     init() override;
    void     set_callbacks(AP_HAL::HAL::Callbacks *cb) { callbacks = cb; };
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void     reboot(bool hold_in_bootloader) override;
    bool     in_main_thread() const override;
    // check and set the startup state
    void     set_system_initialized() override;
    bool     is_system_initialized() override;

    void     print_stats(void) ;
    void     print_main_loop_rate(void);
    uint16_t get_loop_rate_hz(void);
    AP_Int16 _active_loop_rate_hz;
    AP_Int16 _loop_rate_hz;

    static void thread_create_trampoline(void *ctx);
    static void osalSysLock(void);
    static void osalSysUnlock(void);
    bool thread_create(AP_HAL::MemberProc, const char *name, uint32_t stack_size, priority_base base, int8_t priority) override;

    /*
      Priority layout (FreeRTOS: higher number = higher priority):
        MAIN/SPI 24  — flight loop + IMU (may preempt everything below)
        TIMER    23  — high-rate timers
        COMM     18  — UART / I2C / CAN / Ethernet share ONE band so they
                       round-robin and do not preempt each other. Each still
                       has its own task + stack (see MemMap.h).
        RCOUT    10
        RCIN/IO   5
        STORAGE   4
     */
    static const int SPI_PRIORITY = 24;
    static const int MAIN_PRIO    = 24;
    static const int TIMER_PRIO   = 23;
    static const int COMM_PRIO    = 18; /* shared UART+I2C+CAN+NET band */
    static const int I2C_PRIORITY = COMM_PRIO;
    static const int CAN_PRIO     = COMM_PRIO;
    static const int UART_PRIO    = COMM_PRIO;
    static const int WIFI_PRIO1   = COMM_PRIO; /* PRIORITY_NET base */
    static const int WIFI_PRIO2   = COMM_PRIO;
    static const int RCIN_PRIO    = 5;
    static const int RCOUT_PRIO   = 10;
    static const int IO_PRIO      = 5;
    static const int STORAGE_PRIO = 4;

    /* FreeRTOS depth in words — sizes from MemMap.h (ChibiOS byte WA → words). */
    static const int TIMER_SS     = (int)HPM_TIMER_STACK_WORDS;
    static const int MAIN_SS      = (int)HPM_MAIN_STACK_WORDS;
    static const int RCIN_SS      = (int)HPM_RCIN_STACK_WORDS;
    static const int RCOUT_SS     = (int)HPM_RCOUT_STACK_WORDS;
    static const int WIFI_SS1     = (int)HPM_UART_STACK_WORDS; /* legacy */
    static const int WIFI_SS2     = (int)HPM_IO_STACK_WORDS;
    static const int UART_SS      = (int)HPM_UART_STACK_WORDS;
    static const int DEVICE_SS    = (int)HPM_DEVICE_STACK_WORDS;
    static const int IO_SS        = (int)HPM_IO_STACK_WORDS;
    static const int STORAGE_SS   = (int)HPM_STORAGE_STACK_WORDS;

private:
    AP_HAL::HAL::Callbacks *callbacks;
    AP_HAL::Proc _failsafe;

    AP_HAL::MemberProc _timer_proc[HPM_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;

    AP_HAL::MemberProc _io_proc[HPM_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs;

    static bool _initialized;

    tskTaskControlBlock* _main_task_handle;
    tskTaskControlBlock* _timer_task_handle;
    tskTaskControlBlock* _rcin_task_handle;
    tskTaskControlBlock* _rcout_task_handle;
    tskTaskControlBlock* _uart_task_handle;
    tskTaskControlBlock* _io_task_handle;
    tskTaskControlBlock* test_task_handle;
    tskTaskControlBlock* _storage_task_handle;

    static void _main_thread(void *arg);
    static void _timer_thread(void *arg);
    static void _rcout_thread(void *arg);
    static void _rcin_thread(void *arg);
    static void _uart_thread(void *arg);
    static void _io_thread(void *arg);
    static void _storage_thread(void *arg);

    bool _in_timer_proc;
    void _run_timers();
    Semaphore _timer_sem;

    bool _in_io_proc;
    void _run_io();
    Semaphore _io_sem;
};
