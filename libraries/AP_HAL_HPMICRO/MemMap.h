/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Memory / stack map for fmuv6x_hpm, aligned with ArduPilot ChibiOS (H743)
 * conventions:
 *   - AP_HAL / ChibiOS stack sizes are in BYTES
 *   - FreeRTOS xTaskCreate depth is in StackType_t WORDS (4 bytes)
 *   - Convert with HPM_STACK_WORDS(bytes)
 *
 * ChibiOS H743 reference (Scheduler.h WA sizes, bytes):
 *   TIMER 1536 | RCOUT 512 | RCIN 1024 | IO 2048 | STORAGE 1024 | MONITOR 1024
 * HPM applies a small margin (see HPM_STACK_MARGIN_BYTES) for FreeRTOS TCB
 * habits and historically tighter ISR/driver nesting on this port.
 *
 * Physical regions (see flash_xip_ardupilot.ld):
 *   CPU1_SLV 512KB  — FreeRTOS ucHeap (task stacks + pvPortMalloc)
 *   AXI_SRAM 768KB  — .bss / .ardupilot image (static objects)
 *   NONCACHE 128KB  — USB/ETH DMA (tinyalloc + ENET desc/bufs)
 *   ILM      256KB  — vectors + hot code (NOT for stacks)
 *   DLM      256KB  — fast data
 */
#pragma once

#include <stdint.h>
#include "FreeRTOS.h"

#ifndef HPM_STACK_MARGIN_BYTES
/* Extra bytes on top of ChibiOS WA when converting to FreeRTOS words. */
#define HPM_STACK_MARGIN_BYTES 512U
#endif

#ifndef HPM_THREAD_EXTRA_BYTES
/* Added in Scheduler::thread_create (AP_HAL dynamic threads). */
#define HPM_THREAD_EXTRA_BYTES 512U
#endif

/* bytes → FreeRTOS stack depth in words */
#define HPM_STACK_WORDS(bytes) \
    (((uint32_t)(bytes) + (uint32_t)HPM_STACK_MARGIN_BYTES + 3U) / sizeof(StackType_t))

/*
 * ChibiOS-equivalent working-area sizes (bytes), then → words for xTaskCreate.
 * MAIN has no separate WA on ChibiOS (uses main thread); HPM uses 8KB like a
 * typical H7 main stack budget.
 */
#ifndef HPM_MAIN_STACK_BYTES
#define HPM_MAIN_STACK_BYTES       8192U
#endif
#ifndef HPM_TIMER_STACK_BYTES
#define HPM_TIMER_STACK_BYTES      3072U   /* ChibiOS 1536 × ~2 */
#endif
#ifndef HPM_RCOUT_STACK_BYTES
#define HPM_RCOUT_STACK_BYTES      1536U   /* ChibiOS 512 + margin */
#endif
#ifndef HPM_RCIN_STACK_BYTES
#define HPM_RCIN_STACK_BYTES       2048U   /* ChibiOS 1024 × 2 */
#endif
#ifndef HPM_IO_STACK_BYTES
#define HPM_IO_STACK_BYTES         4096U   /* ChibiOS 2048 × 2 */
#endif
#ifndef HPM_STORAGE_STACK_BYTES
#define HPM_STORAGE_STACK_BYTES    2048U   /* ChibiOS 1024 × 2 */
#endif
#ifndef HPM_UART_STACK_BYTES
#define HPM_UART_STACK_BYTES       4096U   /* dedicated UART pump stack */
#endif
/*
 * DeviceBus: one thread+stack per SPI/I2C bus (independent of UART/CAN/NET).
 * Keep 8KB — previously faulted when undersized.
 */
#ifndef HPM_DEVICE_STACK_BYTES
#define HPM_DEVICE_STACK_BYTES     8192U
#endif

#define HPM_MAIN_STACK_WORDS       HPM_STACK_WORDS(HPM_MAIN_STACK_BYTES)
#define HPM_TIMER_STACK_WORDS      HPM_STACK_WORDS(HPM_TIMER_STACK_BYTES)
#define HPM_RCOUT_STACK_WORDS      HPM_STACK_WORDS(HPM_RCOUT_STACK_BYTES)
#define HPM_RCIN_STACK_WORDS       HPM_STACK_WORDS(HPM_RCIN_STACK_BYTES)
#define HPM_IO_STACK_WORDS         HPM_STACK_WORDS(HPM_IO_STACK_BYTES)
#define HPM_STORAGE_STACK_WORDS    HPM_STACK_WORDS(HPM_STORAGE_STACK_BYTES)
#define HPM_UART_STACK_WORDS       HPM_STACK_WORDS(HPM_UART_STACK_BYTES)
#define HPM_DEVICE_STACK_WORDS     HPM_STACK_WORDS(HPM_DEVICE_STACK_BYTES)

/*
 * Independent protocol stacks (all run at Scheduler::COMM_PRIO so they do not
 * preempt each other; buffers/pools are also separate):
 *   UART     — APM_UART (+ per-port ring buffers)
 *   I2C/SPI  — APM_I2C:n / APM_SPI:n DeviceBus threads
 *   CAN      — apcandrvN (DRONECAN_STACK) + per-driver node pool
 *   Ethernet — network + tcpip_thread + NET_Pn (+ RX/TX rings)
 */
#ifndef HPM_DRONECAN_STACK_BYTES
#define HPM_DRONECAN_STACK_BYTES   4096U   /* ChibiOS default without CANFD */
#endif
#ifndef HPM_NETWORK_STACK_BYTES
#define HPM_NETWORK_STACK_BYTES    4096U
#endif
#ifndef HPM_TCPIP_STACK_BYTES
#define HPM_TCPIP_STACK_BYTES      4096U
#endif
#ifndef HPM_NET_PORT_STACK_BYTES
#define HPM_NET_PORT_STACK_BYTES   2048U   /* was 1300; dedicated per NET_Pn */
#endif
#ifndef HPM_NET_PORT_RX_BYTES
#define HPM_NET_PORT_RX_BYTES      2048U
#endif
#ifndef HPM_NET_PORT_TX_BYTES
#define HPM_NET_PORT_TX_BYTES      2048U
#endif

/* Static pools (not FreeRTOS heap). */
#ifndef HPM_DMA_POOL_BYTES
#define HPM_DMA_POOL_BYTES         (24U * 1024U)  /* tinyalloc in NONCACHEABLE */
#endif
#ifndef HPM_FREERTOS_HEAP_BYTES
#define HPM_FREERTOS_HEAP_BYTES    (512U * 1024U) /* CPU1_SLV ucHeap */
#endif
