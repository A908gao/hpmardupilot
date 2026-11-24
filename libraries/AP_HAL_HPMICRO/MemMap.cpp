/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "MemMap.h"
#include "Util.h"
#include <AP_Common/ExpandingString.h>
#include "FreeRTOS.h"
#include "task.h"

using namespace HPMicro;

namespace {

struct StackRow {
    const char *name;
    const char *region;
    uint32_t bytes;
    uint32_t words;
    const char *note;
};

}  // namespace

void Util::mem_map_info(ExpandingString &str)
{
    const StackRow rows[] = {
        {"APM_MAIN",     "CPU1_SLV/ucHeap", HPM_MAIN_STACK_BYTES,    HPM_MAIN_STACK_WORDS,    "ChibiOS main~8KB"},
        {"APM_TIMER",    "CPU1_SLV/ucHeap", HPM_TIMER_STACK_BYTES,   HPM_TIMER_STACK_WORDS,   "ChibiOS WA 1536×2"},
        {"APM_RCOUT",    "CPU1_SLV/ucHeap", HPM_RCOUT_STACK_BYTES,   HPM_RCOUT_STACK_WORDS,   "ChibiOS WA 512+"},
        {"APM_RCIN",     "CPU1_SLV/ucHeap", HPM_RCIN_STACK_BYTES,    HPM_RCIN_STACK_WORDS,    "ChibiOS WA 1024×2"},
        {"APM_UART",     "CPU1_SLV/ucHeap", HPM_UART_STACK_BYTES,    HPM_UART_STACK_WORDS,    "UART stack @COMM_PRIO"},
        {"APM_IO",       "CPU1_SLV/ucHeap", HPM_IO_STACK_BYTES,      HPM_IO_STACK_WORDS,      "ChibiOS WA 2048×2"},
        {"APM_STORAGE",  "CPU1_SLV/ucHeap", HPM_STORAGE_STACK_BYTES, HPM_STORAGE_STACK_WORDS, "ChibiOS WA 1024×2"},
        {"APM_I2C/SPI",  "CPU1_SLV/ucHeap", HPM_DEVICE_STACK_BYTES,  HPM_DEVICE_STACK_WORDS,  "I2C@COMM SPI@24/bus"},
        {"DroneCAN×N",   "CPU1_SLV/ucHeap", HPM_DRONECAN_STACK_BYTES, HPM_STACK_WORDS(HPM_DRONECAN_STACK_BYTES + HPM_THREAD_EXTRA_BYTES), "CAN stack @COMM_PRIO"},
        {"network",      "CPU1_SLV/ucHeap", HPM_NETWORK_STACK_BYTES, HPM_STACK_WORDS(HPM_NETWORK_STACK_BYTES + HPM_THREAD_EXTRA_BYTES), "ENET @COMM_PRIO"},
        {"tcpip_thread", "CPU1_SLV/ucHeap", HPM_TCPIP_STACK_BYTES,   HPM_STACK_WORDS(HPM_TCPIP_STACK_BYTES + HPM_THREAD_EXTRA_BYTES), "lwIP @COMM_PRIO"},
        {"NET_Pn×4",     "CPU1_SLV/ucHeap", HPM_NET_PORT_STACK_BYTES, HPM_STACK_WORDS(HPM_NET_PORT_STACK_BYTES + HPM_THREAD_EXTRA_BYTES), "ETH ports @COMM_PRIO"},
        {"idle/timer",   "CPU1_SLV/ucHeap", 512U * sizeof(StackType_t), 512U, "FreeRTOS minimal"},
        {"DMA tinyalloc","NONCACHEABLE",   HPM_DMA_POOL_BYTES,      0, "MEM_DMA_SAFE"},
        {"ENET RX/TX",   "NONCACHEABLE",   4U * 1536U * 2U,         0, "desc+bufs static"},
        {"ucHeap total", "CPU1_SLV",        HPM_FREERTOS_HEAP_BYTES, 0, "all task stacks+malloc"},
        {"AXI .bss",     "AXI_SRAM",       0,                       0, "statics (link map)"},
    };

    str.printf("HPM memory map (ChibiOS-aligned stacks)\n");
    str.printf("%-14s %-16s %8s %8s  %s\n", "Name", "Region", "Bytes", "Words", "Note");
    for (const auto &r : rows) {
        str.printf("%-14s %-16s %8u %8u  %s\n",
                   r.name, r.region,
                   unsigned(r.bytes), unsigned(r.words), r.note);
    }
    str.printf("live heap free=%u min=%u total=%u\n",
               unsigned(xPortGetFreeHeapSize()),
               unsigned(xPortGetMinimumEverFreeHeapSize()),
               unsigned(configTOTAL_HEAP_SIZE));
}
