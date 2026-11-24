/*
 * Ethernet bring-up probe: ENET0 RMII + LAN8742A.
 * Verifies MDIO / PHY ID / link without enabling full AP_Networking/lwIP.
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_HPM_ETH_PROBE
#define HAL_HPM_ETH_PROBE 0
#endif

#if HAL_HPM_ETH_PROBE

namespace HPMicro
{

class ETHProbe
{
public:
    static void start(void);
    static void service(void);

private:
    static void probe_task(void *arg);
    static void thread(void);
};

}

#endif
