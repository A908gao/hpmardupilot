
#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HPM

#include "AP_Networking_Backend.h"

/*
  HPM6750 ENET0 (RMII + LAN8742A) + lwIP backend.
 */
class AP_Networking_HPM : public AP_Networking_Backend
{
public:
    using AP_Networking_Backend::AP_Networking_Backend;

    CLASS_NO_COPY(AP_Networking_HPM);

    bool init() override;
    void update() override;

private:
    void thread();
    bool hw_init();
    bool phy_init();
    void poll_link();

    static void link_up_cb(void *p);
    static void link_down_cb(void *p);
    static int8_t ethernetif_init(struct netif *netif);
    static int8_t low_level_output(struct netif *netif, struct pbuf *p);
    static bool low_level_input(struct netif *netif, struct pbuf **pbuf);

    struct netif *thisif;
    uint8_t phy_addr;
    bool link_up;
};

#endif // AP_NETWORKING_BACKEND_HPM
