
#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HPM

#include "AP_Networking_HPM.h"
#include <GCS_MAVLink/GCS.h>

#include <lwip/udp.h>
#include <lwip/ip_addr.h>
#include <lwip/tcpip.h>
#include <lwip/netifapi.h>
#if LWIP_DHCP
#include <lwip/dhcp.h>
#endif
#include <lwip/etharp.h>
#include <lwip/pbuf.h>
#include <netif/ethernet.h>

#include "board.h"
#include "hpm_enet_drv.h"
#include "hpm_lan8742.h"
#include "hpm_l1c_drv.h"
#include <AP_Math/AP_Math.h>
#include <AP_HAL/Semaphores.h>

#include <string.h>

extern const AP_HAL::HAL &hal;

#ifndef HPM_NET_RX_COUNT
#define HPM_NET_RX_COUNT 4U
#endif
#ifndef HPM_NET_TX_COUNT
#define HPM_NET_TX_COUNT 4U
#endif
#ifndef HPM_NET_BUF_SIZE
#define HPM_NET_BUF_SIZE 1536U
#endif

#define HPM_NET_MTU 1500

/*
  Descriptors + packet buffers in NONCACHEABLE AXI (128K region) so ENET DMA
  does not need cache maintain.
 */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_DESC_ADDR_ALIGNMENT)
static enet_rx_desc_t hpm_rx_desc[HPM_NET_RX_COUNT];
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_DESC_ADDR_ALIGNMENT)
static enet_tx_desc_t hpm_tx_desc[HPM_NET_TX_COUNT];
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_BUFF_ADDR_ALIGNMENT)
static uint8_t hpm_rx_buf[HPM_NET_RX_COUNT][HPM_NET_BUF_SIZE];
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_BUFF_ADDR_ALIGNMENT)
static uint8_t hpm_tx_buf[HPM_NET_TX_COUNT][HPM_NET_BUF_SIZE];

static enet_desc_t hpm_enet_desc;
static HAL_Semaphore hpm_tx_sem;

bool AP_Networking_HPM::hw_init()
{
    ENET_Type *const enet = BOARD_ENET_RMII;
    enet_mac_config_t mac_cfg {};
    enet_int_config_t int_cfg {};
    enet_tx_control_config_t tx_ctrl {};

    board_init_enet_pins(enet);
    board_reset_enet_phy(enet);
    if (board_init_enet_rmii_reference_clock(enet, BOARD_ENET_RMII_INT_REF_CLK) != status_success) {
        return false;
    }
    board_init_enet_ptp_clock(enet);

    memset(hpm_rx_desc, 0, sizeof(hpm_rx_desc));
    memset(hpm_tx_desc, 0, sizeof(hpm_tx_desc));
    memset(hpm_rx_buf, 0, sizeof(hpm_rx_buf));
    memset(hpm_tx_buf, 0, sizeof(hpm_tx_buf));
    memset(&hpm_enet_desc, 0, sizeof(hpm_enet_desc));

    hpm_enet_desc.tx_desc_list_head = (enet_tx_desc_t *)core_local_mem_to_sys_address(
        BOARD_RUNNING_CORE, (uint32_t)hpm_tx_desc);
    hpm_enet_desc.rx_desc_list_head = (enet_rx_desc_t *)core_local_mem_to_sys_address(
        BOARD_RUNNING_CORE, (uint32_t)hpm_rx_desc);
    hpm_enet_desc.tx_buff_cfg.buffer = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)hpm_tx_buf);
    hpm_enet_desc.tx_buff_cfg.count = HPM_NET_TX_COUNT;
    hpm_enet_desc.tx_buff_cfg.size = HPM_NET_BUF_SIZE;
    hpm_enet_desc.rx_buff_cfg.buffer = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)hpm_rx_buf);
    hpm_enet_desc.rx_buff_cfg.count = HPM_NET_RX_COUNT;
    hpm_enet_desc.rx_buff_cfg.size = HPM_NET_BUF_SIZE;

    enet_get_default_tx_control_config(enet, &tx_ctrl);
    /* lwIP generates IP/ICMP/TCP/UDP checksums in software — do not also
       insert via ENET CIC (default enet_cic_ip_pseudoheader breaks ICMP). */
    tx_ctrl.cic = enet_cic_disable;
    memcpy(&hpm_enet_desc.tx_control_config, &tx_ctrl, sizeof(tx_ctrl));

    uint8_t mac[6];
    frontend.param.macaddr.get_address(mac);
    mac_cfg.mac_addr_high[0] = (uint32_t(mac[5]) << 8) | mac[4];
    mac_cfg.mac_addr_low[0] = (uint32_t(mac[3]) << 24) | (uint32_t(mac[2]) << 16)
                              | (uint32_t(mac[1]) << 8) | mac[0];
    mac_cfg.valid_max_count = 1;
    mac_cfg.dma_pbl = board_get_enet_dma_pbl(enet);
    mac_cfg.sarc = enet_sarc_replace_mac0;

    /* Polling RX path — leave RX interrupt disabled */
    enet_get_default_interrupt_config(enet, &int_cfg);
    int_cfg.int_enable = 0;

    if (enet_controller_init(enet, enet_inf_rmii, &hpm_enet_desc, &mac_cfg, &int_cfg) != status_success) {
        return false;
    }

    memcpy(activeSettings.macaddr, mac, 6);
    return true;
}

bool AP_Networking_HPM::phy_init()
{
    ENET_Type *const enet = BOARD_ENET_RMII;
    phy_addr = 0xFF;

    for (uint8_t addr = 0; addr < 8; addr++) {
        const uint16_t id1 = enet_read_phy(enet, addr, LAN8742_PHYID1);
        const uint16_t id2 = enet_read_phy(enet, addr, LAN8742_PHYID2);
        if ((id1 == 0xFFFF && id2 == 0xFFFF) || (id1 == 0 && id2 == 0)) {
            continue;
        }
        lan8742_config_t cfg;
        lan8742_basic_mode_default_config(enet, &cfg);
        if (lan8742_basic_mode_init(enet, addr, &cfg)) {
            phy_addr = addr;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "NET: PHY OK id1=0x%04x id2=0x%04x addr=%u",
                          unsigned(id1), unsigned(id2), unsigned(addr));
            return true;
        }
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: LAN8742A not found");
    return false;
}

bool AP_Networking_HPM::init()
{
#if defined(HAL_HPM_NET_RUNTIME) && !HAL_HPM_NET_RUNTIME
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: HPM runtime disabled");
    return false;
#endif

#if AP_NETWORKING_CONTROLS_HOST_IP_SETTINGS_ENABLED
    activeSettings.ip = frontend.get_ip_param();
    activeSettings.nm = frontend.get_netmask_param();
    activeSettings.gw = frontend.get_gateway_param();
#else
    activeSettings.ip = 0;
    activeSettings.nm = 0;
    activeSettings.gw = 0;
#endif
    activeSettings.last_change_ms = AP_HAL::millis();
    link_up = false;
    phy_addr = 0xFF;
    thisif = nullptr;

    /*
      Defer MAC/PHY bring-up to the network thread. PHY power/reset needs
      settle time; doing it in Vehicle::setup() raced USB and failed probe.
     */
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_HPM::thread, void),
                                      "network",
#ifdef HPM_NETWORK_STACK_BYTES
                                      HPM_NETWORK_STACK_BYTES,
#else
                                      4096,
#endif
                                      AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: thread create failed");
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: HPM ENET0 thread started");
    return true;
}

void AP_Networking_HPM::link_up_cb(void *p)
{
#if LWIP_DHCP
    auto *driver = (AP_Networking_HPM *)p;
    if (driver->frontend.get_dhcp_enabled()) {
        dhcp_start(driver->thisif);
    }
#else
    (void)p;
#endif
}

void AP_Networking_HPM::link_down_cb(void *p)
{
#if LWIP_DHCP
    auto *driver = (AP_Networking_HPM *)p;
    if (driver->frontend.get_dhcp_enabled()) {
        dhcp_stop(driver->thisif);
    }
#else
    (void)p;
#endif
}

int8_t AP_Networking_HPM::low_level_output(struct netif *netif, struct pbuf *p)
{
    (void)netif;
    ENET_Type *const enet = BOARD_ENET_RMII;

    if (p == nullptr || p->tot_len > HPM_NET_BUF_SIZE) {
        return ERR_BUF;
    }

    if (!hpm_tx_sem.take(50)) {
        return ERR_TIMEOUT;
    }

    enet_tx_desc_t *dma_tx_desc = hpm_enet_desc.tx_desc_list_cur;
    if (dma_tx_desc == nullptr || dma_tx_desc->tdes0_bm.own != 0) {
        hpm_tx_sem.give();
        return ERR_INPROGRESS;
    }

    uint8_t *buffer = (uint8_t *)(uintptr_t)dma_tx_desc->tdes2_bm.buffer1;
    uint16_t offset = 0;
    for (struct pbuf *q = p; q != nullptr; q = q->next) {
        memcpy(buffer + offset, q->payload, q->len);
        offset = uint16_t(offset + q->len);
    }

    /* +4: include CRC length expected by HPM prepare helper */
    const uint16_t frame_length = uint16_t(offset + 4);
    if (enet_prepare_tx_desc(enet, &hpm_enet_desc.tx_desc_list_cur,
                             &hpm_enet_desc.tx_control_config,
                             frame_length, HPM_NET_BUF_SIZE) == 0) {
        hpm_tx_sem.give();
        return ERR_IF;
    }

    hpm_tx_sem.give();
    return ERR_OK;
}

bool AP_Networking_HPM::low_level_input(struct netif *netif, struct pbuf **pbuf)
{
    (void)netif;
    ENET_Type *const enet = BOARD_ENET_RMII;
    *pbuf = nullptr;

    if (enet_check_received_frame(&hpm_enet_desc.rx_desc_list_cur,
                                  &hpm_enet_desc.rx_frame_info) != 1) {
        enet_rx_resume(enet);
        return false;
    }

    enet_frame_t frame = enet_get_received_frame(&hpm_enet_desc.rx_desc_list_cur,
                                                 &hpm_enet_desc.rx_frame_info);
    const uint16_t len = frame.length;
    if (len == 0 || frame.buffer == 0 || frame.rx_desc == nullptr) {
        enet_rx_resume(enet);
        return true;
    }

    *pbuf = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
    if (*pbuf != nullptr) {
        uint16_t offset = 0;
        enet_rx_desc_t *dma_rx_desc = frame.rx_desc;
        for (uint32_t i = 0; i < frame.seg && offset < len; i++) {
            const uint8_t *src = (const uint8_t *)(uintptr_t)dma_rx_desc->rdes2_bm.buffer1;
            const uint16_t chunk = MIN(uint16_t(len - offset), uint16_t(HPM_NET_BUF_SIZE));
            pbuf_take_at(*pbuf, src, chunk, offset);
            offset = uint16_t(offset + chunk);
            dma_rx_desc = (enet_rx_desc_t *)(uintptr_t)dma_rx_desc->rdes3_bm.next_desc;
        }
    }

    /* Return descriptors to DMA */
    enet_rx_desc_t *dma_rx_desc = frame.rx_desc;
    for (uint32_t i = 0; i < frame.seg; i++) {
        dma_rx_desc->rdes0_bm.own = 1;
        dma_rx_desc = (enet_rx_desc_t *)(uintptr_t)dma_rx_desc->rdes3_bm.next_desc;
    }
    hpm_enet_desc.rx_frame_info.seg_count = 0;
    enet_rx_resume(enet);
    return true;
}

int8_t AP_Networking_HPM::ethernetif_init(struct netif *netif)
{
    netif->state = nullptr;
    netif->name[0] = 'e';
    netif->name[1] = '0';
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;
    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    netif->mtu = HPM_NET_MTU;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
#if LWIP_IGMP
    netif->flags |= NETIF_FLAG_IGMP;
#endif
    return ERR_OK;
}

void AP_Networking_HPM::poll_link()
{
    if (phy_addr == 0xFF || thisif == nullptr) {
        return;
    }

    enet_phy_status_t st {};
    lan8742_get_phy_status(BOARD_ENET_RMII, phy_addr, &st);
    const bool up = st.enet_phy_link;

    if (up == link_up) {
        return;
    }
    link_up = up;

    if (up) {
        const uint8_t speed = (st.enet_phy_speed == enet_phy_port_speed_10mbps) ? 10 : 100;
        enet_set_line_speed(BOARD_ENET_RMII,
                            speed == 10 ? enet_line_speed_10mbps : enet_line_speed_100mbps);
        enet_set_duplex_mode(BOARD_ENET_RMII,
                             st.enet_phy_duplex ? enet_full_duplex : enet_half_duplex);
        tcpip_callback_with_block((tcpip_callback_fn)netif_set_link_up, thisif, 0);
        tcpip_callback_with_block(link_up_cb, this, 0);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: link UP %uMbps %s",
                      unsigned(speed), st.enet_phy_duplex ? "full" : "half");
    } else {
        tcpip_callback_with_block((tcpip_callback_fn)netif_set_link_down, thisif, 0);
        tcpip_callback_with_block(link_down_cb, this, 0);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NET: link DOWN");
    }
}

void AP_Networking_HPM::thread()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }

    /* Allow ETH PHY rail (PC13) and LAN8742A to settle after reset */
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: waiting PHY settle");
    hal.scheduler->delay(3000);

    if (!hw_init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: ENET MAC init failed");
        for (;;) {
            hal.scheduler->delay(5000);
        }
    }
    if (!phy_init()) {
        for (;;) {
            hal.scheduler->delay(5000);
        }
    }

    thisif = NEW_NOTHROW netif;
    if (thisif == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: netif alloc failed");
        for (;;) {
            hal.scheduler->delay(5000);
        }
    }

    tcpip_init(NULL, NULL);

    frontend.param.macaddr.get_address(thisif->hwaddr);

    ip4_addr_t ip {}, gateway {}, netmask {};
    if (!frontend.get_dhcp_enabled()) {
        ip.addr = htonl(frontend.get_ip_param());
        gateway.addr = htonl(frontend.get_gateway_param());
        netmask.addr = htonl(frontend.get_netmask_param());
    }

    const err_t result = netifapi_netif_add(thisif, &ip, &netmask, &gateway,
                                            nullptr, ethernetif_init, tcpip_input);
    if (result != ERR_OK) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: netif_add failed %d", int(result));
        for (;;) {
            hal.scheduler->delay(5000);
        }
    }

    netifapi_netif_set_default(thisif);
    netifapi_netif_set_up(thisif);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: HPM ENET0+lwIP ready");

    uint32_t last_link_ms = 0;
    while (true) {
        const uint32_t now = AP_HAL::millis();
        if (now - last_link_ms >= 1000) {
            last_link_ms = now;
            poll_link();
        }

        if (thisif == nullptr) {
            hal.scheduler->delay(100);
            continue;
        }

        struct pbuf *p = nullptr;
        uint8_t budget = 16;
        while (budget-- && low_level_input(thisif, &p)) {
            if (p == nullptr) {
                continue;
            }
            if (thisif->input(p, thisif) != ERR_OK) {
                pbuf_free(p);
            }
            p = nullptr;
        }

        if (budget == 0) {
            hal.scheduler->delay_microseconds(100);
        } else {
            hal.scheduler->delay_microseconds(500);
        }
    }
}

void AP_Networking_HPM::update()
{
    uint32_t ip = activeSettings.ip;
    if (thisif != nullptr) {
        ip = ntohl(thisif->ip_addr.addr);
        const uint32_t nm = ntohl(thisif->netmask.addr);
        const uint32_t gw = ntohl(thisif->gw.addr);

        if (ip != activeSettings.ip ||
            nm != activeSettings.nm ||
            gw != activeSettings.gw) {
            activeSettings.ip = ip;
            activeSettings.gw = gw;
            activeSettings.nm = nm;
            activeSettings.last_change_ms = AP_HAL::millis();
        }
    }

    /* Boot STATUSTEXT is often missed before USB GCS connects — refresh periodically */
    static uint32_t last_status_ms;
    const uint32_t now = AP_HAL::millis();
    if (now - last_status_ms >= 30000) {
        last_status_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NET: %s phy=%u if=%u ip=%u.%u.%u.%u",
                      link_up ? "UP" : "DOWN",
                      unsigned(phy_addr),
                      unsigned(thisif != nullptr),
                      unsigned((ip >> 24) & 0xFF),
                      unsigned((ip >> 16) & 0xFF),
                      unsigned((ip >> 8) & 0xFF),
                      unsigned(ip & 0xFF));
    }
}

#endif // AP_NETWORKING_BACKEND_HPM
