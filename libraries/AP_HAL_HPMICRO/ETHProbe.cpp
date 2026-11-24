/*
 * Ethernet bring-up probe for fmuv6x_hpm (ENET0 RMII + LAN8742A).
 */
#include "ETHProbe.h"

#if HAL_HPM_ETH_PROBE

#include "Scheduler.h"
#include <GCS_MAVLink/GCS.h>
#include "board.h"
#include "hpm_enet_drv.h"
#include "hpm_lan8742.h"
#include "hpm_l1c_drv.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

extern const AP_HAL::HAL &hal;

namespace HPMicro
{

enum class EthReport : uint8_t {
    None = 0,
    Started,
    PhyOk,
    PhyFail,
    LinkUp,
    LinkDown,
    InitFail,
};

struct EthStatus {
    volatile EthReport kind;
    volatile uint16_t id1;
    volatile uint16_t id2;
    volatile uint8_t phy_addr;
    volatile uint8_t speed;
    volatile uint8_t duplex;
};

static EthStatus eth_st {};

#define ETH_PROBE_RX_COUNT 2
#define ETH_PROBE_TX_COUNT 2
#define ETH_PROBE_BUF_SIZE 256

ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_DESC_ADDR_ALIGNMENT)
static enet_rx_desc_t eth_rx_desc[ETH_PROBE_RX_COUNT];
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ENET_SOC_DESC_ADDR_ALIGNMENT)
static enet_tx_desc_t eth_tx_desc[ETH_PROBE_TX_COUNT];
ATTR_ALIGN(HPM_L1C_CACHELINE_SIZE)
static uint8_t eth_rx_buf[ETH_PROBE_RX_COUNT][ETH_PROBE_BUF_SIZE];
ATTR_ALIGN(HPM_L1C_CACHELINE_SIZE)
static uint8_t eth_tx_buf[ETH_PROBE_TX_COUNT][ETH_PROBE_BUF_SIZE];

static bool eth_mac_init(void)
{
    ENET_Type *const enet = BOARD_ENET_RMII;
    enet_desc_t desc {};
    enet_mac_config_t mac_cfg {};
    enet_int_config_t int_cfg {};
    enet_tx_control_config_t tx_ctrl {};

    memset(eth_rx_desc, 0, sizeof(eth_rx_desc));
    memset(eth_tx_desc, 0, sizeof(eth_tx_desc));
    memset(eth_rx_buf, 0, sizeof(eth_rx_buf));
    memset(eth_tx_buf, 0, sizeof(eth_tx_buf));

    desc.tx_desc_list_head = (enet_tx_desc_t *)core_local_mem_to_sys_address(
        BOARD_RUNNING_CORE, (uint32_t)eth_tx_desc);
    desc.rx_desc_list_head = (enet_rx_desc_t *)core_local_mem_to_sys_address(
        BOARD_RUNNING_CORE, (uint32_t)eth_rx_desc);
    desc.tx_buff_cfg.buffer = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)eth_tx_buf);
    desc.tx_buff_cfg.count = ETH_PROBE_TX_COUNT;
    desc.tx_buff_cfg.size = ETH_PROBE_BUF_SIZE;
    desc.rx_buff_cfg.buffer = core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)eth_rx_buf);
    desc.rx_buff_cfg.count = ETH_PROBE_RX_COUNT;
    desc.rx_buff_cfg.size = ETH_PROBE_BUF_SIZE;

    enet_get_default_tx_control_config(enet, &tx_ctrl);
    memcpy(&desc.tx_control_config, &tx_ctrl, sizeof(tx_ctrl));

    /* C2:AF:51:03:CF:46 */
    const uint8_t mac[6] = {0xC2, 0xAF, 0x51, 0x03, 0xCF, 0x46};
    mac_cfg.mac_addr_high[0] = (uint32_t(mac[5]) << 8) | mac[4];
    mac_cfg.mac_addr_low[0] = (uint32_t(mac[3]) << 24) | (uint32_t(mac[2]) << 16)
                              | (uint32_t(mac[1]) << 8) | mac[0];
    mac_cfg.valid_max_count = 1;
    mac_cfg.dma_pbl = board_get_enet_dma_pbl(enet);
    mac_cfg.sarc = enet_sarc_replace_mac0;

    enet_get_default_interrupt_config(enet, &int_cfg);

    return enet_controller_init(enet, enet_inf_rmii, &desc, &mac_cfg, &int_cfg) == status_success;
}

void ETHProbe::probe_task(void *arg)
{
    (void)arg;
    thread();
}

void ETHProbe::start(void)
{
    if (xTaskCreate(probe_task, "eth_probe", 4096, nullptr, Scheduler::IO_PRIO, nullptr) != pdPASS) {
        eth_st.kind = EthReport::InitFail;
    }
}

void ETHProbe::service(void)
{
    const EthReport kind = eth_st.kind;
    if (kind == EthReport::None) {
        return;
    }
    eth_st.kind = EthReport::None;

    switch (kind) {
    case EthReport::Started:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ETH: ENET0 RMII LAN8742A bring-up");
        break;
    case EthReport::InitFail:
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ETH: MAC/task init failed");
        break;
    case EthReport::PhyFail:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "ETH: PHY ID fail id1=0x%04x id2=0x%04x addr=%u",
                      unsigned(eth_st.id1), unsigned(eth_st.id2), unsigned(eth_st.phy_addr));
        break;
    case EthReport::PhyOk:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "ETH: PHY OK id1=0x%04x id2=0x%04x addr=%u",
                      unsigned(eth_st.id1), unsigned(eth_st.id2), unsigned(eth_st.phy_addr));
        break;
    case EthReport::LinkUp:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "ETH: link UP %uMbps %s",
                      unsigned(eth_st.speed),
                      eth_st.duplex ? "full" : "half");
        break;
    case EthReport::LinkDown:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ETH: link DOWN");
        break;
    case EthReport::None:
        break;
    }
}

void ETHProbe::thread(void)
{
    hal.scheduler->delay(4000);
    eth_st.kind = EthReport::Started;

    ENET_Type *const enet = BOARD_ENET_RMII;

    board_init_enet_pins(enet);
    board_reset_enet_phy(enet);
    if (board_init_enet_rmii_reference_clock(enet, BOARD_ENET_RMII_INT_REF_CLK) != status_success) {
        eth_st.kind = EthReport::InitFail;
        for (;;) {
            hal.scheduler->delay(5000);
        }
    }
    board_init_enet_ptp_clock(enet);

    if (!eth_mac_init()) {
        eth_st.kind = EthReport::InitFail;
        for (;;) {
            hal.scheduler->delay(5000);
        }
    }

    uint8_t found_addr = 0xFF;
    uint16_t id1 = 0, id2 = 0;
    for (uint8_t addr = 0; addr < 8; addr++) {
        id1 = enet_read_phy(enet, addr, LAN8742_PHYID1);
        id2 = enet_read_phy(enet, addr, LAN8742_PHYID2);
        if ((id1 == 0xFFFF && id2 == 0xFFFF) || (id1 == 0 && id2 == 0)) {
            continue;
        }
        lan8742_config_t cfg;
        lan8742_basic_mode_default_config(enet, &cfg);
        if (lan8742_basic_mode_init(enet, addr, &cfg)) {
            found_addr = addr;
            break;
        }
        if (found_addr == 0xFF) {
            eth_st.id1 = id1;
            eth_st.id2 = id2;
            eth_st.phy_addr = addr;
        }
    }

    if (found_addr == 0xFF) {
        if (eth_st.id1 == 0 && eth_st.id2 == 0) {
            eth_st.id1 = enet_read_phy(enet, BOARD_ENET_RMII_PHY_ADDR, LAN8742_PHYID1);
            eth_st.id2 = enet_read_phy(enet, BOARD_ENET_RMII_PHY_ADDR, LAN8742_PHYID2);
            eth_st.phy_addr = BOARD_ENET_RMII_PHY_ADDR;
        }
        eth_st.kind = EthReport::PhyFail;
        for (;;) {
            hal.scheduler->delay(5000);
            eth_st.kind = EthReport::PhyFail;
        }
    }

    eth_st.phy_addr = found_addr;
    eth_st.id1 = enet_read_phy(enet, found_addr, LAN8742_PHYID1);
    eth_st.id2 = enet_read_phy(enet, found_addr, LAN8742_PHYID2);
    eth_st.kind = EthReport::PhyOk;

    bool last_up = false;
    while (true) {
        enet_phy_status_t st {};
        lan8742_get_phy_status(enet, found_addr, &st);
        const bool up = st.enet_phy_link;
        if (up != last_up) {
            last_up = up;
            if (up) {
                eth_st.speed = (st.enet_phy_speed == enet_phy_port_speed_10mbps) ? 10 : 100;
                eth_st.duplex = st.enet_phy_duplex ? 1 : 0;
                enet_set_line_speed(enet, eth_st.speed == 10 ? enet_line_speed_10mbps : enet_line_speed_100mbps);
                enet_set_duplex_mode(enet, eth_st.duplex ? enet_full_duplex : enet_half_duplex);
                eth_st.kind = EthReport::LinkUp;
            } else {
                eth_st.kind = EthReport::LinkDown;
            }
        }
        hal.scheduler->delay(1000);
    }
}

}

#endif // HAL_HPM_ETH_PROBE
