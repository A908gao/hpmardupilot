# Copyright (c) 2025-2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

set(ENV{CONFIG_PANEL_RGB_TM070RDH13} 1)
sdk_compile_definitions_ifdef(BUILD_FOR_SECONDARY_CORE BOARD_RUNNING_CORE=HPM_CORE1)

sdk_inc(.)
sdk_src(pinmux.c)
sdk_src(board.c)

# fmuv6x_hpm: single ENET0 RMII port with LAN8742A
set(APP_USE_ENET_PORT_COUNT 1)
set(APP_USE_ENET_ITF_RMII 1)
set(ENV{CONFIG_ENET_PHY_LAN8742} 1)
sdk_compile_definitions(-DRMII=1)
sdk_compile_definitions(-D__USE_LAN8742=1)

if(APP_USE_ENET_PORT_COUNT EQUAL 2)
    set(ENV{CONFIG_ENET_PHY_RTL8211} 1)
    set(ENV{CONFIG_ENET_PHY_RTL8201} 1)
elseif(APP_USE_ENET_PORT_COUNT EQUAL 1)
    if(APP_USE_ENET_ITF_RGMII)
        set(ENV{CONFIG_ENET_PHY_RTL8211} 1)
        sdk_compile_definitions(-DRGMII=1)
    elseif(APP_USE_ENET_ITF_RMII)
        # already set LAN8742 above
        sdk_compile_definitions(-DRMII=1)
    endif()
endif()
