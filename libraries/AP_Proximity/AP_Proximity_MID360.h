/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_MID360_ENABLED

#include "AP_Proximity_Backend.h"
#include <AP_HAL/utility/Socket.h>

/// Livox MID360 (Agras radar) proximity backend.
///
/// The Agras radar powers up with IP 192.168.1.10, probes for the host at
/// 192.168.1.20 (the flight controller) and then streams raw point cloud
/// frames to UDP port 60001 and raw IMU frames to UDP port 60003 all by
/// itself — no SDK and no handshake response are required. This backend
/// listens on the point cloud port, decodes the raw Livox ethernet packets
/// in process and feeds horizontal obstacle readings into the 8-sector 3D
/// boundary used by object avoidance.
///
/// Only points between the radar plane and PRXx_MID360_HMAX above it are
/// kept; everything higher is filtered out, leaving the low-altitude ranging
/// slice the avoidance code cares about.
class AP_Proximity_MID360 : public AP_Proximity_Backend
{
public:
    AP_Proximity_MID360(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_Proximity_Params &_params);

    void update() override;

    float distance_max() const override { return 40.0f; }
    float distance_min() const override { return 0.05f; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    // Livox wire-format frame header (full packed definition in the .cpp)
    struct PACKED LivoxLidarEthernetPacket {
        uint8_t version;
        uint16_t length;
        uint16_t time_interval;
        uint16_t dot_num;
        uint16_t udp_cnt;
        uint8_t frame_cnt;
        uint8_t data_type;
        uint8_t time_type;
        uint8_t rsvd[12];
        uint32_t crc32;
        uint8_t timestamp[8];
        uint8_t data[1];
    };

    bool init_socket();
    void handle_datagram(const uint8_t *buf, uint32_t len);
    void handle_packet(const LivoxLidarEthernetPacket *pkt);
    void handle_point(float x_m, float y_m, float z_m);

    SocketAPM _socket{true};    // UDP datagram socket

    uint32_t _last_recv_ms;     // system time of the last valid datagram

    AP_Int16 _udp_port;         // point cloud UDP port (default 60001)
    AP_Int16 _hmax_cm;          // keep points from the radar plane up to this height (default 30 cm)
};

#endif // AP_PROXIMITY_MID360_ENABLED
