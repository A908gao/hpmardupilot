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

/*
 * ArduPilot proximity backend for the Livox MID360 (Agras radar).
 *
 * The Agras radar (192.168.1.10) detects the host (flight controller at
 * 192.168.1.20) after power-up and streams raw Livox point cloud frames to
 * UDP 60001 by itself. The backend decodes the raw LivoxLidarEthernetPacket
 * frames (cartesian high / low / spherical), keeps only points between the
 * radar plane and PRXx_MID360_HMAX above it (higher points are filtered out)
 * and pushes the horizontal range of each surviving point into the 8-sector
 * 3D boundary used for object avoidance.
 */

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_MID360_ENABLED

#include "AP_Proximity_MID360.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

// Wire format of the Livox point stream, copied from Livox-SDK2
// (livox_lidar_def.h). No SDK is used at runtime.
#pragma pack(push, 1)
struct PACKED LivoxLidarEthernetPacket {
    uint8_t version;
    uint16_t length;        // total frame length in bytes
    uint16_t time_interval; // unit: 0.1 us
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
struct PACKED LivoxLidarCartesianHighRawPoint {
    int32_t x;  // mm
    int32_t y;  // mm
    int32_t z;  // mm
    uint8_t reflectivity;
    uint8_t tag;
};
struct PACKED LivoxLidarCartesianLowRawPoint {
    int16_t x;  // cm
    int16_t y;  // cm
    int16_t z;  // cm
    uint8_t reflectivity;
    uint8_t tag;
};
struct PACKED LivoxLidarSpherPoint {
    uint32_t depth; // mm
    uint16_t theta; // 0.01 deg
    uint16_t phi;   // 0.01 deg
    uint8_t reflectivity;
    uint8_t tag;
};
#pragma pack(pop)

#define MID360_DATA_TYPE_IMU         0x00
#define MID360_DATA_TYPE_CART_HIGH   0x01
#define MID360_DATA_TYPE_CART_LOW    0x02
#define MID360_DATA_TYPE_SPHERICAL   0x03

// processing limits: the MID360 point rate is far above what the avoidance
// slice needs, so cap the work per update call
#define MID360_MAX_DATAGRAMS_PER_UPDATE   5
#define MID360_MAX_POINTS_PER_DATAGRAM    4000
#define MID360_DATAGRAM_BUF_SIZE          2048
#define MID360_TIMEOUT_MS                 500

const AP_Param::GroupInfo AP_Proximity_MID360::var_info[] = {

    // @Param: PORT
    // @DisplayName: MID360 point cloud UDP port
    // @Description: UDP port the Agras radar streams raw point cloud frames to
    // @Range: 1 65535
    // @User: Advanced
    AP_GROUPINFO("PORT", 1, AP_Proximity_MID360, _udp_port, 60001),

    // @Param: HMAX
    // @DisplayName: MID360 max obstacle height above radar
    // @Description: Only points between the radar plane and this height above it are used for obstacle avoidance; higher points are filtered out
    // @Units: cm
    // @Range: 1 500
    // @User: Standard
    AP_GROUPINFO("HMAX", 2, AP_Proximity_MID360, _hmax_cm, 30),

    AP_GROUPEND
};

AP_Proximity_MID360::AP_Proximity_MID360(AP_Proximity &_frontend,
                                         AP_Proximity::Proximity_State &_state,
                                         AP_Proximity_Params &_params)
    : AP_Proximity_Backend(_frontend, _state, _params)
{
    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
    _last_recv_ms = 0;

    if (!init_socket()) {
        set_status(AP_Proximity::Status::NotConnected);
    }
}

bool AP_Proximity_MID360::init_socket()
{
    _socket.reuseaddress();
    if (!_socket.bind("0.0.0.0", uint16_t(_udp_port.get()))) {
        return false;
    }
    _socket.set_broadcast();
    return true;
}

// update the state of the sensor
void AP_Proximity_MID360::update()
{
    // drain pending datagrams into the sector boundary
    uint8_t buf[MID360_DATAGRAM_BUF_SIZE];
    for (uint8_t i = 0; i < MID360_MAX_DATAGRAMS_PER_UPDATE; i++) {
        const ssize_t ret = _socket.recv(buf, sizeof(buf), 0);
        if (ret <= 0) {
            break;
        }
        handle_datagram(buf, uint32_t(ret));
    }

    const uint32_t now = AP_HAL::millis();
    if (_last_recv_ms != 0 && (now - _last_recv_ms) <= MID360_TIMEOUT_MS) {
        set_status(AP_Proximity::Status::Good);
    } else {
        set_status(AP_Proximity::Status::NoData);
    }
}

// a datagram carries one or more LivoxLidarEthernetPacket frames
void AP_Proximity_MID360::handle_datagram(const uint8_t *buf, uint32_t len)
{
    const uint32_t hdr_size = offsetof(LivoxLidarEthernetPacket, data);
    uint32_t offset = 0;
    bool any = false;

    while (len - offset >= hdr_size) {
        const LivoxLidarEthernetPacket *pkt = reinterpret_cast<const LivoxLidarEthernetPacket *>(buf + offset);
        if (pkt->length < hdr_size || pkt->length > len - offset) {
            break;  // malformed frame, stop parsing this datagram
        }
        handle_packet(pkt);
        offset += pkt->length;
        any = true;
    }

    if (any) {
        _last_recv_ms = AP_HAL::millis();
    }
}

void AP_Proximity_MID360::handle_packet(const LivoxLidarEthernetPacket *pkt)
{
    if (pkt->data_type == MID360_DATA_TYPE_IMU) {
        return; // IMU frames are not needed for avoidance
    }

    const uint32_t avail = pkt->length - offsetof(LivoxLidarEthernetPacket, data);

    switch (pkt->data_type) {
    case MID360_DATA_TYPE_CART_HIGH: {
        uint32_t count = MIN(pkt->dot_num, avail / uint32_t(sizeof(LivoxLidarCartesianHighRawPoint)));
        count = MIN(count, uint32_t(MID360_MAX_POINTS_PER_DATAGRAM));
        for (uint32_t i = 0; i < count; i++) {
            LivoxLidarCartesianHighRawPoint raw;
            memcpy(&raw, pkt->data + i * sizeof(raw), sizeof(raw));
            if (raw.tag == 0x00) {
                continue;   // no laser return
            }
            if (raw.x == 0 && raw.y == 0 && raw.z == 0) {
                continue;   // zero-fill placeholder
            }
            handle_point(raw.x * 0.001f, raw.y * 0.001f, raw.z * 0.001f);
        }
        break;
    }
    case MID360_DATA_TYPE_CART_LOW: {
        uint32_t count = MIN(pkt->dot_num, avail / uint32_t(sizeof(LivoxLidarCartesianLowRawPoint)));
        count = MIN(count, uint32_t(MID360_MAX_POINTS_PER_DATAGRAM));
        for (uint32_t i = 0; i < count; i++) {
            LivoxLidarCartesianLowRawPoint raw;
            memcpy(&raw, pkt->data + i * sizeof(raw), sizeof(raw));
            if (raw.tag == 0x00) {
                continue;
            }
            if (raw.x == 0 && raw.y == 0 && raw.z == 0) {
                continue;
            }
            handle_point(raw.x * 0.01f, raw.y * 0.01f, raw.z * 0.01f);
        }
        break;
    }
    case MID360_DATA_TYPE_SPHERICAL: {
        uint32_t count = MIN(pkt->dot_num, avail / uint32_t(sizeof(LivoxLidarSpherPoint)));
        count = MIN(count, uint32_t(MID360_MAX_POINTS_PER_DATAGRAM));
        for (uint32_t i = 0; i < count; i++) {
            LivoxLidarSpherPoint raw;
            memcpy(&raw, pkt->data + i * sizeof(raw), sizeof(raw));
            if (raw.tag == 0x00) {
                continue;
            }
            if (raw.depth < 100 || raw.depth > 50000) {
                continue;   // 0.1 m .. 50 m (mm)
            }
            const float r = raw.depth * 0.001f;
            const float theta = radians(raw.theta * 0.01f);
            const float phi = radians(raw.phi * 0.01f);
            handle_point(r * sinf(theta) * cosf(phi),
                         r * sinf(theta) * sinf(phi),
                         r * cosf(theta));
        }
        break;
    }
    default:
        break;
    }
}

// keep the low-altitude slice (radar plane up to HMAX above it) and feed the
// horizontal range into the 8-sector boundary
void AP_Proximity_MID360::handle_point(float x_m, float y_m, float z_m)
{
    const float max_z = _hmax_cm.get() * 0.01f;
    if (z_m < 0.0f || z_m > max_z) {
        return; // outside the avoidance slice, filter out
    }

    const float horiz = sqrtf(x_m * x_m + y_m * y_m);
    if (horiz < 0.05f || horiz > 40.0f) {
        return;
    }

    const float yaw_deg = wrap_360(degrees(atan2f(y_m, x_m)));
    const float pitch_deg = degrees(atan2f(z_m, horiz));
    const float yaw_corrected = correct_angle_for_orientation(yaw_deg);

    const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(pitch_deg, yaw_corrected);
    frontend.boundary.set_face_attributes(face, pitch_deg, yaw_corrected, horiz, state.instance);
}

#endif // AP_PROXIMITY_MID360_ENABLED
