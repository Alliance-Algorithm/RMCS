#pragma once

#include <numeric>

#include "endian_promise.hpp"
#include "serial_deliver.hpp"

namespace hw_states_packages {

class UartDbusRxData {
public:
    double ch1, ch2, ch3, ch4;
    double sw1, sw2;

    UartDbusRxData()          = default;
    virtual ~UartDbusRxData() = default;

    void set(const uint8_t* buf) {
        // satori：这里完成的是数据的分离和拼接，减去1024是为了让数据的中间值变为0
        ch1 = (static_cast<int16_t>(buf[0]) | static_cast<int16_t>(buf[1]) << 8) & 0x07FF;
        ch1 -= 1024;
        ch2 = (static_cast<int16_t>(buf[1]) >> 3 | static_cast<int16_t>(buf[2]) << 5) & 0x07FF;
        ch2 -= 1024;
        ch3 = (static_cast<int16_t>(buf[2]) >> 6 | static_cast<int16_t>(buf[3]) << 2
               | static_cast<int16_t>(buf[4] << 10))
            & 0x07FF;
        ch3 -= 1024;
        ch4 = (static_cast<int16_t>(buf[4]) >> 1 | static_cast<int16_t>(buf[5]) << 7) & 0x07FF;
        ch4 -= 1024;
        // satori:防止数据零漂，设置正负5的死区
        /* prevent remote control zero deviation */
        if (ch1 <= 5 && ch1 >= -5)
            ch1 = 0;
        if (ch2 <= 5 && ch2 >= -5)
            ch2 = 0;
        if (ch3 <= 5 && ch3 >= -5)
            ch3 = 0;
        if (ch4 <= 5 && ch4 >= -5)
            ch4 = 0;
        sw1 = ((buf[5] >> 4) & 0x000C) >> 2;
        sw2 = (buf[5] >> 4) & 0x0003;
        /*/ satori:防止数据溢出
        if ((abs(ch1) > 660) || (abs(ch2) > 660) || (abs(ch3) > 660)
            || (abs(ch4) > 660)) {
            memset(rc, 0, sizeof(struct rc_info));
            return;
        }
        mouse.x     = buf[6] | (buf[7] << 8); // x axis
        mouse.y     = buf[8] | (buf[9] << 8);
        mouse.z     = buf[10] | (buf[11] << 8);
        mouse.l     = buf[12];
        mouse.r     = buf[13];
        kb.key_code = buf[14] | buf[15] << 8; // key borad code
        wheel       = (buf[16] | buf[17] << 8) - 1024;
        //*/// Unused data
    }
};

constexpr auto dbus_type_code =
    serial::SerialPackage::TypeEncode(serial::SerialPackage::PackageType::USB_PKG_UART, 0x03);

} // namespace hw_states_packages