// #include <cmath>
// #include <cstdint>
// #include <iostream>

// #include <serial/serial.h>
// #include <sys/types.h>

// #include "endian_promise.hpp"
// #include "usb_cdc/package_deliver.hpp"

int main() {
    // usb_cdc::PackageDeliver deliver("/dev/ttyACM0");
    // deliver.subscribe(0x11, 1);
    // while (true) {
    //     deliver.update();
    //     auto package = deliver.get(0x11);

    //     if (package) {
    //         struct __attribute__((packed)) ReceivePackage {
    //             usb_cdc::PackageStaticPart static_part;
    //             endian::le_int32_t can_origin;
    //             endian::be_int16_t angle;
    //             endian::be_int16_t speed;
    //             endian::be_int16_t current;
    //             uint8_t temperature;
    //             uint8_t _;
    //             usb_cdc::package_verify_code_t crc;
    //         }& receive_package = *reinterpret_cast<ReceivePackage*>(package->buffer);

    //         double angle    = receive_package.angle;
    //         double velocity = receive_package.speed;

    //         // std::for_each(
    //         //     reinterpret_cast<uint8_t*>(&receive_package),
    //         //     reinterpret_cast<uint8_t*>(&receive_package) + sizeof(receive_package),
    //         //     [](uint8_t byte) { printf("%02X ", byte); });
    //         // putchar('\n');

    //         auto simple_pd = [](double velocity) -> double {
    //             static double setpoint = -2000.0;
    //             static double last_err = 0;

    //             double err    = setpoint - velocity;
    //             double result = 30.0 * err + 60.0 * (err - last_err);
    //             last_err      = err;

    //             return result;
    //         };

    //         struct __attribute__((packed)) SendPackage {
    //             uint8_t head      = 0xAF;
    //             uint8_t type_code = 0x11;
    //             uint8_t index     = 0;
    //             uint8_t size      = 12;
    //             endian::le_int32_t can_origin;
    //             endian::be_int16_t current1;
    //             endian::be_int16_t current2;
    //             endian::be_int16_t current3;
    //             endian::be_int16_t current4;
    //             usb_cdc::package_verify_code_t crc;
    //         } send_package;
    //         send_package.can_origin = 0x200;

    //         double current = simple_pd(velocity);
    //         if (current < -16384)
    //             current = -16384;
    //         if (current > 0)
    //             current = 0;
    //         if (current > 16384)
    //             current = 16384;
    //         current = std::round(current);
    //         current = -16384;

    //         send_package.current1 = static_cast<int16_t>(current);
    //         send_package.current2 = 0;
    //         send_package.current3 = 0;
    //         send_package.current4 = 0;
    //         send_package.crc      = verify::CheckSum::Calculate(
    //             reinterpret_cast<uint8_t*>(&send_package), sizeof(send_package) - 1);

    //         // std::for_each(
    //         //     reinterpret_cast<uint8_t*>(&send_package),
    //         //     reinterpret_cast<uint8_t*>(&send_package) + sizeof(send_package),
    //         //     [](uint8_t byte) { printf("%02X ", byte); });
    //         // putchar('\n');

    //         deliver.serial_.write(reinterpret_cast<uint8_t*>(&send_package), sizeof(send_package));
    //         std::cout << angle << ' ' << velocity << ' ' << send_package.current1 << std::endl;
    //     }
    // }
}