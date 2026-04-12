#pragma once

#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <libusb.h>
#include <sys/types.h>

#include "core/src/utility/assert.hpp"
#include "host/src/logging/logging.hpp"
#include "host/src/transport/transport.hpp"
#include "host/src/transport/usb/helper.hpp"
#include "host/src/utility/final_action.hpp"

namespace librmcs::host::transport::usb {

class DeviceScanner {
public:
    static libusb_device_handle* select_device(
        libusb_context* context, uint16_t vendor_id, int32_t product_id,
        std::string_view serial_filter, const ConnectionOptions& options) {
        libusb_device** device_list = nullptr;
        const ssize_t device_count = libusb_get_device_list(context, &device_list);
        if (device_count < 0) {
            const int error_code = static_cast<int>(device_count);
            throw std::runtime_error(
                std::format(
                    "Failed to get USB device list: {} ({})", error_code,
                    helper::libusb_errname(error_code)));
        }

        const utility::FinalAction free_device_list{
            [&device_list]() noexcept { libusb_free_device_list(device_list, 1); }};

        auto infos =
            scan_devices(device_list, device_count, vendor_id, product_id, serial_filter, options);

        std::vector<libusb_device_handle*> devices_opened;
        for (const auto& info : infos) {
            if (const auto* matched = std::get_if<DeviceInfo::Matched>(&info.result))
                devices_opened.push_back(matched->handle);
        }

        if (devices_opened.size() != 1) {
            for (auto& device : devices_opened)
                libusb_close(device);

            const bool multiple_compatible_devices = devices_opened.size() > 1;
            const auto report = generate_device_discovery_report(
                infos, vendor_id, product_id, serial_filter, options, multiple_compatible_devices);
            throw std::runtime_error(
                std::format(
                    "{}\n\n{}",
                    multiple_compatible_devices ? "Multiple compatible devices found."
                                                : "No compatible device found.",
                    report));
        }

        return devices_opened[0];
    }

private:
    struct DeviceInfo {
        libusb_device* device = nullptr;
        libusb_device_descriptor descriptor = {};
        std::string serial_number;

        struct Matching {};

        struct DescriptorReadFailed {
            int libusb_error_number = 0;
        };
        struct DescriptorEmpty {};

        struct VendorMismatch {};
        struct ProductMismatch {};
        struct OpenFailed {
            int libusb_error_number = 0;
        };

        struct MissingSerialNumber {};
        struct SerialReadFailed {
            int libusb_error_number = 0;
        };
        struct SerialFilterMismatch {};

        struct MissingProductString {};
        struct ProductStringReadFailed {
            int libusb_error_number;
        };
        struct ProtocolVersionMismatch {
            std::string product_string;
        };

        struct Matched {
            libusb_device_handle* handle = nullptr;
        };

        using MatchResult = std::variant<
            Matching, DescriptorReadFailed, DescriptorEmpty, VendorMismatch, MissingSerialNumber,
            ProductMismatch, OpenFailed, SerialReadFailed, SerialFilterMismatch,
            MissingProductString, ProductStringReadFailed, ProtocolVersionMismatch, Matched>;
        MatchResult result;
    };

    struct DiscoveryTroubleshooting {
        bool access_denied = false;
        bool device_busy = false;
        bool pid_mismatch = false;
        bool serial_mismatch = false;
        bool firmware_mismatch = false;
    };

    static std::vector<DeviceInfo> scan_devices(
        libusb_device** device_list, ssize_t device_count, uint16_t vendor_id, int32_t product_id,
        std::string_view serial_filter, const ConnectionOptions& options) {
        std::vector<DeviceInfo> infos;
        infos.reserve(static_cast<size_t>(device_count));

        for (ssize_t i = 0; i < device_count; i++) {
            auto& info = infos.emplace_back();
            info.device = device_list[i];

            if (const int ret = libusb_get_device_descriptor(device_list[i], &info.descriptor);
                ret != 0) {
                info.result = DeviceInfo::DescriptorReadFailed{ret};
                continue;
            }
            if (info.descriptor.bLength == 0) {
                info.result = DeviceInfo::DescriptorEmpty{};
                continue;
            }

            if (info.descriptor.idVendor != vendor_id) {
                info.result = DeviceInfo::VendorMismatch{};
                continue;
            }
            if (product_id >= 0 && !std::cmp_equal(info.descriptor.idProduct, product_id)) {
                info.result = DeviceInfo::ProductMismatch{};
                continue;
            }

            libusb_device_handle* handle = nullptr;
            if (const int ret = libusb_open(device_list[i], &handle); ret != 0) {
                info.result = DeviceInfo::OpenFailed{ret};
                continue;
            }
            utility::FinalAction close_device{[&handle]() noexcept { libusb_close(handle); }};

            if (!read_and_match_serial_number(handle, serial_filter, info))
                continue;

            if (!match_product_version(handle, info, options.dangerously_skip_version_checks))
                continue;

            info.result = DeviceInfo::Matched{handle};
            close_device.disable();
        }

        return infos;
    }

    static bool read_and_match_serial_number(
        libusb_device_handle* handle, const std::string_view& serial_filter, DeviceInfo& info) {
        if (info.descriptor.iSerialNumber == 0) {
            info.result = DeviceInfo::MissingSerialNumber{};
            return false;
        }

        char serial_buf[256];
        const int n = libusb_get_string_descriptor_ascii(
            handle, info.descriptor.iSerialNumber, reinterpret_cast<unsigned char*>(serial_buf),
            sizeof(serial_buf));
        if (n < 0) {
            info.result = DeviceInfo::SerialReadFailed{n};
            return false;
        }

        info.serial_number = {serial_buf, static_cast<size_t>(n)};

        if (!serial_filter.empty()
            && !match_filter(serial_filter, std::string_view{info.serial_number})) {
            info.result = DeviceInfo::SerialFilterMismatch{};
            return false;
        }
        return true;
    }

    static constexpr bool match_filter(std::string_view filter, std::string_view target) {
        const auto *filter_it = filter.cbegin(), *filter_end = filter.cend();
        const auto *target_it = target.cbegin(), *target_end = target.cend();

        const auto to_upper = [](char c) constexpr {
            if (c >= 'a' && c <= 'z')
                return static_cast<char>(c - 'a' + 'A');
            return c;
        };

        while (filter_it != filter_end && target_it != target_end) {
            if (*filter_it == '-') {
                ++filter_it;
                continue;
            }
            if (*target_it == '-') {
                ++target_it;
                continue;
            }

            if (to_upper(*filter_it) != to_upper(*target_it))
                return false;

            ++filter_it;
            ++target_it;
        }

        while (filter_it != filter_end && *filter_it == '-') {
            ++filter_it;
        }

        return filter_it == filter_end;
    }

    static bool match_product_version(
        libusb_device_handle* handle, DeviceInfo& info, bool warn_only_when_mismatch) {
        if (info.descriptor.iProduct == 0) {
            info.result = DeviceInfo::MissingProductString{};
            return false;
        }

        char product_buf[256];
        const int n = libusb_get_string_descriptor_ascii(
            handle, info.descriptor.iProduct, reinterpret_cast<unsigned char*>(product_buf),
            sizeof(product_buf));
        if (n < 0) {
            info.result = DeviceInfo::ProductStringReadFailed{n};
            return false;
        }

        const std::string_view product_string{product_buf, static_cast<size_t>(n)};
        if (product_string != "RMCS Agent v" LIBRMCS_PROJECT_VERSION_STRING) {
            if (warn_only_when_mismatch) {
                logging::get_logger().warn(
                    "USB device firmware version mismatch: found '{}', expected "
                    "'RMCS Agent v{}'. Continuing because "
                    "dangerously_skip_version_checks is enabled.",
                    product_string, LIBRMCS_PROJECT_VERSION_STRING);
                return true;
            }

            info.result = DeviceInfo::ProtocolVersionMismatch{std::string{product_string}};
            return false;
        }

        return true;
    }

    static std::string generate_device_discovery_report(
        std::span<const DeviceInfo> infos, uint16_t vendor_id, int32_t product_id,
        std::string_view serial_filter, const ConnectionOptions& options,
        bool multiple_compatible_devices) {
        const std::string pid_text = product_id >= 0 ? std::format("0x{:04x}", product_id) : "Any";
        const std::string serial_filter_text =
            serial_filter.empty() ? "Any" : std::string{serial_filter};
        const std::string firmware_filter_text =
            options.dangerously_skip_version_checks
                ? "Any"
                : std::string{"v" LIBRMCS_PROJECT_VERSION_STRING};

        std::string report = std::format(
            "Target Specs:\n"
            "- VID: 0x{:04x}\n"
            "- PID: {}\n"
            "- Serial Filter: {}\n"
            "- Firmware: {}\n\n"
            "Discovered Devices:\n",
            vendor_id, pid_text, serial_filter_text, firmware_filter_text);

        DiscoveryTroubleshooting troubleshooting = {};
        std::size_t reported_device_count = 0;
        std::size_t matched_count = 0;

        for (const auto& info : infos) {
            if (std::holds_alternative<DeviceInfo::VendorMismatch>(info.result))
                continue;

            report.append(std::format("- {}\n", format_device_summary(info)));
            report.append(
                std::format(
                    "    -> {}\n",
                    describe_device_match_result(info, matched_count, troubleshooting)));
            reported_device_count++;
        }

        if (reported_device_count == 0)
            report.append("- No relevant devices discovered.\n");

        append_troubleshooting_section(report, troubleshooting, multiple_compatible_devices);
        return report;
    }

    static std::string format_device_summary(const DeviceInfo& info) {
        core::utility::assert_debug(info.device);
        std::string summary = std::format(
            "Bus {:03} Dev {:03}", static_cast<unsigned int>(libusb_get_bus_number(info.device)),
            static_cast<unsigned int>(libusb_get_device_address(info.device)));

        const bool has_descriptor =
            !std::holds_alternative<DeviceInfo::DescriptorReadFailed>(info.result)
            && !std::holds_alternative<DeviceInfo::DescriptorEmpty>(info.result);
        if (!has_descriptor)
            return summary;

        summary.append(
            std::format(" (ID {:04x}:{:04x}", info.descriptor.idVendor, info.descriptor.idProduct));
        if (!info.serial_number.empty())
            summary.append(std::format(", SN {}", info.serial_number));
        summary.push_back(')');

        return summary;
    }

    static std::string describe_device_match_result(
        const DeviceInfo& info, std::size_t& matched_count,
        DiscoveryTroubleshooting& troubleshooting) {
        if (std::holds_alternative<DeviceInfo::Matching>(info.result)) {
            core::utility::assert_failed_debug();
            return "Ignored: Internal matching state error";
        }
        if (const auto* descriptor_read_failed =
                std::get_if<DeviceInfo::DescriptorReadFailed>(&info.result)) {
            return std::format(
                "Ignored: USB descriptor unreadable ({} / {})",
                descriptor_read_failed->libusb_error_number,
                helper::libusb_errname(descriptor_read_failed->libusb_error_number));
        }
        if (std::holds_alternative<DeviceInfo::DescriptorEmpty>(info.result))
            return "Ignored: USB descriptor empty";
        if (std::holds_alternative<DeviceInfo::VendorMismatch>(info.result))
            return "Ignored: VID mismatch";
        if (std::holds_alternative<DeviceInfo::ProductMismatch>(info.result)) {
            troubleshooting.pid_mismatch = true;
            return "Ignored: PID mismatch";
        }
        if (const auto* open_failed = std::get_if<DeviceInfo::OpenFailed>(&info.result)) {
            if (open_failed->libusb_error_number == LIBUSB_ERROR_ACCESS) {
                troubleshooting.access_denied = true;
                return std::format(
                    "Ignored: Access denied ({} / {})", open_failed->libusb_error_number,
                    helper::libusb_errname(open_failed->libusb_error_number));
            }
            if (open_failed->libusb_error_number == LIBUSB_ERROR_BUSY) {
                troubleshooting.device_busy = true;
                return std::format(
                    "Ignored: Device busy ({} / {})", open_failed->libusb_error_number,
                    helper::libusb_errname(open_failed->libusb_error_number));
            }

            return std::format(
                "Ignored: Open failed ({} / {})", open_failed->libusb_error_number,
                helper::libusb_errname(open_failed->libusb_error_number));
        }
        if (std::holds_alternative<DeviceInfo::MissingSerialNumber>(info.result))
            return "Ignored: Missing serial number";
        if (const auto* serial_read_failed =
                std::get_if<DeviceInfo::SerialReadFailed>(&info.result)) {
            return std::format(
                "Ignored: Serial number read failed ({} / {})",
                serial_read_failed->libusb_error_number,
                helper::libusb_errname(serial_read_failed->libusb_error_number));
        }
        if (std::holds_alternative<DeviceInfo::SerialFilterMismatch>(info.result)) {
            troubleshooting.serial_mismatch = true;
            return "Ignored: Serial mismatch";
        }
        if (std::holds_alternative<DeviceInfo::MissingProductString>(info.result))
            return "Ignored: Missing product string";
        if (const auto* product_read_failed =
                std::get_if<DeviceInfo::ProductStringReadFailed>(&info.result)) {
            return std::format(
                "Ignored: Product string read failed ({} / {})",
                product_read_failed->libusb_error_number,
                helper::libusb_errname(product_read_failed->libusb_error_number));
        }
        if (const auto* protocol_mismatch =
                std::get_if<DeviceInfo::ProtocolVersionMismatch>(&info.result)) {
            troubleshooting.firmware_mismatch = true;
            return std::format(
                "Ignored: Firmware mismatch (Found: {})", protocol_mismatch->product_string);
        }
        if (std::holds_alternative<DeviceInfo::Matched>(info.result)) {
            matched_count++;
            return std::format("Matched #{}", matched_count);
        }
        core::utility::assert_failed_debug();
    }

    static void append_troubleshooting_section(
        std::string& report, const DiscoveryTroubleshooting& troubleshooting,
        bool multiple_compatible_devices) {
        report.append("\nTroubleshooting:\n");

        bool has_any_hints = false;
        const auto append_hint = [&report, &has_any_hints](std::string_view hint) {
            has_any_hints = true;
            report.append(std::format("- {}\n", hint));
        };

        if (troubleshooting.access_denied)
            append_hint("Access denied: Check USB permissions (e.g., udev rules).");
        if (troubleshooting.device_busy)
            append_hint("Device busy: Close other processes using the device.");
        if (troubleshooting.pid_mismatch)
            append_hint("PID mismatch: Check board type selection.");
        if (troubleshooting.serial_mismatch)
            append_hint("Serial mismatch: Verify target SN or loosen the serial filter.");
        if (troubleshooting.firmware_mismatch) {
            append_hint(
                std::format(
                    "Firmware mismatch: Flash device firmware to v{}.",
                    LIBRMCS_PROJECT_VERSION_STRING));
        }
        if (multiple_compatible_devices) {
            append_hint(
                "Multiple matches: Provide a stricter serial filter to target a specific device.");
        }

        if (!has_any_hints)
            report.append("- No additional hints.\n");
    }
};

} // namespace librmcs::host::transport::usb
