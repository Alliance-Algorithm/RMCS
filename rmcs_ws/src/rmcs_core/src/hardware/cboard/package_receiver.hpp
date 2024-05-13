#pragma once

#include <cassert>
#include <cstdint>
#include <functional>
#include <unordered_map>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <serial/serial.h>

#include "hardware/cboard/package.hpp"

namespace rmcs_core::hardware::cboard {

class PackageReceiver final {
public:
    PackageReceiver()                                  = default;
    PackageReceiver(const PackageReceiver&)            = delete;
    PackageReceiver& operator=(const PackageReceiver&) = delete;

    void subscribe(
        package_type_t type_code, std::function<void(std::unique_ptr<Package>)> callback) {
        const auto [it, success] =
            subscribed_containers_.insert(std::make_pair(type_code, std::move(callback)));
        assert(success);
    }

    void update(serial::Serial& serial, rclcpp::Logger& logger) {
        while (true) {
            auto result = receive_static_part(serial);

            if (result == ReceiveResult::SUCCESS) {
                size_t package_real_size = receiving_package->size();
                if (package_real_size > sizeof(receiving_package->buffer)) {
                    RCLCPP_WARN(logger, "[package_receiver]: invalid static part");
                    received_size_ = 0;
                    continue;
                }

                received_size_ += serial.read(
                    receiving_package->buffer + received_size_, package_real_size - received_size_);
                if (received_size_ < package_real_size)
                    break;

                if (receiving_package->verify_part()
                    != calculate_verify_code(
                        receiving_package->buffer,
                        receiving_package->size() - receiving_package->verify_part_size())) {
                    RCLCPP_WARN(logger, "[package_receiver]: invalid verify digit");
                    received_size_ = 0;
                    continue;
                }

                auto iter = subscribed_containers_.find(receiving_package->static_part().type);
                if (iter == subscribed_containers_.end()) {
                    RCLCPP_INFO(
                        logger, "[package_receiver]: unsubscribed package: 0x%02X",
                        receiving_package->static_part().type);
                    received_size_ = 0;
                    continue;
                }

                iter->second(std::move(receiving_package));
                received_size_    = 0;
                receiving_package = std::make_unique<Package>();
            } else if (result == ReceiveResult::TIMEOUT) {
                break;
            } else if (result == ReceiveResult::INVALID_HEADER) {
                RCLCPP_WARN(logger, "[package_receiver]: invalid header");
            }
        }
    }

private:
    enum class ReceiveResult : uint8_t {
        SUCCESS              = 0,
        TIMEOUT              = 1,
        INVALID_HEADER       = 2,
        INVALID_VERIFY_DIGIT = 4
    };

    ReceiveResult receive_static_part(serial::Serial& serial) {
        ReceiveResult result;

        if (received_size_ >= sizeof(PackageStaticPart)) {
            // If the static data part has been cached, return success directly.
            return ReceiveResult::SUCCESS;
        }

        // Receive as much data as possible.
        received_size_ += serial.read(
            receiving_package->buffer + received_size_, sizeof(PackageStaticPart) - received_size_);

        if (received_size_ < sizeof(package_head_t)) {
            // If even the package header cannot be received, return timeout.
            return ReceiveResult::TIMEOUT;
        }

        // Successfully received the package header.
        if (reinterpret_cast<PackageStaticPart*>(receiving_package->buffer)->head == kPackageHead) {
            // If the package header is correct.
            if (received_size_ == sizeof(PackageStaticPart)) {
                // If the header is correct and the data is received completely, return success.
                return ReceiveResult::SUCCESS;
            } else {
                // If the header is correct but the data is not received completely, return timeout
                // and wait for the next reception.
                return ReceiveResult::TIMEOUT;
            }
        } else {
            result = ReceiveResult::INVALID_HEADER;
        }

        // If the package header is incorrect, search for a matching header. If found, align the
        // data as a whole for the next reception of the remaining part. Even if no matching header
        // is found, when traversing the remaining bytes is less than the number of header bytes,
        // the remaining bytes will still be aligned forward.
        --received_size_;
        auto head_pointer = receiving_package->buffer + 1;

        while (true) {
            if (received_size_ < sizeof(package_head_t)
                || *reinterpret_cast<package_head_t*>(head_pointer) == kPackageHead) {
                for (size_t i = 0; i < received_size_; ++i)
                    reinterpret_cast<uint8_t*>(
                        &reinterpret_cast<PackageStaticPart*>(receiving_package->buffer)->head)[i] =
                        head_pointer[i];
                break;
            }
            --received_size_;
            ++head_pointer;
        }

        return result;
    }

    std::unordered_map<package_type_t, std::function<void(std::unique_ptr<Package>)>>
        subscribed_containers_ = {};

    std::unique_ptr<Package> receiving_package = std::make_unique<Package>();
    size_t received_size_                      = 0;
};

} // namespace rmcs_core::hardware::cboard