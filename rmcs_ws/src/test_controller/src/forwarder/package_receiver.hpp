#pragma once

#include <cassert>
#include <cstdint>
#include <functional>
#include <map>

#include <rclcpp/logging.hpp>
#include <serial/serial.h>

#include "forwarder/package.hpp"

namespace forwarder {

class PackageReceiver final {
public:
    explicit PackageReceiver(serial::Serial& serial)
        : serial_(serial) {}
    PackageReceiver(const PackageReceiver&)            = delete;
    PackageReceiver& operator=(const PackageReceiver&) = delete;

    void subscribe(
        package_type_t type_code, std::function<void(std::unique_ptr<Package>)> callback) {
        const auto [it, success] =
            subscribed_containers_.insert(std::make_pair(type_code, std::move(callback)));
        assert(success);
    }

    void update() {
        while (true) {
            auto result = receive_static_part();

            if (result == ReceiveResult::SUCCESS) {
                size_t package_real_size = receiving_package->size();
                if (package_real_size > sizeof(receiving_package->buffer)) {
                    RCLCPP_WARN(rclcpp::get_logger("PackageReceiver"), "invaild static part");
                    received_size_ = 0;
                    continue;
                }

                received_size_ += serial_.read(
                    receiving_package->buffer + received_size_, package_real_size - received_size_);
                if (received_size_ < package_real_size)
                    break;

                if (receiving_package->verify_part()
                    != calculate_verify_code(
                        receiving_package->buffer,
                        receiving_package->size() - receiving_package->verify_part_size())) {
                    RCLCPP_WARN(rclcpp::get_logger("PackageReceiver"), "invaild verify degit");
                    received_size_ = 0;
                    continue;
                }

                auto iter = subscribed_containers_.find(receiving_package->static_part().type);
                if (iter == subscribed_containers_.end()) {
                    RCLCPP_INFO(
                        rclcpp::get_logger("PackageReceiver"), "unsubscribed package: 0x%02X",
                        receiving_package->static_part().type);
                    received_size_ = 0;
                    continue;
                }

                iter->second(std::move(receiving_package));
                received_size_    = 0;
                receiving_package = std::make_unique<Package>();
            } else if (result == ReceiveResult::TIMEOUT) {
                break;
            } else if (result == ReceiveResult::INVAILD_HEADER) {
                RCLCPP_WARN(rclcpp::get_logger("PackageReceiver"), "invaild header");
            }
        }
    }

public:
    enum class ReceiveResult : uint8_t {
        SUCCESS              = 0,
        TIMEOUT              = 1,
        INVAILD_HEADER       = 2,
        INVAILD_VERIFY_DEGIT = 4
    };

    ReceiveResult receive_static_part() {
        ReceiveResult result;

        if (received_size_ >= sizeof(PackageStaticPart)) {
            // 如果静态数据部分已被缓存，则直接返回Success
            return ReceiveResult::SUCCESS;
        }

        // 接收尽可能多的数据
        received_size_ += serial_.read(
            receiving_package->buffer + received_size_, sizeof(PackageStaticPart) - received_size_);

        if (received_size_ < sizeof(package_head_t)) {
            // 连包头都接收不到，则返回Timeout
            return ReceiveResult::TIMEOUT;
        }

        // 成功接收到包头
        if (reinterpret_cast<PackageStaticPart*>(receiving_package->buffer)->head == kPackageHead) {
            // 若数据包头正确
            if (received_size_ == sizeof(PackageStaticPart)) {
                // 若包头正确且数据接收完整，则返回Success
                return ReceiveResult::SUCCESS;
            } else {
                // 若包头正确但数据未接收完整，则返回Timeout，等待下一次接收
                return ReceiveResult::TIMEOUT;
            }
        } else {
            result = ReceiveResult::INVAILD_HEADER;
        }

        // 若数据包头错误，则向后寻找匹配的头，若找到，则把数据整体向前对齐，以便下一次接收剩余部分
        // 即使没有找到匹配的头，当遍历剩余字节数低于头的字节数时，也会把剩余字节向前对齐
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

    serial::Serial& serial_;

    std::map<package_type_t, std::function<void(std::unique_ptr<Package>)>> subscribed_containers_ =
        {};

    std::unique_ptr<Package> receiving_package = std::make_unique<Package>();
    size_t received_size_                      = 0;
};

} // namespace forwarder