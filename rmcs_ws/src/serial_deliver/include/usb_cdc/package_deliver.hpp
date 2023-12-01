#pragma once

#include <cassert>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>

#include <rclcpp/logging.hpp>
#include <serial/serial.h>

#include "fps_counter.hpp"

namespace usb_cdc {

constexpr size_t package_size_max = 64;

using package_head_t  = uint8_t;
using package_type_t  = uint8_t;
using package_index_t = uint8_t;
using package_size_t  = uint8_t;

using package_verify_code_t = uint8_t;
inline package_verify_code_t calculate_verify_code(const uint8_t* data, size_t size) {
    return std::accumulate(data, data + size, static_cast<package_verify_code_t>(0));
}

constexpr package_head_t package_head = 0xAF;

struct __attribute__((packed)) PackageStaticPart final {
    package_head_t head;
    package_type_t type;
    package_index_t index;
    package_size_t data_size;
};

struct alignas(8) Package final {
    static constexpr size_t static_part_size() noexcept { return sizeof(PackageStaticPart); }

    PackageStaticPart& static_part() noexcept {
        return *reinterpret_cast<PackageStaticPart*>(buffer);
    }

    size_t dymatic_part_size() noexcept { return static_part().data_size; }

    template <typename T>
    T& dymatic_part() noexcept {
        return *reinterpret_cast<T*>(buffer + static_part_size());
    }

    static constexpr size_t verify_part_size() noexcept { return sizeof(package_verify_code_t); }

    package_verify_code_t& verify_part() noexcept {
        size_t length_without_verify = static_part_size() + dymatic_part_size();
        return *reinterpret_cast<package_verify_code_t*>(buffer + length_without_verify);
    }

    size_t size() noexcept { return static_part_size() + dymatic_part_size() + verify_part_size(); }

    uint8_t buffer[package_size_max];
};

class PackageContainer final {
public:
    explicit PackageContainer(size_t max_keep_count)
        : max_keep_count_(max_keep_count) {
        assert(max_keep_count > 0);
    }
    PackageContainer(const PackageContainer&)            = delete;
    PackageContainer& operator=(const PackageContainer&) = delete;
    PackageContainer(PackageContainer&&)                 = default;
    PackageContainer& operator=(PackageContainer&&)      = default;

    ~PackageContainer() {
        while (count_) {
            pop();
        }
    }

    void push(std::unique_ptr<Package> package) {
        assert(package != nullptr);
        Package* pointer = package.release();

        if (count_ == 0) {
            first_package_ = pointer;
            last_package_  = pointer;
        } else {
            last_package_->next = pointer;
            last_package_       = pointer;
        }

        if (count_ == max_keep_count_) {
            pop();
        } else
            ++count_;
    }

    std::unique_ptr<Package> pop() {
        if (count_ == 0)
            return nullptr;

        std::unique_ptr<Package> package(first_package_);
        first_package_ = first_package_->next;
        --count_;
        return package;
    }

private:
    size_t count_ = 0;
    size_t max_keep_count_;

    Package* first_package_ = nullptr;
    Package* last_package_  = nullptr;
};

class PackageDeliver final {
public:
    PackageDeliver(const std::string& port)
        : serial_(port, 9600, serial::Timeout::simpleTimeout(0)) {}
    PackageDeliver(const PackageDeliver&)            = delete;
    PackageDeliver& operator=(const PackageDeliver&) = delete;

    void subscribe(package_type_t type_code, size_t max_keep_count = 1) {
        const auto [it, success] = subscribed_containers_.insert(
            std::make_pair(type_code, PackageContainer{max_keep_count}));
        assert(success);
    }

    void update() {
        while (true) {
            auto result = receive_static_part();

            if (result == ReceiveResult::SUCCESS) {
                size_t package_real_size = receiving_package->size();
                if (package_real_size > sizeof(receiving_package->buffer)) {
                    RCLCPP_WARN(rclcpp::get_logger("PackageDeliver"), "invaild static part");
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
                    RCLCPP_WARN(rclcpp::get_logger("PackageDeliver"), "invaild verify degit");
                    received_size_ = 0;
                    continue;
                }

                auto iter = subscribed_containers_.find(receiving_package->static_part().type);
                if (iter == subscribed_containers_.end()) {
                    RCLCPP_INFO(
                        rclcpp::get_logger("PackageDeliver"), "unsubscribed package: %02X",
                        receiving_package->static_part().type);
                    received_size_ = 0;
                    continue;
                }

                iter->second.push(std::move(receiving_package));
                receiving_package = std::make_unique<Package>();
            } else if (result == ReceiveResult::TIMEOUT) {
                break;
            } else if (result == ReceiveResult::INVAILD_HEADER) {
                RCLCPP_WARN(rclcpp::get_logger("PackageDeliver"), "invaild header");
            }
        }
    }

    std::unique_ptr<Package> get(package_type_t type_code) {
        auto iter = subscribed_containers_.find(type_code);
        assert(iter != subscribed_containers_.end());

        return iter->second.pop();
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
        if (reinterpret_cast<PackageStaticPart*>(receiving_package->buffer)->head == package_head) {
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
                || *reinterpret_cast<package_head_t*>(head_pointer) == package_head) {
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

    serial::Serial serial_;

    std::map<package_type_t, PackageContainer> subscribed_containers_ = {};

    std::unique_ptr<Package> receiving_package = std::make_unique<Package>();
    size_t received_size_                      = 0;
};

} // namespace usb_cdc