#pragma once

#include <eigen3/Eigen/Core>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <serial_util/crc/dji_crc.hpp>
#include <serial_util/package_receive.hpp>

#include "package/graphic.hpp"

namespace rmcs_core::hardware::referee {

class UI
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    UI()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {

        register_input("/gimbal/auto_aim/origin_coordinate", coordinate_auto_aim_, false);

        try {
            register_output(
                "/referee/serial", serial_, get_parameter("path").as_string(), 115200,
                serial::Timeout::simpleTimeout(0));
        } catch (serial::IOException& ex) {
            RCLCPP_ERROR(logger_, "Unable to open serial port: %s", ex.what());
        }
    };

    void update() override {}

private:
    void init_ui() {}

    void draw_string(const ui::Description& description, const std::string& string) {
        auto package                  = ui::StringPackage();
        package.header.length         = 45;
        package.command               = 0x0301;
        package.data.command          = 0x0110;
        package.data.sender           = sender_;
        package.data.receiver         = receiver_;
        package.data.data.description = description;
        string.copy(reinterpret_cast<char*>(package.data.data.data), string.length());

        this->write(reinterpret_cast<uint8_t*>(&package), sizeof(package));
    }

    template <typename T, typename... Args>
    requires(std::is_same_v<T, uint8_t>) void delete_layer(T index, Args... args) {
        assert(index > -1 && index < 10);
        if (sizeof...(Args) != 0) {
            delete_layer(args...);
        }

        auto package                  = ui::DeletePackage();
        package.header.length         = 2;
        package.command               = 0x0301;
        package.data.command          = 0x0100;
        package.data.sender           = sender_;
        package.data.receiver         = receiver_;
        package.data.data.delete_type = 1;
        package.data.data.layer       = index;

        this->write(reinterpret_cast<uint8_t*>(&package), sizeof(package));
    }
    void delete_layer() {
        auto package                  = ui::DeletePackage();
        package.header.length         = 2;
        package.command               = 0x0301;
        package.data.command          = 0x0100;
        package.data.sender           = sender_;
        package.data.receiver         = receiver_;
        package.data.data.delete_type = 2;

        this->write(reinterpret_cast<uint8_t*>(&package), sizeof(package));
    }

    void apply_description(const ui::Description& description) {
        auto package          = ui::DrawPackage1();
        package.header.length = 15;
        package.command       = 0x0301;
        package.data.command  = 0x0101;
        package.data.sender   = sender_;
        package.data.receiver = receiver_;
        package.data.data     = description;

        this->write(reinterpret_cast<uint8_t*>(&package), sizeof(package));
    }

    template <size_t n>
    requires(n == 2 || n == 5 || n == 7)
    void apply_description(const std::array<ui::Description, n>& description) {
        if constexpr (n == 2) {
            auto package          = ui::DrawPackage2();
            package.header.length = 30;
            package.command       = 0x0301;
            package.data.command  = 0x0102;
            package.data.sender   = sender_;
            package.data.receiver = receiver_;

            for (int i = 0; i < n; i++) {
                package.data.data.description[i] = description[i];
            }
            this->write(reinterpret_cast<uint8_t*>(&package), sizeof(package));

        } else if constexpr (n == 5) {
            auto package          = ui::DrawPackage5();
            package.header.length = 75;
            package.command       = 0x0301;
            package.data.command  = 0x0103;
            package.data.sender   = sender_;
            package.data.receiver = receiver_;

            for (int i = 0; i < n; i++) {
                package.data.data.description[i] = description[i];
            }

            this->write(reinterpret_cast<uint8_t*>(&package), sizeof(package));

        } else if constexpr (n == 7) {
            auto package          = ui::DrawPackage7();
            package.header.length = 105;
            package.command       = 0x0301;
            package.data.command  = 0x0104;
            package.data.sender   = sender_;
            package.data.receiver = receiver_;

            for (int i = 0; i < n; i++) {
                package.data.data.description[i] = description[i];
            }

            this->write(reinterpret_cast<uint8_t*>(&package), sizeof(package));
        }
    }

    size_t write(const uint8_t* data, size_t size) { return serial_->write(data, size); }

private:
    OutputInterface<serial::Serial> serial_;
    rclcpp::Logger logger_;

    InputInterface<Eigen::Vector2d> coordinate_auto_aim_;

    uint16_t sender_;
    uint16_t receiver_;
};
} // namespace rmcs_core::hardware::referee

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::referee::UI, rmcs_executor::Component)