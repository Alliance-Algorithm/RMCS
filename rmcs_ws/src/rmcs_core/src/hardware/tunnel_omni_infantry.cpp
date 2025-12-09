#include <rclcpp/node.hpp>

#include <librmcs/client/cboard.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/framerate.hpp>

#include "description/tf/tunnel_omni_infantry.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/supercap.hpp"

#include <ranges>

namespace rmcs_core::hardware {

class SlaveDevice final
    : public librmcs::client::CBoard
    , public rmcs_executor::Component {
public:
    explicit SlaveDevice(rmcs_executor::Component& master, rclcpp::Node& node, int usb_pid = -1)
        : CBoard{usb_pid}
        , logger(node.get_logger())
        , master{master} {

        using namespace device;
        {
            auto config = DjiMotor::Config{DjiMotor::Type::M3508};
            config.set_reversed();
            config.set_reduction_ratio(13.);
            config.enable_multi_turn_angle();
            std::ranges::for_each(
                chassis_wheel_motors, [&](DjiMotor& motor) { motor.configure(config); });
        }
        {
            auto config = LkMotor::Config{LkMotor::Type::MG5010E_I10};
            config.set_reversed();
            config.set_encoder_zero_point(node.get_parameter_or<int>("yaw_motor_zero_point", 0));
            gimbal_motor_yaw.configure(config);
        }
        {
            auto config = LkMotor::Config{LkMotor::Type::MG4010E_I10};
            config.set_reversed();
            config.set_encoder_zero_point(node.get_parameter_or<int>("pitch_motor_zero_point", 0));
            gimbal_motor_pitch.configure(config);
        }
        {
            auto config = DjiMotor::Config{DjiMotor::Type::M3508};
            config.set_reduction_ratio(1.);
            gimbal_friction_left.configure(config);

            config.set_reversed();
            gimbal_friction_right.configure(config);
        }
        {
            auto config = DjiMotor::Config{DjiMotor::Type::M2006};
            config.enable_multi_turn_angle();
            gimbal_bullet_feeder.configure(config);
        }
        {
            register_output("/gimbal/yaw/velocity_imu", velocity_gimbal_yaw);
            register_output("/gimbal/pitch/velocity_imu", velocity_gimbal_pitch);

            register_output("/referee/serial", referee_serial);
            referee_serial->read = [this](std::byte* buffer, std::size_t length) {
                return referee_buffer.pop_front_multi(
                    [&](std::byte byte) { *buffer++ = byte; }, length);
            };
            referee_serial->write = [this](const std::byte* buffer, std::size_t length) {
                transmit_buffer.add_uart1_transmission(buffer, length);
                return length;
            };

            TunnelOmniInfantryTf::set_state<"imu_link", "pitch_link">(
                Eigen::AngleAxisd{+0.5 * std::numbers::pi, Eigen::Vector3d::UnitZ()});

            constexpr auto gimbal_center_height = 0.32059;
            TunnelOmniInfantryTf::set_state<"gimbal_center_link", "base_link">(
                Eigen::Translation3d{0, 0, -gimbal_center_height});

            constexpr auto wheel_spacing = 0.15897;
            TunnelOmniInfantryTf::set_state<"base_link", "left_front_wheel_link">(
                Eigen::Translation3d{+1 * wheel_spacing, +1 * wheel_spacing, 0});
            TunnelOmniInfantryTf::set_state<"base_link", "right_front_wheel_link">(
                Eigen::Translation3d{-1 * wheel_spacing, +1 * wheel_spacing, 0});
            TunnelOmniInfantryTf::set_state<"base_link", "left_back_wheel_link">(
                Eigen::Translation3d{-1 * wheel_spacing, -1 * wheel_spacing, 0});
            TunnelOmniInfantryTf::set_state<"base_link", "right_back_wheel_link">(
                Eigen::Translation3d{+1 * wheel_spacing, -1 * wheel_spacing, 0});
        }
    }

    ~SlaveDevice() override {
        CBoard::stop_handling_events();
        if (background_thread.joinable()) {
            background_thread.join();
        }
    }

    auto update() -> void override { update_control(); }

    auto update_peripherals() {
        using namespace rmcs_description;
        {
            imu.update_status();
            const auto orientation_odom_to_imu =
                Eigen::Quaterniond{imu.q0(), imu.q1(), imu.q2(), imu.q3()};
            TunnelOmniInfantryTf::set_state<"imu_link">(orientation_odom_to_imu);
        }
        {
            gimbal_bullet_feeder.update_status();
            gimbal_friction_left.update_status();
            gimbal_friction_right.update_status();

            std::ranges::for_each(
                chassis_wheel_motors, [](device::DjiMotor& device) { device.update_status(); });

            gimbal_motor_yaw.update_status();
            gimbal_motor_pitch.update_status();
            // TODO:
        }
        {
            // supercap.update_status();
            dr16.update_status();
        }
    }

    auto set_enable_control(bool enable) { enable_control = enable; }

    auto set_collect_can_id(bool enable) { collect_can_id = enable; }

    auto gimbal_yaw_raw_angle() const { return gimbal_motor_yaw.raw_angle(); }

    auto gimbal_pitch_raw_angle() const { return gimbal_motor_pitch.raw_angle(); }

    auto get_can1_ids() const -> const std::unordered_set<std::uint32_t>& { return can1_ids; }
    auto get_can2_ids() const -> const std::unordered_set<std::uint32_t>& { return can2_ids; }

private:
    TransmitBuffer transmit_buffer{*this, 32};

    rclcpp::Logger logger;
    rmcs_executor::Component& master;

    librmcs::utility::RingBuffer<std::byte> referee_buffer{255};
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial;

    OutputInterface<double> velocity_gimbal_yaw;
    OutputInterface<double> velocity_gimbal_pitch;

    std::thread background_thread{
        [this] { handle_events(); },
    };

    // CAN1
    // std::uint32_t supercap_send_id{0x1FE};
    // std::uint32_t supercap_recv_id{0x300};
    // device::Supercap supercap{master, *this};

    std::uint32_t chassis_wheel_motors_send_id{0x200};
    std::array<std::uint32_t, 4> chassis_wheel_motors_recv_ids{0x201, 0x202, 0x203, 0x204};
    std::array<device::DjiMotor, 4> chassis_wheel_motors{
        device::DjiMotor{master, *this, "/chassis/left_front_wheel"},
        device::DjiMotor{master, *this, "/chassis/right_front_wheel"},
        device::DjiMotor{master, *this, "/chassis/right_back_wheel"},
        device::DjiMotor{master, *this, "/chassis/left_back_wheel"},
    };

    std::uint32_t gimbal_motor_yaw_send_id{0x145};
    std::uint32_t gimbal_motor_yaw_recv_id{0x145};
    device::LkMotor gimbal_motor_yaw{master, *this, "/gimbal/yaw"};

    // CAN2
    std::uint32_t gimbal_motor_pitch_send_id{0x142};
    std::uint32_t gimbal_motor_pitch_recv_id{0x142};
    device::LkMotor gimbal_motor_pitch{master, *this, "/gimbal/pitch"};

    std::uint32_t gimbal_shooting_device_send_id{0x200};

    std::uint32_t gimbal_bullet_feeder_recv_id{0x202};
    device::DjiMotor gimbal_bullet_feeder{master, *this, "/gimbal/bullet_feeder"};

    std::uint32_t gimbal_friction_left_recv_id{0x203};
    device::DjiMotor gimbal_friction_left{master, *this, "/gimbal/left_friction"};

    std::uint32_t gimbal_friction_right_recv_id{0x204};
    device::DjiMotor gimbal_friction_right{master, *this, "/gimbal/right_friction"};

    // OTHER
    device::Dr16 dr16{master};
    device::Bmi088 imu{1000, 0.2, 0.0};

    bool enable_control = false;
    bool collect_can_id = false;

    std::unordered_set<std::uint32_t> can1_ids;
    std::unordered_set<std::uint32_t> can2_ids;

private:
    auto can1_receive_callback(
        uint32_t id, uint64_t data, bool is_extended, bool is_remote_transmission, uint8_t length)
        -> void override {

        if (collect_can_id)
            can1_ids.insert(id);

        if (!is_extended && !is_remote_transmission && length >= 8) {
            // if (id == supercap_recv_id)
            //     supercap.store_status(data);
            if (id == chassis_wheel_motors_recv_ids.at(0))
                chassis_wheel_motors.at(0).store_status(data);
            else if (id == chassis_wheel_motors_recv_ids.at(1))
                chassis_wheel_motors.at(1).store_status(data);
            else if (id == chassis_wheel_motors_recv_ids.at(2))
                chassis_wheel_motors.at(2).store_status(data);
            else if (id == chassis_wheel_motors_recv_ids.at(3))
                chassis_wheel_motors.at(3).store_status(data);
            else if (id == gimbal_motor_yaw_recv_id)
                gimbal_motor_yaw.store_status(data);
        }
    };
    auto can2_receive_callback(
        uint32_t id, uint64_t data, bool is_extended, bool is_remote_transmission, uint8_t length)
        -> void override {

        if (collect_can_id)
            can2_ids.insert(id);

        if (!is_extended && !is_remote_transmission && length >= 8) {
            if (id == gimbal_motor_pitch_recv_id)
                gimbal_motor_pitch.store_status(data);
            else if (id == gimbal_bullet_feeder_recv_id)
                gimbal_bullet_feeder.store_status(data);
            else if (id == gimbal_friction_left_recv_id)
                gimbal_friction_left.store_status(data);
            else if (id == gimbal_friction_right_recv_id)
                gimbal_friction_right.store_status(data);
        }
    }
    auto uart1_receive_callback(const std::byte* data, uint8_t length) -> void override {
        referee_buffer.emplace_back_multi([&](std::byte* buffer) { *buffer = *data++; }, length);
    }
    auto uart2_receive_callback(const std::byte* data, uint8_t length) -> void override {
        std::ignore = data, std::ignore = length;
    }
    auto dbus_receive_callback(const std::byte* data, uint8_t length) -> void override {
        dr16.store_status(data, length);
    }
    auto accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) -> void override {
        imu.store_accelerometer_status(x, y, z);
    }
    auto gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) -> void override {
        imu.store_gyroscope_status(x, y, z);
    }

    auto update_control() -> void {
        auto command = std::array<std::uint16_t, 4>{};
        auto command_data = [&] { return std::bit_cast<std::uint64_t>(command.data()); };
        auto long_command = std::uint64_t{0};

        std::ranges::fill(command, 0);

        if (enable_control) {
            // CAN1
            // command.at(3) = supercap.generate_command();
            // transmit_buffer.add_can1_transmission(supercap_send_id, command_data());

            for (auto&& [byte, motor] : std::views::zip(command, chassis_wheel_motors)) {
                byte = motor.generate_command();
            }
            transmit_buffer.add_can1_transmission(chassis_wheel_motors_send_id, command_data());

            long_command = gimbal_motor_yaw.generate_torque_command();
            transmit_buffer.add_can1_transmission(gimbal_motor_yaw_send_id, long_command);

            // CAN2
            long_command = gimbal_motor_pitch.generate_torque_command();
            transmit_buffer.add_can2_transmission(gimbal_motor_pitch_send_id, long_command);

            command.at(0) = 0;
            command.at(1) = gimbal_bullet_feeder.generate_command();
            command.at(2) = gimbal_friction_left.generate_command();
            command.at(3) = gimbal_friction_right.generate_command();
            transmit_buffer.add_can2_transmission(gimbal_shooting_device_send_id, command_data());

            transmit_buffer.trigger_transmission();
        } else {
            // CAN1
            long_command = gimbal_motor_yaw.generate_status_request();
            transmit_buffer.add_can1_transmission(gimbal_motor_yaw_send_id, long_command);

            // CAN2
            long_command = gimbal_motor_pitch.generate_status_request();
            transmit_buffer.add_can2_transmission(gimbal_motor_pitch_send_id, long_command);
        }
    }
};

class TunnelOmniInfantry : public rmcs_executor::Component {
public:
    TunnelOmniInfantry() noexcept {
        using namespace std::chrono_literals;
        log_calibrable_device = node.get_parameter("log_calibrable_device").as_bool();
        log_framerate.set_intetval(2s);

        auto enable_control = node.get_parameter("enable_control").as_bool();
        slave_device->set_enable_control(enable_control);
        if (!enable_control) {
            RCLCPP_INFO(node.get_logger(), "Use uncontrolled mode, to clibrate or debug");
        }

        log_received_can_ids = node.get_parameter("log_received_can_ids").as_bool();
        slave_device->set_collect_can_id(log_received_can_ids);
    }

    auto update() noexcept -> void override {
        slave_device->update_peripherals();

        if (log_framerate.tick()) {
            if (log_calibrable_device) {
                const auto yaw = slave_device->gimbal_yaw_raw_angle();
                const auto pitch = slave_device->gimbal_pitch_raw_angle();
                RCLCPP_INFO(node.get_logger(), "Clibration: yaw(%5ld) pitch(%5ld)", yaw, pitch);
            }
            if (log_received_can_ids) {
                auto text = std::string{};

                text = std::string{"Can1 ids: "};
                for (auto id : slave_device->get_can1_ids()) {
                    text += std::format("0x{:X} ", id);
                }
                RCLCPP_INFO(node.get_logger(), "%s", text.c_str());

                text = std::string{"Can2 ids: "};
                for (auto id : slave_device->get_can2_ids()) {
                    text += std::format("0x{:X} ", id);
                }
                RCLCPP_INFO(node.get_logger(), "%s", text.c_str());
            }
        }
    }

private:
    rmcs_util::Framerate log_framerate;
    bool log_received_can_ids = false;
    bool log_calibrable_device = false;

    rclcpp::Node node{
        Component::get_component_name(),
        rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)};

    std::shared_ptr<SlaveDevice> slave_device{
        Component::create_partner_component<SlaveDevice>(
            std::string{get_component_name() + "_slave"}, *this, node,
            node.get_parameter_or<int>("usb_pid", -1)),
    };
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TunnelOmniInfantry, rmcs_executor::Component)
