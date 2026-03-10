#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/force_sensor.hpp"
#include "hardware/device/pwm_servo.hpp"
#include "librmcs/agent/c_board.hpp"

namespace rmcs_core::hardware {

class CatapultDartforlibrmcsv3
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    CatapultDartforlibrmcsv3()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::agent::CBoard{get_parameter("serial_filter").as_string()}
        , dart_command_(
              create_partner_component<DartCommand>(get_component_name() + "_command", *this))
        , logger_{get_logger()}
        , left_belt_motor_{*this, *this->dart_command_, "/dart/left_belt_motor"}
        , right_belt_motor_{*this, *this->dart_command_, "/dart/right_belt_motor"}
        , yaw_control_motor_{*this, *this->dart_command_, "/dart/yaw_control_motor"}
        , pitch_control_motor_{*this, *this->dart_command_, "/dart/pitch_control_motor"}
        , screw_motor_{*this, *this->dart_command_, "/dart/screw_motor"}
        , trigger_servo_{"/dart/trigger_servo", *this->dart_command_, 20.0, 0.5, 2.5}
        , force_sensor_{*this} {

        left_belt_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));

        right_belt_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                .set_reversed()
                .set_reduction_ratio(19.));

        yaw_control_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));

        pitch_control_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));

        screw_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));
    }

    CatapultDartforlibrmcsv3(const CatapultDartforlibrmcsv3&) = delete;
    CatapultDartforlibrmcsv3& operator=(const CatapultDartforlibrmcsv3&) = delete;
    CatapultDartforlibrmcsv3(CatapultDartforlibrmcsv3&&) = delete;
    CatapultDartforlibrmcsv3& operator=(CatapultDartforlibrmcsv3&&) = delete;

    ~CatapultDartforlibrmcsv3() override = default;

    void update() override { force_sensor_.update_status(); }

    void command_update() {
        auto board = start_transmit();
        board.gpio_analog_transmit({.channel = 1, .value = trigger_servo_.generate_duty_cycle()});

        board.can2_transmit({
            .can_id = 0x200,
            .can_data =
                device::CanPacket8{
                                   pitch_control_motor_.generate_command(),
                                   yaw_control_motor_.generate_command(),
                                   screw_motor_.generate_command(),
                                   device::CanPacket8::PaddingQuarter{},
                                   }
                    .as_bytes(),
        });

        board.can2_transmit({
            .can_id = 0x1FF,
            .can_data =
                device::CanPacket8{
                                   left_belt_motor_.generate_command(),
                                   right_belt_motor_.generate_command(),
                                   device::CanPacket8::PaddingQuarter{},
                                   device::CanPacket8::PaddingQuarter{},
                                   }
                    .as_bytes(),
        });

        if (pub_time_count_++ > 100) {
            board.can2_transmit({
                .can_id = 0x301,
                .can_data = device::CanPacket8{device::ForceSensor::generate_command()}.as_bytes(),
            });
            pub_time_count_ = 0;
        }
    }

protected:
    void can2_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        auto can_id = data.can_id;
        if (can_id == 0x201) {
            pitch_control_motor_.store_status(data.can_data);
        } else if (can_id == 0x202) {
            yaw_control_motor_.store_status(data.can_data);
        } else if (can_id == 0x203) {
            screw_motor_.store_status(data.can_data);
        } else if (can_id == 0x205) {
            left_belt_motor_.store_status(data.can_data);
        } else if (can_id == 0x206) {
            right_belt_motor_.store_status(data.can_data);
        } else if (can_id == 0x302) {
            force_sensor_.store_status(data.can_data);
        }
    }

private:
    class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(CatapultDartforlibrmcsv3& dart)
            : dart_(dart) {}

        void update() override { dart_.command_update(); }

    private:
        CatapultDartforlibrmcsv3& dart_;
    };
    std::shared_ptr<DartCommand> dart_command_;

    rclcpp::Logger logger_;

    device::DjiMotor left_belt_motor_, right_belt_motor_;
    device::DjiMotor yaw_control_motor_, pitch_control_motor_;
    device::DjiMotor screw_motor_;
    device::PWMServo trigger_servo_;
    device::ForceSensor force_sensor_;
    int pub_time_count_ = 0;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDartforlibrmcsv3, rmcs_executor::Component)