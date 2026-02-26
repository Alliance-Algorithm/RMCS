#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <string_view>

#include <librmcs/agent/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/encorder.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/power_meter.hpp"

namespace rmcs_core::hardware {

class Engineer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Engineer()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , engineer_command_(
              create_partner_component<EngineerCommand>(get_component_name() + "_command", *this))
        , arm_board_(*this, *engineer_command_, get_parameter("arm_board_serial").as_string())
        , left_board_(*this, *engineer_command_, get_parameter("left_board_serial").as_string())
        , right_board_(*this, *engineer_command_, get_parameter("right_board_serial").as_string()) {}

    Engineer(const Engineer&) = delete;
    Engineer& operator=(const Engineer&) = delete;
    Engineer(Engineer&&) = delete;
    Engineer& operator=(Engineer&&) = delete;

    ~Engineer() override = default;

    void update() override {
        arm_board_.update();
        left_board_.update();
        right_board_.update();
    }

    void command_update() {
        arm_board_.command_update();
        left_board_.command_update();
        right_board_.command_update();
    }

private:
    class EngineerCommand : public rmcs_executor::Component {
    public:
        explicit EngineerCommand(Engineer& engineer)
            : engineer_(engineer) {}

        void update() override { engineer_.command_update(); }

    private:
        Engineer& engineer_;
    };

    class ArmBoard final : private librmcs::agent::CBoard {
    public:
        explicit ArmBoard(
            Engineer& engineer, EngineerCommand& engineer_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , joint_motors_(
                  {engineer, engineer_command, "/arm/joint_1/motor"},
                  {engineer, engineer_command, "/arm/joint_2/motor"},
                  {engineer, engineer_command, "/arm/joint_3/motor"},
                  {engineer, engineer_command, "/arm/joint_4/motor"},
                  {engineer, engineer_command, "/arm/joint_5/motor"},
                  {engineer, engineer_command, "/arm/joint_6/motor"})
            , joint2_encoder_(engineer, "/arm/joint_2/encoder")
            , dr16_(engineer) {

            using device::Encoder;
            using device::LkMotor;

            joint_motors_[5].configure(
                LkMotor::Config{LkMotor::Type::kMHF6015}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("joint6_zero_point").as_int())));
            joint_motors_[4].configure(
                LkMotor::Config{LkMotor::Type::kMG5010Ei10}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("joint5_zero_point").as_int())));
            joint_motors_[3].configure(
                LkMotor::Config{LkMotor::Type::kMG4010Ei36}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("joint4_zero_point").as_int())));
            joint_motors_[2].configure(
                LkMotor::Config{LkMotor::Type::kMG6012Ei36}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("joint3_zero_point").as_int())));
            joint_motors_[1].configure(
                LkMotor::Config{LkMotor::Type::kMF7015V210T}
                    .set_reversed()
                    .set_reduction_ratio(42.0));
            joint_motors_[0].configure(
                LkMotor::Config{LkMotor::Type::kMG5010Ei36}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("joint1_zero_point").as_int())));

            joint2_encoder_.configure(
                Encoder::Config{Encoder::Type::kKTH7823}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("joint2_zero_point").as_int())));
        }

        ArmBoard(const ArmBoard&) = delete;
        ArmBoard& operator=(const ArmBoard&) = delete;
        ArmBoard(ArmBoard&&) = delete;
        ArmBoard& operator=(ArmBoard&&) = delete;

        ~ArmBoard() override {
            auto builder = start_transmit();
            const auto disable_command = device::LkMotor::generate_disable_command().as_bytes();

            builder.can2_transmit({.can_id = 0x145, .can_data = disable_command});
            builder.can2_transmit({.can_id = 0x144, .can_data = disable_command});
            builder.can2_transmit({.can_id = 0x141, .can_data = disable_command});
            builder.can1_transmit({.can_id = 0x142, .can_data = disable_command});
            builder.can1_transmit({.can_id = 0x143, .can_data = disable_command});
            builder.can1_transmit({.can_id = 0x146, .can_data = disable_command});
        }

        void update() {
            for (auto& motor : joint_motors_)
                motor.update_status();
            joint2_encoder_.update_status();
            dr16_.update_status();
        }

        void command_update() {
            auto builder = start_transmit();

            if (even_phase_) {
                builder.can1_transmit({
                    .can_id = 0x143,
                    .can_data = joint_motors_[2].generate_torque_command().as_bytes(),
                });
                builder.can2_transmit({
                    .can_id = 0x141,
                    .can_data = joint_motors_[5].generate_torque_command().as_bytes(),
                });
            } else {
                builder.can1_transmit({
                    .can_id = 0x146,
                    .can_data = joint_motors_[0].generate_torque_command().as_bytes(),
                });
                builder.can1_transmit({
                    .can_id = 0x142,
                    .can_data = joint_motors_[1].generate_torque_command().as_bytes(),
                });
                builder.can2_transmit({
                    .can_id = 0x145,
                    .can_data = joint_motors_[4].generate_torque_command().as_bytes(),
                });
                builder.can2_transmit({
                    .can_id = 0x144,
                    .can_data = joint_motors_[3].generate_torque_command().as_bytes(),
                });
            }

            even_phase_ = !even_phase_;
        }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
                return;

            const auto can_id = data.can_id;
            if (can_id == 0x143) {
                joint_motors_[2].store_status(data.can_data);
            } else if (can_id == 0x142) {
                joint_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x146) {
                joint_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x200) {
                joint2_encoder_.store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
                return;

            const auto can_id = data.can_id;
            if (can_id == 0x141) {
                joint_motors_[5].store_status(data.can_data);
            } else if (can_id == 0x145) {
                joint_motors_[4].store_status(data.can_data);
            } else if (can_id == 0x144) {
                joint_motors_[3].store_status(data.can_data);
            }
        }

        void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

    private:
        device::LkMotor joint_motors_[6];
        device::Encoder joint2_encoder_;
        device::Dr16 dr16_;

        bool even_phase_ = true;
    };

    class LeftBoard final : private librmcs::agent::CBoard {
    public:
        explicit LeftBoard(
            Engineer& engineer, EngineerCommand& engineer_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , steering_motors_(
                  {engineer, engineer_command, "/chassis/left_front_steering"},
                  {engineer, engineer_command, "/chassis/left_back_steering"})
            , wheel_motors_(
                  {engineer, engineer_command, "/chassis/left_front_wheel"},
                  {engineer, engineer_command, "/chassis/left_back_wheel"})
            , power_meter_(engineer, "/steering/power_meter")
            , omni_motor_(engineer, engineer_command, "/leg/omni/l")
            , leg_motors_(
                  {engineer, engineer_command, "/leg/joint/lf"},
                  {engineer, engineer_command, "/leg/joint/lb"})
            , leg_encoders_({engineer, "/leg/encoder/lf"}, {engineer, "/leg/encoder/lb"}) {

            using device::DjiMotor;
            using device::Encoder;

            steering_motors_[0].configure(
                DjiMotor::Config{DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("steering_lf_zero_point").as_int())));
            steering_motors_[1].configure(
                DjiMotor::Config{DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("steering_lb_zero_point").as_int())));

            wheel_motors_[0].configure(
                DjiMotor::Config{DjiMotor::Type::kM3508}.set_reduction_ratio(18.2));
            wheel_motors_[1].configure(
                DjiMotor::Config{DjiMotor::Type::kM3508}.set_reduction_ratio(18.2));

            leg_motors_[0].configure(
                DjiMotor::Config{DjiMotor::Type::kM3508}.set_reduction_ratio(92.0).set_reversed());
            leg_motors_[1].configure(
                DjiMotor::Config{DjiMotor::Type::kM3508}.set_reduction_ratio(277.6));

            leg_encoders_[0].configure(
                Encoder::Config{Encoder::Type::kOld}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("leg_lf_ecd_zero_point").as_int())));
            leg_encoders_[1].configure(
                Encoder::Config{Encoder::Type::kOld}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("leg_lb_ecd_zero_point").as_int())));

            omni_motor_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508}.set_reduction_ratio(18.2).set_reversed());

            engineer.register_output(
                "/leg/joint/lb/control_theta_error", leg_joint_lb_control_theta_error_,
                std::numeric_limits<double>::quiet_NaN());
            engineer.register_output(
                "/leg/joint/lf/control_theta_error", leg_joint_lf_control_theta_error_,
                std::numeric_limits<double>::quiet_NaN());

            engineer_command.register_input("/leg/joint/lb/target_theta", leg_lb_target_theta_);
            engineer_command.register_input("/leg/joint/lf/target_theta", leg_lf_target_theta_);
        }

        LeftBoard(const LeftBoard&) = delete;
        LeftBoard& operator=(const LeftBoard&) = delete;
        LeftBoard(LeftBoard&&) = delete;
        LeftBoard& operator=(LeftBoard&&) = delete;

        ~LeftBoard() override {
            auto builder = start_transmit();
            const auto zero_command =
                device::CanPacket8{
                    device::CanPacket8::PaddingQuarter{},
                    device::CanPacket8::PaddingQuarter{},
                    device::CanPacket8::PaddingQuarter{},
                    device::CanPacket8::PaddingQuarter{},
                }
                    .as_bytes();

            builder.can1_transmit({.can_id = 0x1FE, .can_data = zero_command});
            builder.can2_transmit({.can_id = 0x1FE, .can_data = zero_command});
            builder.can1_transmit({.can_id = 0x200, .can_data = zero_command});
            builder.can2_transmit({.can_id = 0x200, .can_data = zero_command});
        }

        void update() {
            omni_motor_.update_status();
            for (auto& motor : steering_motors_)
                motor.update_status();
            for (auto& motor : wheel_motors_)
                motor.update_status();
            for (auto& motor : leg_motors_)
                motor.update_status();
            for (auto& encoder : leg_encoders_)
                encoder.update_status();
            power_meter_.update_status();
        }

        void command_update() {
            auto builder = start_transmit();

            if (turn_) {
                builder.can2_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            steering_motors_[1].generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
                builder.can1_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                            steering_motors_[0].generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
            } else {
                *leg_joint_lb_control_theta_error_ =
                    normalize_angle(leg_lb_target_theta() - leg_encoders_[1].angle());
                builder.can2_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            wheel_motors_[1].generate_command(),
                            leg_motors_[1].generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });

                *leg_joint_lf_control_theta_error_ =
                    normalize_angle(leg_lf_target_theta() - leg_encoders_[0].angle());
                builder.can1_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            wheel_motors_[0].generate_command(),
                            leg_motors_[0].generate_command(),
                            omni_motor_.generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
            }

            turn_ = !turn_;
        }

    private:
        static double normalize_angle(double angle) {
            while (angle > std::numbers::pi)
                angle -= 2.0 * std::numbers::pi;
            while (angle < -std::numbers::pi)
                angle += 2.0 * std::numbers::pi;
            return angle;
        }

        double leg_lf_target_theta() const {
            if (leg_lf_target_theta_.ready()) [[likely]]
                return *leg_lf_target_theta_;
            return std::numeric_limits<double>::quiet_NaN();
        }

        double leg_lb_target_theta() const {
            if (leg_lb_target_theta_.ready()) [[likely]]
                return *leg_lb_target_theta_;
            return std::numeric_limits<double>::quiet_NaN();
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
                return;

            const auto can_id = data.can_id;
            if (can_id == 0x205) {
                steering_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x202) {
                leg_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x201) {
                wheel_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x203) {
                omni_motor_.store_status(data.can_data);
            } else if (can_id == 0x321) {
                leg_encoders_[0].store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
                return;

            const auto can_id = data.can_id;
            if (can_id == 0x207) {
                steering_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x201) {
                wheel_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x202) {
                leg_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x100) {
                power_meter_.store_status(data.can_data);
            } else if (can_id == 0x320) {
                leg_encoders_[1].store_status(data.can_data);
            }
        }

    private:
        device::DjiMotor steering_motors_[2];
        device::DjiMotor wheel_motors_[2];
        device::PowerMeter power_meter_;
        device::DjiMotor omni_motor_;
        device::DjiMotor leg_motors_[2];
        device::Encoder leg_encoders_[2];

        rmcs_executor::Component::InputInterface<double> leg_lf_target_theta_;
        rmcs_executor::Component::InputInterface<double> leg_lb_target_theta_;
        rmcs_executor::Component::OutputInterface<double> leg_joint_lb_control_theta_error_;
        rmcs_executor::Component::OutputInterface<double> leg_joint_lf_control_theta_error_;

        bool turn_ = false;
    };

    class RightBoard final : private librmcs::agent::CBoard {
    public:
        explicit RightBoard(
            Engineer& engineer, EngineerCommand& engineer_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , steering_motors_(
                  {engineer, engineer_command, "/chassis/right_back_steering"},
                  {engineer, engineer_command, "/chassis/right_front_steering"})
            , wheel_motors_(
                  {engineer, engineer_command, "/chassis/right_back_wheel"},
                  {engineer, engineer_command, "/chassis/right_front_wheel"})
            , omni_motor_(engineer, engineer_command, "/leg/omni/r")
            , leg_motors_(
                  {engineer, engineer_command, "/leg/joint/rb"},
                  {engineer, engineer_command, "/leg/joint/rf"})
            , leg_encoders_({engineer, "/leg/encoder/rb"}, {engineer, "/leg/encoder/rf"})
            , big_yaw_(engineer, engineer_command, "/chassis/big_yaw") {

            using device::DjiMotor;
            using device::DmMotor;
            using device::Encoder;

            steering_motors_[0].configure(
                DjiMotor::Config{DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("steering_rb_zero_point").as_int())));
            steering_motors_[1].configure(
                DjiMotor::Config{DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("steering_rf_zero_point").as_int())));

            wheel_motors_[0].configure(
                DjiMotor::Config{DjiMotor::Type::kM3508}.set_reduction_ratio(18.2));
            wheel_motors_[1].configure(
                DjiMotor::Config{DjiMotor::Type::kM3508}.set_reduction_ratio(18.2));

            omni_motor_.configure(
                DjiMotor::Config{DjiMotor::Type::kM3508}.set_reduction_ratio(18.2));

            leg_motors_[0].configure(
                DjiMotor::Config{DjiMotor::Type::kM3508}.set_reduction_ratio(277.6).set_reversed());
            leg_motors_[1].configure(
                DjiMotor::Config{DjiMotor::Type::kM3508}.set_reduction_ratio(92.0));

            leg_encoders_[0].configure(
                Encoder::Config{Encoder::Type::kOld}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("leg_rb_ecd_zero_point").as_int())));
            leg_encoders_[1].configure(
                Encoder::Config{Encoder::Type::kOld}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(engineer.get_parameter("leg_rf_ecd_zero_point").as_int())));

            big_yaw_.configure(
                DmMotor::Config{DmMotor::Type::kDM8009}.set_encoder_zero_point(
                    static_cast<int>(engineer.get_parameter("big_yaw_zero_point").as_int())));

            engineer.register_output(
                "/leg/joint/rb/control_theta_error", leg_joint_rb_control_theta_error_,
                std::numeric_limits<double>::quiet_NaN());
            engineer.register_output(
                "/leg/joint/rf/control_theta_error", leg_joint_rf_control_theta_error_,
                std::numeric_limits<double>::quiet_NaN());

            engineer_command.register_input("/leg/joint/rb/target_theta", leg_rb_target_theta_);
            engineer_command.register_input("/leg/joint/rf/target_theta", leg_rf_target_theta_);
        }

        RightBoard(const RightBoard&) = delete;
        RightBoard& operator=(const RightBoard&) = delete;
        RightBoard(RightBoard&&) = delete;
        RightBoard& operator=(RightBoard&&) = delete;

        ~RightBoard() override {
            auto builder = start_transmit();
            const auto zero_command =
                device::CanPacket8{
                    device::CanPacket8::PaddingQuarter{},
                    device::CanPacket8::PaddingQuarter{},
                    device::CanPacket8::PaddingQuarter{},
                    device::CanPacket8::PaddingQuarter{},
                }
                    .as_bytes();

            builder.can1_transmit({.can_id = 0x1FE, .can_data = zero_command});
            builder.can2_transmit({.can_id = 0x1FE, .can_data = zero_command});
            builder.can1_transmit({.can_id = 0x200, .can_data = zero_command});
            builder.can2_transmit({.can_id = 0x200, .can_data = zero_command});
            builder.can1_transmit({.can_id = 0x3, .can_data = device::DmMotor::generate_close_command().as_bytes()});
        }

        void update() {
            omni_motor_.update_status();
            for (auto& motor : steering_motors_)
                motor.update_status();
            for (auto& motor : wheel_motors_)
                motor.update_status();
            for (auto& motor : leg_motors_)
                motor.update_status();
            for (auto& encoder : leg_encoders_)
                encoder.update_status();
            big_yaw_.update_status();
        }

        void command_update() {
            auto builder = start_transmit();

            if (turn_) {
                builder.can2_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            steering_motors_[0].generate_command(),
                        }
                            .as_bytes(),
                });
                builder.can1_transmit({
                    .can_id = 0x1FE,
                    .can_data =
                        device::CanPacket8{
                            device::CanPacket8::PaddingQuarter{},
                            steering_motors_[1].generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
                builder.can1_transmit({
                    .can_id = 0x3,
                    .can_data = big_yaw_.generate_torque_command().as_bytes(),
                });
            } else {
                *leg_joint_rb_control_theta_error_ =
                    normalize_angle(leg_rb_target_theta() - leg_encoders_[0].angle());
                builder.can2_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            wheel_motors_[0].generate_command(),
                            leg_motors_[0].generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });

                *leg_joint_rf_control_theta_error_ =
                    normalize_angle(leg_rf_target_theta() - leg_encoders_[1].angle());
                builder.can1_transmit({
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            wheel_motors_[1].generate_command(),
                            leg_motors_[1].generate_command(),
                            omni_motor_.generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
            }

            turn_ = !turn_;
        }

    private:
        static double normalize_angle(double angle) {
            while (angle > std::numbers::pi)
                angle -= 2.0 * std::numbers::pi;
            while (angle < -std::numbers::pi)
                angle += 2.0 * std::numbers::pi;
            return angle;
        }

        double leg_rf_target_theta() const {
            if (leg_rf_target_theta_.ready()) [[likely]]
                return *leg_rf_target_theta_;
            return std::numeric_limits<double>::quiet_NaN();
        }

        double leg_rb_target_theta() const {
            if (leg_rb_target_theta_.ready()) [[likely]]
                return *leg_rb_target_theta_;
            return std::numeric_limits<double>::quiet_NaN();
        }

        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
                return;

            const auto can_id = data.can_id;
            if (can_id == 0x201) {
                wheel_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x202) {
                leg_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x203) {
                omni_motor_.store_status(data.can_data);
            } else if (can_id == 0x206) {
                steering_motors_[1].store_status(data.can_data);
            } else if (can_id == 0x322) {
                leg_encoders_[1].store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() < 8)
                return;

            const auto can_id = data.can_id;
            if (can_id == 0x201) {
                wheel_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x202) {
                leg_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x208) {
                steering_motors_[0].store_status(data.can_data);
            } else if (can_id == 0x33) {
                big_yaw_.store_status(data.can_data);
            } else if (can_id == 0x319) {
                leg_encoders_[0].store_status(data.can_data);
            }
        }

    private:
        device::DjiMotor steering_motors_[2];
        device::DjiMotor wheel_motors_[2];
        device::DjiMotor omni_motor_;
        device::DjiMotor leg_motors_[2];
        device::Encoder leg_encoders_[2];
        device::DmMotor big_yaw_;

        rmcs_executor::Component::InputInterface<double> leg_rf_target_theta_;
        rmcs_executor::Component::InputInterface<double> leg_rb_target_theta_;
        rmcs_executor::Component::OutputInterface<double> leg_joint_rb_control_theta_error_;
        rmcs_executor::Component::OutputInterface<double> leg_joint_rf_control_theta_error_;

        bool turn_ = false;
    };

private:
    std::shared_ptr<EngineerCommand> engineer_command_;

    ArmBoard arm_board_;
    LeftBoard left_board_;
    RightBoard right_board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Engineer, rmcs_executor::Component)
