#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>

namespace rmcs_core::hardware::ec {

using namespace rmcs_description;

class Command
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Command()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        register_input("/serial", serial_);

        register_input("/tf", tf_);
        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_);
    }
    ~Command() = default;

    void update() override {
        double yaw_angle_error, pitch_angle_error;
        Eigen::Vector3d auto_aim_control_direction = *auto_aim_control_direction_;

        if (!auto_aim_control_direction.isZero()) {
            PitchLink::DirectionVector dir = fast_tf::cast<PitchLink>(
                OdomImu::DirectionVector{auto_aim_control_direction}, *tf_);
            double &x = dir->x(), &y = dir->y(), &z = dir->z();
            yaw_angle_error   = std::atan2(y, x);
            pitch_angle_error = -std::atan2(z, std::sqrt(y * y + x * x));
        } else {
            yaw_angle_error = pitch_angle_error = 0.0;
        }

        SendStruct send_struct;
        send_struct.yaw   = static_cast<float>(yaw_angle_error);
        send_struct.pitch = static_cast<float>(pitch_angle_error);
        const_cast<serial::Serial&>(*serial_).write(
            reinterpret_cast<uint8_t*>(&send_struct), sizeof(send_struct));

        // RCLCPP_INFO(get_logger(), "%f %f", yaw_angle_error, pitch_angle_error);
    }

private:
    InputInterface<serial::Serial> serial_;

    InputInterface<Tf> tf_;
    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;

    struct SendStruct {
        float yaw, pitch;
    };
};

} // namespace rmcs_core::hardware::ec

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ec::Command, rmcs_executor::Component)