#include <cmath>

#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"



namespace rmcs_core::controller::test{
class GantryStateController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
        GantryStateController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , vel_filter_(20.0, 1000.0)
        , angle_to_velocity_left_pid_calculator_(
            get_parameter("kp1").as_double(),
            get_parameter("ki1").as_double(),
            get_parameter("kd1").as_double())
        , angle_to_velocity_right_pid_calculator_(
            get_parameter("kp1").as_double(),
            get_parameter("ki1").as_double(),
            get_parameter("kd1").as_double())
        , velocity_to_torque_left_pid_calculator_(
            get_parameter("kp2").as_double(),
            get_parameter("ki2").as_double(),
            get_parameter("kd2").as_double())
        ,velocity_to_torque_right_pid_calculator_(
            get_parameter("kp2").as_double(),
            get_parameter("ki2").as_double(),
            get_parameter("kd2").as_double()
        )          {
                angle_to_velocity_left_pid_calculator_.integral_min = this->get_parameter_or("integral_min1", -inf);
                angle_to_velocity_left_pid_calculator_.integral_max = this->get_parameter_or("integral_max1",  inf);
                angle_to_velocity_left_pid_calculator_.integral_split_min = this->get_parameter_or("integral_split_min1", -inf);
                angle_to_velocity_left_pid_calculator_.integral_split_max = this->get_parameter_or("integral_split_max1", inf);
                angle_to_velocity_left_pid_calculator_.output_min = this->get_parameter_or("output_min1", -inf);
                angle_to_velocity_left_pid_calculator_.output_max = this->get_parameter_or("output_max1", inf);

                angle_to_velocity_right_pid_calculator_.integral_min = this->get_parameter_or("integral_min1", -inf);
                angle_to_velocity_right_pid_calculator_.integral_max = this->get_parameter_or("integral_max1",  inf);
                angle_to_velocity_right_pid_calculator_.integral_split_min = this->get_parameter_or("integral_split_min1", -inf);
                angle_to_velocity_right_pid_calculator_.integral_split_max = this->get_parameter_or("integral_split_max1", inf);
                angle_to_velocity_right_pid_calculator_.output_min = this->get_parameter_or("output_min1", -inf);
                angle_to_velocity_right_pid_calculator_.output_max = this->get_parameter_or("output_max1", inf);


                velocity_to_torque_left_pid_calculator_.integral_min = this->get_parameter_or("integral_min2", -inf);
                velocity_to_torque_left_pid_calculator_.integral_max = this->get_parameter_or("integral_max2",  inf);
                velocity_to_torque_left_pid_calculator_.integral_split_min = this->get_parameter_or("integral_split_min2", -inf);
                velocity_to_torque_left_pid_calculator_.integral_split_max = this->get_parameter_or("integral_split_max2", inf);
                velocity_to_torque_left_pid_calculator_.output_min = this->get_parameter_or("output_min2", -inf);
                velocity_to_torque_left_pid_calculator_.output_max = this->get_parameter_or("output_max2", inf);


                velocity_to_torque_right_pid_calculator_.integral_min = this->get_parameter_or("integral_min2", -inf);
                velocity_to_torque_right_pid_calculator_.integral_max = this->get_parameter_or("integral_max2",  inf);
                velocity_to_torque_right_pid_calculator_.integral_split_min = this->get_parameter_or("integral_split_min2", -inf);
                velocity_to_torque_right_pid_calculator_.integral_split_max = this->get_parameter_or("integral_split_max2", inf);
                velocity_to_torque_right_pid_calculator_.output_min = this->get_parameter_or("output_min2", -inf);
                velocity_to_torque_right_pid_calculator_.output_max = this->get_parameter_or("output_max2", inf);

                register_input("/gantry/left_motor/max_torque", motor_max_control_torque_);
                register_input("/gantry/control_angle", gantry_control_angle_);
                register_input("/gantry/left_motor/angle",left_motor_angle_);
                register_input("/gantry/right_motor/angle",right_motor_angle_);
                register_input("/gantry/left_motor/velocity", left_motor_velocity_);
                register_input("/gantry/right_motor/velocity", right_motor_velocity_);
                register_output("/gantry/left_motor/filtered_velocity", left_motor_filtered_velocity_);
                register_output("/gantry/right_motor/filtered_velocity", right_motor_filtered_velocity_);
                register_output(
                "/gantry/left_motor/control_torque", left_motor_control_torque_,nan);
                register_output(
                "/gantry/right_motor/control_torque", right_motor_control_torque_, nan);
            }

            void before_updating() override {
                RCLCPP_INFO(
                get_logger(), "Max control torque of gantry motor: %.f",
                *motor_max_control_torque_);
            }
            void update() override{
                if (std::isnan(*gantry_control_angle_)) {
                    reset_all_controls();
                    return;
                }
                *left_motor_filtered_velocity_ = vel_filter_.update(*left_motor_velocity_);
                *right_motor_filtered_velocity_ = vel_filter_.update(*right_motor_velocity_);
                Eigen::Vector2d gantry_angle={*left_motor_angle_,*right_motor_angle_};
                Eigen::Vector2d gantry_velocity={*left_motor_filtered_velocity_,*right_motor_filtered_velocity_};

                Eigen::Vector2d control_velocity_ = calculate_control_velocity(gantry_angle,*gantry_control_angle_);
                Eigen::Vector2d control_torques_ = calculate_control_torques(gantry_velocity, control_velocity_, capture_leftandright_angle_err(gantry_angle));

                *left_motor_control_torque_ = control_torques_.x();
                *right_motor_control_torque_ = control_torques_.y();

            }


            Eigen::Vector2d calculate_control_velocity(const Eigen::Vector2d& gantry_angle,const double& control_angle){
                Eigen::Vector2d pid_velocity;
                pid_velocity.x()=angle_to_velocity_left_pid_calculator_.update(control_angle-gantry_angle.x());
                pid_velocity.y()=angle_to_velocity_right_pid_calculator_.update(control_angle-gantry_angle.y());
                return pid_velocity;
            }//都是默认x为左，y为右

             Eigen::Vector2d calculate_control_torques(const Eigen::Vector2d& gantry_velocity,Eigen::Vector2d control_velocity,const double& angle_err){
                Eigen::Vector2d pid_torques;
                pid_torques.x()=velocity_to_torque_left_pid_calculator_.update(control_velocity.x()-gantry_velocity.x()-angle_err);
                pid_torques.y()=velocity_to_torque_right_pid_calculator_.update(control_velocity.y()-gantry_velocity.y()+angle_err);
                return pid_torques;
            }

            void reset_all_controls() {
                *left_motor_control_torque_ = 0.0;
                *right_motor_control_torque_ = 0.0;
            }            

            static double capture_leftandright_angle_err(const Eigen::Vector2d& gantry_angle){
                double angle_err=gantry_angle.x()-gantry_angle.y();
                return angle_err;
            }

            static double place_translate_to_angle(double place){
                double angle;
                angle = place*(2*std::numbers::pi /6);
                return angle;
            }

private:
        rclcpp::Logger logger_;
        filter::LowPassFilter<1> vel_filter_;
        static constexpr double inf = std::numeric_limits<double>::infinity();
        static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        InputInterface<double> motor_max_control_torque_;

        InputInterface<double> left_motor_angle_;
        InputInterface<double> right_motor_angle_;
        InputInterface<double> left_motor_velocity_;
        InputInterface<double> right_motor_velocity_;

        InputInterface<double> gantry_control_angle_;     

        pid::PidCalculator angle_to_velocity_left_pid_calculator_;
        pid::PidCalculator angle_to_velocity_right_pid_calculator_;
        pid::PidCalculator velocity_to_torque_left_pid_calculator_;
        pid::PidCalculator velocity_to_torque_right_pid_calculator_;
        OutputInterface<double> left_motor_filtered_velocity_;
        OutputInterface<double> right_motor_filtered_velocity_;
        OutputInterface<double> left_motor_control_torque_;
        OutputInterface<double> right_motor_control_torque_;

};


}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::test::GantryStateController, rmcs_executor::Component)