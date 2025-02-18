
#include <chrono>
#include <queue>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

namespace rmcs_core::controller::dart {

class DataRecorder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DataRecorder()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        friction_working_velocity_ = get_parameter("working_velocity").as_double();

        register_input("/dart/friction_lf/velocity", friction_lf_velocity_);
        register_input("/dart/friction_rf/velocity", friction_rf_velocity_);
        register_input("/dart/friction_lb/velocity", friction_lb_velocity_);
        register_input("/dart/friction_rb/velocity", friction_rb_velocity_);

        publisher_1_ = this->create_publisher<std_msgs::msg::String>("msg_friction_lf_current_velocity_", 10);
        publisher_2_ = this->create_publisher<std_msgs::msg::String>("msg_friction_lb_current_velocity_", 10);
        publisher_3_ = this->create_publisher<std_msgs::msg::String>("msg_friction_rb_current_velocity_", 10);
        publisher_4_ = this->create_publisher<std_msgs::msg::String>("msg_friction_rf_current_velocity_", 10);
    }

    void update() override {
        update_command();
        update_buffer_data();

        if (data_copy_enable_) {
            package_friction_velocity_lf_ = buffer_friction_lf_;
            package_friction_velocity_rf_ = buffer_friction_rf_;
            package_friction_velocity_lb_ = buffer_friction_lb_;
            package_friction_velocity_rb_ = buffer_friction_rb_;
            data_copy_enable_             = false;
            publisher_enable_             = true;
        }
    }

private:
    void update_command() {
        if ((*friction_lb_velocity_ + *friction_rb_velocity_) / 2 <= friction_working_velocity_ - 50) {
            launch_state_ = true;
        }

        if (abs((*friction_lb_velocity_ + *friction_rb_velocity_) / 2 - friction_working_velocity_) < 15) {
            stable_state_ = true;
            if (launch_state_ == true) {
                data_copy_enable_ = true;
                launch_state_     = false;
            }
        } else {
            stable_state_ = false;
        }
    }

    void update_buffer_data() {

        if (buffer_friction_lf_.size() == 3000) {
            // 可优化：换环形队列
            buffer_friction_lf_.pop();
            buffer_friction_rf_.pop();
            buffer_friction_lb_.pop();
            buffer_friction_rb_.pop();
        }

        buffer_friction_lf_.push(*friction_lf_velocity_);
        buffer_friction_rf_.push(*friction_rf_velocity_);
        buffer_friction_lb_.push(*friction_lb_velocity_);
        buffer_friction_rb_.push(*friction_rb_velocity_);
    }

    void package_publisher() {
        while (true) {
            auto time_point_A = std::chrono::steady_clock::now();

            if (publisher_enable_) {
                std_msgs::msg::String msg_friction_lf_;

                while (!package_friction_velocity_lf_.empty()) {}
                // TODO: publish part，see code in MessagePublisher
            }
            auto time_point_B = std::chrono::steady_clock::now();
            auto delat_count =
                std::chrono::duration_cast<std::chrono::microseconds>(time_point_B - time_point_A).count();
            std::this_thread::sleep_for(std::chrono::microseconds(5000 - delat_count));
        }
    }

    rclcpp::Logger logger_;
    bool launch_state_     = false;
    bool stable_state_     = false;
    bool data_copy_enable_ = false;
    bool publisher_enable_ = false;

    double friction_working_velocity_;

    InputInterface<double> friction_lf_velocity_;
    InputInterface<double> friction_rf_velocity_;
    InputInterface<double> friction_lb_velocity_;
    InputInterface<double> friction_rb_velocity_;

    std::queue<double> buffer_friction_lf_;
    std::queue<double> buffer_friction_rf_;
    std::queue<double> buffer_friction_lb_;
    std::queue<double> buffer_friction_rb_;

    std::queue<double> package_friction_velocity_lf_;
    std::queue<double> package_friction_velocity_rf_;
    std::queue<double> package_friction_velocity_lb_;
    std::queue<double> package_friction_velocity_rb_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_3_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_4_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DataRecorder, rmcs_executor::Component)

/*
速度回显错位是foxglove的锅
等以后需要收集在场上的数据的时候拿来修修补补应该还能用
*/