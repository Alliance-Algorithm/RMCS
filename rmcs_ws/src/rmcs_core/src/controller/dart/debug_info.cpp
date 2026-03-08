#include <cstdint>
#include <deque>
#include <cmath>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {
class Test
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Test()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        register_input("/dart/pitch_motor/control_torque", pitch_control_torque_);
        register_input("/dart/yaw_motor/control_torque", yaw_control_torque_);
        register_input("/dart/pitch_motor/velocity", pitch_speed_);
        register_input("/dart/yaw_motor/velocity", yaw_speed_);
        register_input("/imu/catapult_roll_angle", final_roll_);
        register_input("/imu/catapult_pitch_angle", final_pitch_);
        register_input("/imu/catapult_yaw_angle", final_yaw_);
        register_input("/force_sensor/channel_1/weight", force_sensor_ch1_data_);
        register_input("/force_sensor/channel_2/weight", force_sensor_ch2_data_);
        register_input("/dart/lifting_left/current_angle", lifting_angle_left_);
        register_input("/dart/lifting_right/current_angle", lifting_angle_right_);
        register_input("/dart/lifting_left/control_angle", lifting_control_angle_left_);
        register_input("/dart/lifting_right/control_angle", lifting_control_angle_right_);

        lifting_up_angle_left_ = get_parameter("lifting_up_angle_left").as_int();
        lifting_down_angle_left_ = get_parameter("lifting_down_angle_left").as_int();
        lifting_up_angle_right_ = get_parameter("lifting_up_angle_right").as_int();
        lifting_down_angle_right_ = get_parameter("lifting_down_angle_right").as_int();

        declare_parameter("fit_window_size", 50);
        declare_parameter("fit_threshold", 1.0);
        declare_parameter("stable_min_points", 20);
        declare_parameter("stable_region_timeout", 500);
        
        fit_window_size_ = static_cast<int>(get_parameter("fit_window_size").as_int());
        fit_threshold_ = get_parameter("fit_threshold").as_double();
        stable_min_points_ = static_cast<int>(get_parameter("stable_min_points").as_int());
        stable_region_timeout_ = static_cast<int>(get_parameter("stable_region_timeout").as_int());
        
        total_query_pairs_ = 0;
        sync_query_pairs_ = 0;
        left_has_pending_ = false;
        right_has_pending_ = false;
        left_pending_delta_ = 0;
        right_pending_delta_ = 0;
    }
    
    void update() override {
        // if (count_n % 100 == 0) {
        //     RCLCPP_INFO(logger_, "left_force: %d, right_force: %d", 
        //                 *force_sensor_ch1_data_, *force_sensor_ch2_data_);
        // }
        // count_n++;
        if (*lifting_angle_left_ != last_left_angle_for_sync_) {
            int16_t delta = calculateAngleDelta(last_left_angle_for_sync_, *lifting_angle_left_);
            last_left_angle_for_sync_ = *lifting_angle_left_;
            left_has_pending_ = true;
            left_pending_delta_ = delta;
        }

        if (*lifting_angle_right_ != last_right_angle_for_sync_) {
            int16_t delta = calculateAngleDelta(last_right_angle_for_sync_, *lifting_angle_right_);
            last_right_angle_for_sync_ = *lifting_angle_right_;
            right_has_pending_ = true;
            right_pending_delta_ = delta;
        }

        if (left_has_pending_ && right_has_pending_) {
            total_query_pairs_++;
            int16_t delta_diff = std::abs(left_pending_delta_ - right_pending_delta_);
            if (delta_diff <= 3) {
                sync_query_pairs_++;
            } else {
                RCLCPP_WARN(logger_, "Lift sync mismatch: left delta=%d, right delta=%d, diff=%d",
                            left_pending_delta_, right_pending_delta_, delta_diff);
            }
            left_has_pending_ = false;
            right_has_pending_ = false;
        }
        
        recordAngleToWindow();
        
        checkStableRegion();
        
        if (is_in_stable_region_) {
            calculateVibrationInStableRegion();
        }
        
        if (count_n % 1000 == 0) {
            syn = (count_n > 1) ? (double)sync_coefficient_count_ / (count_n - 1) : 0.0;
            double sync_ratio = (total_query_pairs_ > 0) ? (double)sync_query_pairs_ / total_query_pairs_ : 0.0;

            if (total_stable_points_ > 0) {
                vib = (double)stable_at_control_angle_count_ / total_stable_points_;
                RCLCPP_INFO(logger_,
                    "left_angle=%d, right_angle=%d | "
                    "cumulative_sync=%.4f, per_change_sync=%.4f (pairs=%ld/%ld), vib=%.4f",
                    *lifting_angle_left_, *lifting_angle_right_,
                    syn, sync_ratio, sync_query_pairs_, total_query_pairs_, vib);
            } else {
                RCLCPP_INFO(logger_,
                    "left_angle=%d, right_angle=%d | "
                    "cumulative_sync=%.4f, per_change_sync=%.4f (pairs=%ld/%ld), vib=N/A",
                    *lifting_angle_left_, *lifting_angle_right_,
                    syn, sync_ratio, sync_query_pairs_, total_query_pairs_);
            }
        }
        
        count_n++;
    }
    
private:
    static int16_t calculateAngleDelta(uint16_t last_angle, uint16_t current_angle) {
        int16_t delta = static_cast<int16_t>(current_angle - last_angle);
        
        if (delta > 32767) {
            delta -= static_cast<int16_t>(65536);
        } else if (delta < -32768) {
            delta += static_cast<int16_t>(65536);
        }
        
        return delta;
    }
    
    void recordAngleToWindow() {
        angle_window_.push_back(*lifting_angle_left_);
        if (angle_window_.size() > fit_window_size_) {
            angle_window_.pop_front();
        }
    }
    
    void checkStableRegion() {
        if (angle_window_.size() < stable_min_points_) {
            stable_region_timeout_counter_ = 0;
            return;
        }

        double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
        size_t n = angle_window_.size();
        
        for (size_t i = 0; i < n; ++i) {
            double x = static_cast<double>(i);
            double y = static_cast<double>(angle_window_[i]);
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_xx += x * x;
        }
        
        double b = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        double a = (sum_y - b * sum_x) / n;
        
        double residual_sum = 0.0;
        for (size_t i = 0; i < n; ++i) {
            double x = static_cast<double>(i);
            double y = static_cast<double>(angle_window_[i]);
            double y_fit = a + b * x;
            residual_sum += (y - y_fit) * (y - y_fit);
        }
        
        double rms_error = sqrt(residual_sum / n);
        
        bool is_stable = (std::abs(b) < 0.05 && rms_error < fit_threshold_);
        
        if (is_stable) {
            if (!is_in_stable_region_) {
                is_in_stable_region_ = true;
                current_stable_start_point_ = angle_window_[angle_window_.size() - 1];
                stable_region_timeout_counter_ = 0;
                
                // RCLCPP_DEBUG(logger_, "Enter stable region: slope=%.4f, rms=%.4f", b, rms_error);
            }
        } else {
            if (is_in_stable_region_) {
                stable_region_timeout_counter_++;
                if (stable_region_timeout_counter_ > stable_region_timeout_) {
                    is_in_stable_region_ = false;
                    stable_region_timeout_counter_ = 0;
                    
                    if (current_stable_total_points_ > 0) {
                        double stable_ratio = static_cast<double>(current_stable_at_control_angle_) / 
                                             current_stable_total_points_;
                        // RCLCPP_DEBUG(logger_, "Exit stable region: total=%d, at control=%d, ratio=%.4f",
                        //             current_stable_total_points_, current_stable_at_control_angle_, stable_ratio);
                    }
                    
                    current_stable_total_points_ = 0;
                    current_stable_at_control_angle_ = 0;
                }
            }
        }
    }
    
    void calculateVibrationInStableRegion() {
        current_stable_total_points_++;
        if (*lifting_angle_left_ == *lifting_control_angle_left_ &&
            *lifting_angle_right_ == *lifting_control_angle_right_) {
            current_stable_at_control_angle_++;
        }
        
        total_stable_points_++;
        if (*lifting_angle_left_ == *lifting_control_angle_left_ &&
            *lifting_angle_right_ == *lifting_control_angle_right_) {
            stable_at_control_angle_count_++;
        }
    }
    
private:
    int count_n = 1;
    int sync_coefficient_count_ = 0;
    rclcpp::Logger logger_;
    double syn = 0.0;
    double vib = 0.0;
    
    uint16_t last_left_angle_for_sync_ = 0;
    uint16_t last_right_angle_for_sync_ = 0;
    bool left_has_pending_;
    bool right_has_pending_;
    int16_t left_pending_delta_;
    int16_t right_pending_delta_;
    int64_t total_query_pairs_;
    int64_t sync_query_pairs_;
    
    std::deque<uint16_t> angle_window_;
    
    bool is_in_stable_region_ = false;
    int stable_region_timeout_counter_ = 0;
    uint16_t current_stable_start_point_ = 0;
    
    int current_stable_total_points_ = 0;
    int current_stable_at_control_angle_ = 0;
    
    int total_stable_points_ = 0;
    int stable_at_control_angle_count_ = 0;
    
    int fit_window_size_ = 50;
    double fit_threshold_ = 1.0;
    int stable_min_points_ = 20;
    int stable_region_timeout_ = 500;
    
    InputInterface<double> pitch_control_torque_;
    InputInterface<double> yaw_control_torque_;
    InputInterface<double> pitch_speed_;
    InputInterface<double> yaw_speed_;
    InputInterface<double> final_pitch_;
    InputInterface<double> final_roll_;
    InputInterface<double> final_yaw_;
    InputInterface<int> force_sensor_ch1_data_;
    InputInterface<int> force_sensor_ch2_data_;
    InputInterface<uint16_t> lifting_angle_left_;
    InputInterface<uint16_t> lifting_angle_right_;
    InputInterface<uint16_t> lifting_control_angle_left_;
    InputInterface<uint16_t> lifting_control_angle_right_;
    
    uint16_t lifting_up_angle_left_;
    uint16_t lifting_middle_angle_left_;
    uint16_t lifting_down_angle_left_;
    uint16_t lifting_up_angle_right_;
    uint16_t lifting_middle_angle_right_;
    uint16_t lifting_down_angle_right_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::Test, rmcs_executor::Component)