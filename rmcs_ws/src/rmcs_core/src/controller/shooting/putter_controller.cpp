#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_condiction.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::shooting {

/**
 * @class PutterController
 * @brief 推弹机构控制器
 *
 * 发射机制说明：
 * 由于光电门放置于弹舱口，经测试，双中先触发推杆向后复位，然后堵转检测复位完毕，
 * 默认情况下会给一点点的力保证推杆不会滑下去，再然后上弹采用速度环，在光电门被触发时
 * 记录角度并转为角度环，然后推杆检测发弹根据两部分来检测，一是摩擦轮，二是推杆的行程
 * 整套方案以于暑假前完成压力测试
 */
class PutterController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PutterController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/friction_ready", friction_ready_);

        register_input("/gimbal/bullet_feeder/angle", bullet_feeder_angle_);
        register_input("/gimbal/bullet_feeder/velocity", bullet_feeder_velocity_);

        register_input(
            "/gimbal/control_bullet_allowance/limited_by_heat",
            control_bullet_allowance_limited_by_heat_);

        register_input("/gimbal/photoelectric_sensor", photoelectric_sensor_status_);
        register_input("/gimbal/bullet_fired", bullet_fired_);

        register_input("/gimbal/putter/angle", putter_angle_);
        register_input("/gimbal/putter/velocity", putter_velocity_);

        last_preload_flag_ = false;

        bullet_feeder_velocity_pid_.kp = 50.0;
        bullet_feeder_velocity_pid_.ki = 10.0;
        bullet_feeder_velocity_pid_.kd = 0.0;
        bullet_feeder_velocity_pid_.integral_max = 60.0;
        bullet_feeder_velocity_pid_.integral_min = 0.0;

        bullet_feeder_angle_pid_.kp = 8.0;
        bullet_feeder_angle_pid_.ki = 0.0;
        bullet_feeder_angle_pid_.kd = 2.0;

        putter_return_velocity_pid_.kp = 0.0015;
        putter_return_velocity_pid_.ki = 0.00005;
        putter_return_velocity_pid_.kd = 0.;
        putter_return_velocity_pid_.integral_max = 0.;
        putter_return_velocity_pid_.integral_min = -0.03;

        putter_velocity_pid_.kp = 0.004;
        putter_velocity_pid_.ki = 0.0001;
        putter_velocity_pid_.kd = 0.001;
        putter_velocity_pid_.integral_max = 0.03;
        putter_velocity_pid_.integral_min = 0.;

        putter_return_angle_pid.kp = 0.0001;
        // putter_return_angle_pid.ki = 0.000001;
        putter_return_angle_pid.kd = 0.;

        register_output(
            "/gimbal/bullet_feeder/control_torque", bullet_feeder_control_torque_, nan_);
        register_output("/gimbal/putter/control_torque", putter_control_torque_, nan_);

        register_output("/gimbal/shooter/mode", shoot_mode_, rmcs_msgs::ShootMode::SINGLE);
        register_output("/gimbal/shooter/condiction", shoot_condiction_);
    }

    void update() override {
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto mouse = *mouse_;
        const auto keyboard = *keyboard_;

        using namespace rmcs_msgs;

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }

        if (*friction_ready_) {
            if (shoot_stage_ == ShootStage::RESETTING || bullet_feeder_cool_down_ > 0) {
                *shoot_condiction_ = rmcs_msgs::ShootCondiction::JAM;
            } else if (shoot_stage_ == ShootStage::PRELOADED) {
                *shoot_condiction_ = rmcs_msgs::ShootCondiction::SHOOT;
            } else if (shoot_stage_ == ShootStage::SHOOTING) {
                *shoot_condiction_ = rmcs_msgs::ShootCondiction::FIRED;
            } else {
                *shoot_condiction_ = rmcs_msgs::ShootCondiction::PRELOADING;
            }
        } else {
            *shoot_condiction_ = rmcs_msgs::ShootCondiction::FRICTION_WAITING;
        }

        // 推杆已初始化后的正常控制流程
        if (putter_initialized) {
            // 供弹轮卡弹保护冷却期间的处理
            if (bullet_feeder_cool_down_ > 0) {
                bullet_feeder_cool_down_--;

                // 使用角度环反转到“后退一格”的位置以解除卡弹
                double velocity_err = bullet_feeder_angle_pid_.update(
                                          bullet_feeder_control_angle_ - *bullet_feeder_angle_)
                                    - *bullet_feeder_velocity_;
                *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(velocity_err);

                if (!bullet_feeder_cool_down_) {
                    RCLCPP_INFO(get_logger(), "Jamming Solved, Retrying...");
                    set_resetting();
                }
            } else {
                // 正常运行模式：摩擦轮就绪时才允许发射
                if (*friction_ready_) {

                    // 发射触发检测
                    // RCLCPP_INFO(get_logger(), "%.2f", *bullet_feeder_angle_);
                    // RCLCPP_INFO(get_logger(), "%.2f", *bullet_feeder_velocity_);
                    if (switch_right != Switch::DOWN) {
                        if ((!last_mouse_.left && mouse.left)
                            || (last_switch_left_ == rmcs_msgs::Switch::MIDDLE
                                && switch_left == rmcs_msgs::Switch::DOWN)) {
                            RCLCPP_INFO(
                                get_logger(), "now :%ld",
                                *control_bullet_allowance_limited_by_heat_);
                            if (*control_bullet_allowance_limited_by_heat_ > 0
                                && (shoot_stage_ == ShootStage::PRELOADED || shoot_first)) {
                                RCLCPP_INFO(
                                    get_logger(), "shoot: %ld",
                                    *control_bullet_allowance_limited_by_heat_);
                                set_shooting();
                                shoot_first = false;
                            }
                        }
                    }

                    if (shoot_stage_ == ShootStage::UPDATING) {
                        wait_bullet_ready();
                    }

                    if (shoot_stage_ == ShootStage::PRELOADING) {
                        // 盲拨模式：始终执行角度环定位
                        if (std::isnan(bullet_feeder_control_angle_)) {
                            bullet_feeder_control_angle_ =
                                *bullet_feeder_angle_ + bullet_feeder_angle_per_bullet_;
                            last_preload_flag_ = true;
                        }

                        const auto angle_err = bullet_feeder_control_angle_ - *bullet_feeder_angle_;
                        if (angle_err < 0.1) {
                            set_preloaded();
                        }
                        double velocity_err =
                            bullet_feeder_angle_pid_.update(angle_err) - *bullet_feeder_velocity_;
                        *bullet_feeder_control_torque_ =
                            bullet_feeder_velocity_pid_.update(velocity_err);

                        update_jam_detection();
                    } else if (
                        shoot_stage_ == ShootStage::SHOOTING
                        || shoot_stage_ == ShootStage::UPDATING) {
                        // 供弹轮不给力
                        bullet_feeder_velocity_pid_.reset();
                        bullet_feeder_angle_pid_.reset();
                        *bullet_feeder_control_torque_ = 0.;
                    } else if (shoot_stage_ == ShootStage::RESETTING) {

                        const auto angle_err = bullet_feeder_control_angle_ - *bullet_feeder_angle_;
                        if (angle_err < 0.1) {
                            RCLCPP_INFO(get_logger(), "RESETED");
                            set_preloaded();
                        }
                        double velocity_err =
                            bullet_feeder_angle_pid_.update(angle_err) - *bullet_feeder_velocity_;
                        *bullet_feeder_control_torque_ =
                            bullet_feeder_velocity_pid_.update(velocity_err);

                    } else {
                        // 其他状态：角度环保持角度不变防止弹链退弹
                        double velocity_err =
                            bullet_feeder_angle_pid_.update(
                                bullet_feeder_control_angle_ - *bullet_feeder_angle_)
                            - *bullet_feeder_velocity_;
                        *bullet_feeder_control_torque_ =
                            bullet_feeder_velocity_pid_.update(velocity_err);
                    }

                    if (shoot_stage_ == ShootStage::SHOOTING) {
                        // 发射状态：检测子弹是否发出
                        if (*bullet_fired_
                            || *putter_angle_ - putter_startpoint >= putter_stroke_) {
                            shooted = true;
                        }

                        update_putter_jam_detection();

                        if (shooted) {
                            // 子弹已发出：推杆复位
                            const auto angle_err = putter_startpoint - *putter_angle_;
                            if (angle_err > -0.05) {
                                *putter_control_torque_ = 0.;
                                set_preloading();
                                shooted = false;
                            } else {
                                *putter_control_torque_ =
                                    putter_return_velocity_pid_.update(-80. - *putter_velocity_);
                            }
                        } else {
                            // 子弹未发出：继续推进
                            *putter_control_torque_ =
                                putter_return_velocity_pid_.update(60. - *putter_velocity_);
                        }
                    }

                } else {
                    // 摩擦轮未就绪：停止供弹轮
                    *bullet_feeder_control_torque_ = 0.;
                }

                // 非发射状态：推杆给少许力保持位置
                if (shoot_stage_ != ShootStage::SHOOTING)
                    *putter_control_torque_ = -0.02;
            }
        } else {
            // 推杆未初始化：执行复位操作
            *putter_control_torque_ = putter_return_velocity_pid_.update(-80. - *putter_velocity_);
            update_putter_jam_detection();
        }

        // 保存当前状态用于下次比较
        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_mouse_ = mouse;
        last_keyboard_ = keyboard;
    }

private:
    void reset_all_controls() {
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
        last_mouse_ = rmcs_msgs::Mouse::zero();
        last_keyboard_ = rmcs_msgs::Keyboard::zero();

        overdrive_mode_ = false;

        bullet_feeder_control_angle_ = nan_;
        bullet_feeder_angle_pid_.output_max = inf_;
        bullet_feeder_velocity_pid_.reset();
        bullet_feeder_angle_pid_.reset();
        *bullet_feeder_control_torque_ = nan_;

        // shoot_stage_ = ShootStage::PRELOADING;

        putter_initialized = false;
        putter_startpoint = nan_;
        putter_return_velocity_pid_.reset();
        putter_velocity_pid_.reset();
        putter_return_angle_pid.reset();
        *putter_control_torque_ = nan_;

        last_preload_flag_ = false;
        last_photoelectric_sensor_status_ = false;

        bullet_feeder_faulty_count_ = 0;
        bullet_feeder_cool_down_ = 0;
    }

    void set_resetting() {
        shoot_stage_ = ShootStage::RESETTING;
        bullet_feeder_reset();
    }

    void set_preloading() {
        RCLCPP_INFO(get_logger(), "PRELOADING");
        shoot_stage_ = ShootStage::PRELOADING;
        // 盲拨方案：直接增加目标角度
        if (!std::isnan(bullet_feeder_control_angle_)) {
            bullet_feeder_control_angle_ += bullet_feeder_angle_per_bullet_;
        }
        last_preload_flag_ = true;
    }

    void set_preloaded() {
        RCLCPP_INFO(get_logger(), "PRELOADED");
        shoot_stage_ = ShootStage::PRELOADED;
        last_preload_flag_ = false;
    }

    void set_shooting() {
        RCLCPP_INFO(get_logger(), "SHOOTING");
        shoot_stage_ = ShootStage::SHOOTING;
    }

    void set_updating() { shoot_stage_ = ShootStage::UPDATING; }

    void wait_bullet_ready() {
        if (bullet_ready_count_ >= 1000) {
            bullet_ready_count_ = 0;
            set_preloading();
        } else {
            bullet_ready_count_++;
        }
    }

    void update_jam_detection() {
        // RCLCPP_INFO(get_logger(), "%.2f --", *bullet_feeder_control_torque_);
        if (*bullet_feeder_control_torque_ < 300.0 || std::isnan(*bullet_feeder_control_torque_)) {
            bullet_feeder_faulty_count_ = 0;
            return;
        }

        // 扭矩持续过大时累计故障计数
        if (bullet_feeder_faulty_count_ < 1000)
            bullet_feeder_faulty_count_++;
        else {
            bullet_feeder_faulty_count_ = 0;
            RCLCPP_WARN(get_logger(), "Jam Detected! Reversing 60 degrees...");
            enter_jam_protection();
        }
    }

    void update_putter_jam_detection() {
        if ((*putter_control_torque_ > -0.03 && shoot_stage_ == ShootStage::PRELOADING)
            || (*putter_control_torque_ < 0.05 && shoot_stage_ == ShootStage::SHOOTING)
            || std::isnan(*putter_control_torque_)) {
            putter_faulty_count_ = 0;
            return;
        }

        // 扭矩异常时累计故障计数
        if (putter_faulty_count_ < 500)
            ++putter_faulty_count_;
        else {
            putter_faulty_count_ = 0;
            if (shoot_stage_ != ShootStage::SHOOTING) {
                // 非发射状态下检测到堵转：推杆已到位，设置为已初始化
                putter_initialized = true;
                putter_startpoint = *putter_angle_;
            } else {
                // 发射状态下检测到堵转：认为子弹已发出
                shooted = true;
            }
        }
    }

    void enter_jam_protection() {
        // 设置目标角度为当前角度后退 60 度
        bullet_feeder_control_angle_ = *bullet_feeder_angle_ - bullet_feeder_angle_per_bullet_;
        bullet_feeder_cool_down_ = 1000;
        bullet_feeder_angle_pid_.reset();
        bullet_feeder_velocity_pid_.reset();
        RCLCPP_INFO(get_logger(), "Jammed!");
    }

    void bullet_feeder_reset() {
        double pi = std::numbers::pi;
        double reference_angle = -2 * pi;
        while (std::abs(reference_angle - *bullet_feeder_angle_) > pi / 6) {
            reference_angle += (pi / 3);
        }
        RCLCPP_INFO(get_logger(), "%f, %f", reference_angle, *bullet_feeder_angle_);
        bullet_feeder_control_angle_ = reference_angle;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN(); ///< 非数值常量
    static constexpr double inf_ = std::numeric_limits<double>::infinity();  ///< 无穷大常量

    static constexpr double low_latency_velocity_ = 5.0;                     ///<
    // 低延迟预装弹速度

    static constexpr double putter_stroke_ = 11.5; ///< 推杆行程长度

    static constexpr double max_bullet_feeder_control_torque_ = 0.1;
    static constexpr double bullet_feeder_angle_per_bullet_ = 2 * std::numbers::pi / 6;

    InputInterface<bool> photoelectric_sensor_status_;
    bool last_photoelectric_sensor_status_;
    InputInterface<bool> bullet_fired_;
    bool shooted{false};
    bool shoot_first{true};

    InputInterface<bool> friction_ready_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Mouse last_mouse_ = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    bool overdrive_mode_ = false;

    InputInterface<double> bullet_feeder_angle_;
    InputInterface<double> bullet_feeder_velocity_;

    InputInterface<int64_t> control_bullet_allowance_limited_by_heat_;

    bool last_preload_flag_ = false;

    bool putter_initialized = false;
    int putter_faulty_count_ = 0;
    double putter_startpoint = nan_;
    pid::PidCalculator putter_return_velocity_pid_;
    InputInterface<double> putter_velocity_;

    pid::PidCalculator putter_velocity_pid_;

    enum class ShootStage { RESETTING, PRELOADING, PRELOADED, SHOOTING, UPDATING };
    ShootStage shoot_stage_ = ShootStage::PRELOADING;
    double bullet_feeder_control_angle_ = nan_;

    pid::PidCalculator bullet_feeder_velocity_pid_;
    pid::PidCalculator bullet_feeder_angle_pid_;
    OutputInterface<double> bullet_feeder_control_torque_;

    InputInterface<double> putter_angle_;
    pid::PidCalculator putter_return_angle_pid;
    OutputInterface<double> putter_control_torque_;

    int bullet_feeder_faulty_count_ = 0;
    int bullet_feeder_cool_down_ = 0;
    int bullet_ready_count_ = 0;

    OutputInterface<rmcs_msgs::ShootMode> shoot_mode_;
    OutputInterface<rmcs_msgs::ShootCondiction> shoot_condiction_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::PutterController, rmcs_executor::Component)