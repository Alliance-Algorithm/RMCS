#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::shooting {

class BulletFeederController42mm
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    BulletFeederController42mm()
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
        register_input("/gimbal/bullet_fired", bullet_fired_);

        // 增益改为从参数读取，默认值保持上游的原值（行为不变）。
        // 上游这组数值是按他们那颗电机调的，换成别的电机（比如 LK MG5010E）
        // 后力矩常数不同，必须能调，否则只能改源码重编译。
        bullet_feeder_velocity_pid_.kp = param_d("velocity_kp", 50.0);
        bullet_feeder_velocity_pid_.ki = param_d("velocity_ki", 10.0);
        bullet_feeder_velocity_pid_.kd = param_d("velocity_kd", 0.0);
        bullet_feeder_velocity_pid_.integral_max =
            param_d("velocity_integral_max", 60.0);
        bullet_feeder_velocity_pid_.integral_min =
            param_d("velocity_integral_min", 0.0);

        bullet_feeder_angle_pid_.kp = param_d("angle_kp", 60.0);
        bullet_feeder_angle_pid_.ki = param_d("angle_ki", 0.0);
        bullet_feeder_angle_pid_.kd = param_d("angle_kd", 2.0);

        // 力矩限幅：速度环的输出直接就是力矩指令，必须给它一个上限。
        // 不加限幅时，只要增益偏大或反馈异常，输出就会飙到几十 N·m ——
        // 这是最危险的一处，硬件可能直接被撞坏。
        const double torque_limit = param_d("torque_limit", 2.0);
        bullet_feeder_velocity_pid_.output_max = torque_limit;
        bullet_feeder_velocity_pid_.output_min = -torque_limit;

        // 开环推力模式：直接按"角度误差的符号"给固定力矩，完全不使用速度反馈。
        // 原因：这颗 LK 电机经减速后，低速时速度反馈噪声可达 ±10 rad/s，
        // 而真实运动只有 0.3 rad/s —— 速度环只会把噪声放大成满力矩抖动。
        // 角度信号是干净的，所以改用"位置 + 固定力矩"，本质是位置式 bang-bang，
        // 力量最大、不受噪声影响，代价是到位时有轻微超调（可接受）。
        open_loop_push_ = param_b("open_loop_push", false);
        push_torque_ = param_d("push_torque", 6.0);
        push_deadband_ = param_d("push_deadband", 0.05);
        // 角度比例增益（N·m per rad）。误差大时输出被 push_torque 限住（全力推弹），
        // 接近目标时自动减速，冲过头时输出负值但被 -pull_torque 限住（轻轻拉回）。
        angle_torque_kp_ = param_d("angle_torque_kp", 10.0);
        // 阻尼系数（N·m per rad/s）。只用 P 控制时，弹盘带着动量冲过目标会来回摆动，
        // D 项按"当前速度"反向出力，等于给系统加阻尼，能显著压住过冲。
        // 速度反馈噪声大，所以只用低通滤波之后的值。
        angle_torque_kd_ = param_d("angle_torque_kd", 0.25);
        // 最小起步力矩：齿轮箱有静摩擦，P 项在小误差时输出太小，
        // 会出现"有力矩但推不动"的死点（实测误差 0.13 rad 卡住不动）。
        // 只要误差超出死区，就至少给这么多力矩，保证能动起来。
        min_torque_ = param_d("min_torque", 2.0);
        angle_velocity_filter_alpha_ = param_d("angle_velocity_filter_alpha", 0.1);
        // 几何常量参数化：这两个值原本是按上游机构的尺寸写死的。
        //   compressed_zero_point : 压缩零点（rad）
        //   angle_per_bullet      : 每发弹盘转过的角度（rad）
        // 注意：这里的角度单位与 /gimbal/bullet_feeder/angle 一致。
        // 若电机到弹盘之间有减速比 R，则"弹盘转 60°"对应角度量是 2π/6 × R。
        compressed_zero_point_ = param_d("compressed_zero_point", 2 * std::numbers::pi / 6 * 0.5536);
        angle_per_bullet_ = param_d("angle_per_bullet", 2 * std::numbers::pi / 6);
        // 角度环输出的是"速度设定"。这两个值是各阶段的速度上限（rad/s）：
        //   feed     : 正常拨弹的速度上限
        //   compress : 压缩阶段的速度上限（通常更慢，避免冲击）
        feed_velocity_max_ = param_d("feed_velocity_max", 1.0);
        compress_velocity_max_ = param_d("compress_velocity_max", 0.8);
        // 速度低通滤波系数。LK 电机经 10:1 减速后，低速时编码器速度噪声很大，
        // 不滤波的话速度环等于在拿噪声算力矩，表现为"力矩上不去 + 来回抖"。
        velocity_filter_alpha_ = param_d("velocity_filter_alpha", 0.1);

        register_output(
            "/gimbal/bullet_feeder/control_torque", bullet_feeder_control_torque_, nan_);

        // For compatibility
        register_output("/gimbal/shooter/mode", shoot_mode_, rmcs_msgs::ShootMode::SINGLE);
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
        overdrive_mode_ = keyboard.f;
        if (keyboard.ctrl && !last_keyboard_.r && keyboard.r)
            low_latency_mode_ = !low_latency_mode_;

        if (bullet_feeder_cool_down_ > 0) {
            bullet_feeder_cool_down_--;

            if (bullet_feeder_cool_down_ > 500)
                *bullet_feeder_control_torque_ = bullet_feeder_velocity_pid_.update(
                    -5.0 * angle_per_bullet_ - *bullet_feeder_velocity_);
            else {
                bullet_feeder_velocity_pid_.reset();
                *bullet_feeder_control_torque_ = 0.0;
            }

            bullet_feeder_angle_pid_.reset();

            if (!bullet_feeder_cool_down_)
                RCLCPP_INFO(get_logger(), "Jamming Solved!");
        } else {
            if (!*friction_ready_ || std::isnan(bullet_feeder_control_angle_)) {
                bullet_feeder_control_angle_ = *bullet_feeder_angle_;
                shoot_stage_ = ShootStage::PRELOADED;
                bullet_fed_count_ = static_cast<int>(
                    (*bullet_feeder_angle_ - compressed_zero_point_ - 0.1)
                    / angle_per_bullet_);
            }

            if (*friction_ready_) {
                if (switch_right != Switch::DOWN) {
                    if ((!last_mouse_.left && mouse.left)
                        || (last_switch_left_ == rmcs_msgs::Switch::MIDDLE
                            && switch_left == rmcs_msgs::Switch::DOWN)) {
                        if (*control_bullet_allowance_limited_by_heat_ > 0)
                            set_shooting();
                    }
                }

                double angle_error_abs =
                    std::abs(bullet_feeder_control_angle_ - *bullet_feeder_angle_);
                if (shoot_stage_ == ShootStage::PRELOADING) {
                    if (angle_error_abs < 0.1)
                        set_preloaded();
                }
                if (shoot_stage_ == ShootStage::PRELOADED) {
                    if (low_latency_mode_)
                        set_compressing();
                }
                if (shoot_stage_ == ShootStage::COMPRESSING) {
                    if (angle_error_abs < 0.1)
                        set_compressed();
                }
                if (shoot_stage_ == ShootStage::SHOOTING) {
                    if (angle_error_abs < 0.1)
                        set_preloading();
                }
            }

            // 一阶低通：alpha 越小越平滑。只用于控制，日志里仍然打印原始值，
            // 这样能从日志直接看出"真实运动"和"噪声"的差别。
            filtered_velocity_ +=
                velocity_filter_alpha_ * (*bullet_feeder_velocity_ - filtered_velocity_);

            // —— 用编码器角度差分估计弹盘角速度（两个分支都要用，必须放在 if/else 之外）——
            // 电机的速度反馈在低速时噪声可达真实值的十倍以上，不能用来做阻尼；
            // 编码器角度分辨率极高且平滑，差分得到的才是可靠速度。
            {
                const auto now = std::chrono::steady_clock::now();
                const double angle_now = *bullet_feeder_angle_;
                if (has_last_angle_for_diff_) {
                    const double dt =
                        std::chrono::duration<double>(now - last_diff_time_).count();
                    // dt 合理才更新，避免卡顿/首帧造成巨大尖峰
                    if (dt > 1e-4 && dt < 0.1) {
                        const double raw = (angle_now - last_angle_for_diff_) / dt;
                        estimated_angle_velocity_ +=
                            angle_velocity_filter_alpha_ * (raw - estimated_angle_velocity_);
                    }
                }
                last_angle_for_diff_ = angle_now;
                last_diff_time_ = now;
                has_last_angle_for_diff_ = true;
            }

            if (open_loop_push_) {
                // 角度 PD 控制（不依赖电机速度反馈）：
                //   误差大   → 输出被 push_torque 限住，全力推弹丸
                //   接近目标 → 力矩随误差减小，自然减速
                //   有速度   → D 项反向刹车，压制过冲
                // 死区必须远小于状态机阈值(0.1)，否则会卡在"误差不够小又不输出"的死区里。
                const double err = bullet_feeder_control_angle_ - *bullet_feeder_angle_;
                // PD：P 拉向目标，D 按速度刹车（抑制过冲和摆动）。
                // 注意阻尼用的是"角度差分估计的速度"，不是电机的速度反馈：
                // 这颗电机低速时速度反馈噪声可达真实值的 10 倍以上，用它做 D 项会
                // 在加速阶段就产生巨大的反向力矩，表现为"推不动 + 抖"。
                // 编码器角度本身干净且分辨率极高，差分后才是可靠的速度来源。
                double torque =
                    angle_torque_kp_ * err - angle_torque_kd_ * estimated_angle_velocity_;
                torque = std::clamp(torque, -push_torque_, push_torque_);
                if (std::abs(err) < push_deadband_) {
                    torque = 0.0;
                } else if (std::abs(torque) < min_torque_) {
                    // 打破静摩擦：误差还在死区外，至少给最小起步力矩
                    torque = std::copysign(min_torque_, err);
                }
                *bullet_feeder_control_torque_ = torque;
            } else {
            double velocity_error = bullet_feeder_angle_pid_.update(
                                            bullet_feeder_control_angle_ - *bullet_feeder_angle_)
                                      - filtered_velocity_;
                *bullet_feeder_control_torque_ =
                    bullet_feeder_velocity_pid_.update(velocity_error);
            }

            update_jam_detection();
        }

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_mouse_ = mouse;
        last_keyboard_ = keyboard;
    }

private:
    // 这个节点用 automatically_declare_parameters_from_overrides(true) 构造，
    // 也就是说 yaml 里出现的参数在构造函数体运行前就已经被声明了。
    // 因此这里不能直接 declare_parameter（会抛 ParameterAlreadyDeclaredException），
    // 必须先判断是否已存在：存在就读出来，不存在才用默认值声明。
    double param_d(const char* name, double fallback) {
        if (has_parameter(name))
            return get_parameter(name).as_double();
        return declare_parameter<double>(name, fallback);
    }

    bool param_b(const char* name, bool fallback) {
        if (has_parameter(name))
            return get_parameter(name).as_bool();
        return declare_parameter<bool>(name, fallback);
    }

    void reset_all_controls() {
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
        last_mouse_ = rmcs_msgs::Mouse::zero();
        last_keyboard_ = rmcs_msgs::Keyboard::zero();

        overdrive_mode_ = low_latency_mode_ = false;

        shoot_stage_ = ShootStage::PRELOADED;
        bullet_fed_count_ = std::numeric_limits<int>::min();

        bullet_feeder_control_angle_ = nan_;
        bullet_feeder_angle_pid_.output_max = inf_;

        bullet_feeder_velocity_pid_.reset();
        bullet_feeder_angle_pid_.reset();
        *bullet_feeder_control_torque_ = nan_;

        bullet_feeder_faulty_count_ = 0;
        bullet_feeder_cool_down_ = 0;
    }

    void set_preloading() {
        RCLCPP_INFO(get_logger(), "PRELOADING");
        bullet_fed_count_++;
        shoot_stage_ = ShootStage::PRELOADING;
        bullet_feeder_control_angle_ = compressed_zero_point_
                                     + (bullet_fed_count_ + 0.5) * angle_per_bullet_;
        bullet_feeder_angle_pid_.output_max = feed_velocity_max_;
    }

    void set_preloaded() {
        RCLCPP_INFO(get_logger(), "PRELOADED");
        shoot_stage_ = ShootStage::PRELOADED;
    }

    void set_compressing() {
        RCLCPP_INFO(get_logger(), "COMPRESSING");
        shoot_stage_ = ShootStage::COMPRESSING;
        bullet_feeder_control_angle_ = compressed_zero_point_
                                     + (bullet_fed_count_ + 1) * angle_per_bullet_;
        bullet_feeder_angle_pid_.output_max = compress_velocity_max_;
    }

    void set_compressed() {
        RCLCPP_INFO(get_logger(), "COMPRESSED");
        shoot_stage_ = ShootStage::COMPRESSED;
    }

    void set_shooting() {
        RCLCPP_INFO(get_logger(), "SHOOTING");
        shoot_stage_ = ShootStage::SHOOTING;
        bullet_feeder_control_angle_ = compressed_zero_point_
                                     + (bullet_fed_count_ + 1.2) * angle_per_bullet_;
        bullet_feeder_angle_pid_.output_max = feed_velocity_max_;
    }

    void update_jam_detection() {
        if (*bullet_feeder_control_torque_ < 300.0) {
            bullet_feeder_faulty_count_ = 0;
            return;
        }

        if (bullet_feeder_faulty_count_ < 1000)
            bullet_feeder_faulty_count_++;
        else {
            bullet_feeder_faulty_count_ = 0;
            enter_jam_protection();
        }
    }

    void enter_jam_protection() {
        bullet_feeder_control_angle_ = nan_;
        bullet_feeder_cool_down_ = 1000;
        bullet_feeder_angle_pid_.reset();
        bullet_feeder_velocity_pid_.reset();
        RCLCPP_INFO(get_logger(), "Jammed!");
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();

    double compressed_zero_point_ = 0.58;
    double angle_per_bullet_ = 2 * std::numbers::pi / 6;

    InputInterface<bool> friction_ready_;
    InputInterface<bool> bullet_fired_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Mouse last_mouse_ = rmcs_msgs::Mouse::zero();
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    bool overdrive_mode_ = false, low_latency_mode_ = false;

    InputInterface<double> bullet_feeder_angle_;
    InputInterface<double> bullet_feeder_velocity_;

    InputInterface<int64_t> control_bullet_allowance_limited_by_heat_;

    enum class ShootStage { PRELOADING, PRELOADED, COMPRESSING, COMPRESSED, SHOOTING };
    ShootStage shoot_stage_ = ShootStage::PRELOADED;
    int bullet_fed_count_ = std::numeric_limits<int>::min();
    double bullet_feeder_control_angle_ = nan_;

    pid::PidCalculator bullet_feeder_velocity_pid_;
    pid::PidCalculator bullet_feeder_angle_pid_;
    OutputInterface<double> bullet_feeder_control_torque_;

    bool open_loop_push_ = false;
    double push_torque_ = 6.0;
    double push_deadband_ = 0.05;
    double angle_torque_kp_ = 10.0;
    double angle_torque_kd_ = 0.25;
    double min_torque_ = 2.0;
    double angle_velocity_filter_alpha_ = 0.1;
    double estimated_angle_velocity_ = 0.0;
    double last_angle_for_diff_ = 0.0;
    bool has_last_angle_for_diff_ = false;
    std::chrono::steady_clock::time_point last_diff_time_{};
    double feed_velocity_max_ = 1.0;
    double compress_velocity_max_ = 0.8;
    double velocity_filter_alpha_ = 0.1;
    double filtered_velocity_ = 0.0;
    int bullet_feeder_faulty_count_ = 0;
    int bullet_feeder_cool_down_ = 0;

    OutputInterface<rmcs_msgs::ShootMode> shoot_mode_;
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::shooting::BulletFeederController42mm, rmcs_executor::Component)
