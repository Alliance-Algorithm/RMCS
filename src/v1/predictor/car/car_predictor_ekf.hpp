#pragma once

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/armor_id.hpp"
#include "util/ekf.hpp"
#include "util/math.hpp"
#include <vector>

namespace world_exe::v1::predictor {

/**
 * @class CarPredictEkf
 * @brief 车辆预测EKF核心类
 *
 * 状态向量维度: 11维 [x, vx, y, vy, z, z1, z2, r1, r2, θ, ω]，其中 θ为最近装甲板与车辆中心连线与选定系的x轴的夹角
 * 观测向量维度: 4维 [θ, yaw, pitch, distance]
 *
 * 这个是整个预测的核心，无论是逻辑的维护还是预测的模型都放在这里了，然后其他的主要是为了接口实现
 */
class CarPredictEkf : public world_exe::util::Ekf<11, 4, CarPredictEkf> {
public:
    CarPredictEkf() {
        X_k << 2.0, 0.0, 0.0, 0.0, -0.3, -0.3, -0.3, 0.2, 0.2, std::numbers::pi, 0.0;

        P_k.diagonal() << 1., 1., 1., 1., 1., 0., 0., 1., 1., 1., 1.;
    }

    /**
     * @brief 设置第二装甲板标志
     * 这里这个函数需要在同时识别到两个装甲板时调用，在update传入第一块装甲板的信息后，update传入第二块装甲板的信息前，这样做的目的是因为这个模型内维护有最近装甲板的角度，但又需要在识别到两块装甲板时能充分利用两块装甲板来更新状态，所以说需要一个标志位来动态地改变模型的一些参数，主要是

     */
    inline void set_second_armor() { second_armor_flag = true; }

    /**
     * @brief 获取预测的装甲板位置，这里的预测尚未加入速度擦差分得加速度
     * @param id 装甲板类型
     * @param sec 预测时间（秒）
     * @return 四个装甲板的预测位置和姿态
     */
    inline std::vector<data::ArmorGimbalControlSpacing> get_predict_output_armor(
        const enumeration::ArmorIdFlag& id, const double& sec) const {
        std::vector<data::ArmorGimbalControlSpacing> target_armors;
        const auto model_output = X_k;
        const auto vx           = model_output(1); // x方向速度
        const auto vy           = model_output(3); // y方向速度

        // 预测车辆中心位置
        const double car_x = model_output(0) + vx * sec;
        const double car_y = model_output(2) + vy * sec;

        const double car_z = model_output(4);                          // 车辆高度
        const double z1    = model_output(5);                          // 装甲板1高度
        const double z2    = model_output(6);                          // 装甲板2高度
        const double r1    = model_output(7);                          // 装甲板1到中心距离
        const double r2    = model_output(8);                          // 装甲板2到中心距离
        double model_yaw   = model_output(9) + model_output(10) * sec; // 预测yaw角

        // 这里也可以通过z和z1、z2把pitch算出来
        const double pitch1 = 15. / 180. * std::numbers::pi;
        const double pitch2 = 15. / 180. * std::numbers::pi;

        // 根据side_flag_生成四个装甲板的位置
        if (side_flag_) {
            const auto armor1 = data::ArmorGimbalControlSpacing { id,
                { car_x - r1 * std::cos(model_yaw), car_y - r1 * std::sin(model_yaw), z1 },
                util::math::euler_to_quaternion(model_yaw, pitch1, 0.0) };
            target_armors.emplace_back(armor1);

            model_yaw += std::numbers::pi / 2.;
            const auto armor2 = data::ArmorGimbalControlSpacing { id,
                { car_x - r2 * std::cos(model_yaw), car_y - r2 * std::sin(model_yaw), z2 },
                util::math::euler_to_quaternion(model_yaw, pitch2, 0.0) };
            target_armors.emplace_back(armor2);

            model_yaw += std::numbers::pi / 2.;
            const auto armor3 = data::ArmorGimbalControlSpacing { id,
                { car_x - r1 * std::cos(model_yaw), car_y - r1 * std::sin(model_yaw), z1 },
                util::math::euler_to_quaternion(model_yaw, pitch1, 0.0) };
            target_armors.emplace_back(armor3);

            model_yaw += std::numbers::pi / 2.;
            const auto armor4 = data::ArmorGimbalControlSpacing { id,
                { car_x - r2 * std::cos(model_yaw), car_y - r2 * std::sin(model_yaw), z2 },
                util::math::euler_to_quaternion(model_yaw, pitch2, 0.0) };
            target_armors.emplace_back(armor4);
        } else {
            const auto armor1 = data::ArmorGimbalControlSpacing { id,
                { car_x - r2 * std::cos(model_yaw), car_y - r2 * std::sin(model_yaw), z2 },
                util::math::euler_to_quaternion(model_yaw, pitch2, 0.0) };
            target_armors.emplace_back(armor1);

            model_yaw += std::numbers::pi / 2.;
            const auto armor2 = data::ArmorGimbalControlSpacing { id,
                { car_x - r1 * std::cos(model_yaw), car_y - r1 * std::sin(model_yaw), z1 },
                util::math::euler_to_quaternion(model_yaw, pitch1, 0.0) };
            target_armors.emplace_back(armor2);

            model_yaw += std::numbers::pi / 2.;
            const auto armor3 = data::ArmorGimbalControlSpacing { id,
                { car_x - r2 * std::cos(model_yaw), car_y - r2 * std::sin(model_yaw), z2 },
                util::math::euler_to_quaternion(model_yaw, pitch2, 0.0) };
            target_armors.emplace_back(armor3);

            model_yaw += std::numbers::pi / 2.;
            const auto armor4 = data::ArmorGimbalControlSpacing { id,
                { car_x - r1 * std::cos(model_yaw), car_y - r1 * std::sin(model_yaw), z1 },
                util::math::euler_to_quaternion(model_yaw, pitch1, 0.0) };
            target_armors.emplace_back(armor4);
        }

        return target_armors;
    }

private:
    friend class world_exe::util::Ekf<11, 4, CarPredictEkf>;

    /**
     * @brief 观测预处理函数
     * 状态: x分别代表[ x, v_x, y, v_y, z, z_1, z_2, r1, r2, θ, ω ]
     * 观测: z分别代表[ θ, yaw, pitch, distance ]
     */
    inline ZVec process_z(const ZVec& z_k) {
        ZVec processed_z { z_k };
        if (z_k(0) < 0.) processed_z(0) += std::numbers::pi * 2.;

        // 检测装甲板切换，switch_angle_difference_根据实际情况可能需要调整
        double offset { processed_z(0) - last_theta_ };
        if (std::abs(offset) >= switch_angle_difference_) {
            side_flag_ = !side_flag_; // 切换装甲板面
        } else offset = 0.;

        last_theta_ = processed_z(0);

        // 根据当前装甲板面更新高度
        if (side_flag_) X_k(5) = z_k(3) * std::cos(z_k(1)) * -std::sin(z_k(2));
        else X_k(6) = z_k(3) * std::cos(z_k(1)) * -std::sin(z_k(2));

        if (X_k(9) < 0.) X_k(9) += std::numbers::pi * 2.;

        X_k(9) += offset;
        return processed_z;
    };

    /** @brief 状态向量归一化 */
    inline XVec normalize_x(const XVec& x_k) {
        normalized_x    = x_k;
        auto camera_yaw = x_k(9);
        // 角度归一化到 [0, 2π)
        while (camera_yaw < 0.)
            camera_yaw += 2 * std::numbers::pi;
        while (camera_yaw >= 2 * std::numbers::pi)
            camera_yaw -= 2 * std::numbers::pi;

        normalized_x(9) = camera_yaw;
        return normalized_x;
    };

    /** @brief 状态转移函数 f(x,u,w,dt) */
    inline XVec f(const XVec& X_k, const UVec&, const WVec&, const double& dt) {
        const double x     = X_k(0) + X_k(1) * dt;   // 位置更新
        const double y     = X_k(2) + X_k(3) * dt;   // 位置更新
        const double z     = (X_k(5) + X_k(6)) / 2.; // 平均高度
        const double theta = X_k(9) + X_k(10) * dt;  // 角度更新

        f_ << x, X_k(1), y, X_k(3), z, X_k(5), X_k(6), X_k(7), X_k(8), theta, X_k(10);

        return f_;
    };

    /** @brief 观测函数 h(x,v) */
    inline ZVec h(const XVec& x_k_n, const VVec&) {
        double r, z;
        // 根据当前装甲板面选择参数
        if (side_flag_) {
            r = x_k_n(7); // r1
            z = x_k_n(5); // z1
        } else {
            r = x_k_n(8); // r2
            z = x_k_n(6); // z2
        }

        const double theta    = x_k_n(9);
        const double armor_x  = x_k_n(0) + r * std::cos(theta); // 装甲板x坐标
        const double armor_y  = x_k_n(2) + r * std::sin(theta); // 装甲板y坐标
        const double yaw      = std::atan(armor_y / armor_x);   // yaw角
        const double pitch    = -std::atan(z / armor_x);        // pitch角
        const double distance = std::sqrt(armor_x * armor_x + armor_y * armor_y + z * z); // 距离
        h_ << x_k_n(9), yaw, pitch, distance;
        return h_;
    };

    /** @brief 状态转移雅可比矩阵 A */
    inline AMat A(const XVec&, const XVec&, const XVec&, const double& dt) {
        // clang-format off
            A_ << 1., dt, 0., 0., 0.,  0.,   0., 0., 0., 0., 0.,
                  0., 1., 0., 0., 0.,  0.,   0., 0., 0., 0., 0.,
                  0., 0., 1., dt, 0.,  0.,   0., 0., 0., 0., 0.,
                  0., 0., 0., 1., 0.,  0.,   0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0.5,  0.5, 0., 0., 0., 0., 
                  0., 0., 0., 0., 0.,  1.,   0., 0., 0., 0., 0., 
                  0., 0., 0., 0., 0.,  0.,   1., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0.,  0.,   0., 1., 0., 0., 0.,
                  0., 0., 0., 0., 0.,  0.,   0., 0., 1., 0., 0., 
                  0., 0., 0., 0., 0.,  0.,   0., 0., 0., 1., dt, 
                  0., 0., 0., 0., 0.,  0.,   0., 0., 0., 0., 1.;
        // clang-format on
        return A_;
    };

    /** @brief 过程噪声雅可比矩阵 W */
    inline WMat W(const XVec&, const XVec&, const XVec&) const { return W_; };

    /** @brief 观测雅可比矩阵 H */
    inline HMat H(const XVec& x_k_n, const VVec&) {
        const double theta = x_k_n(9);

        if (side_flag_) {
            const double r         = x_k_n(7);
            const double z         = x_k_n(5);
            const double armor_x   = x_k_n(0) + r * std::cos(theta);
            const double armor_y   = x_k_n(2) + r * std::sin(theta);
            const double armor_x_2 = armor_x * armor_x;
            const double armor_y_2 = armor_y * armor_y;

            const double d_x_2_base = 1.0 / (1.0 + armor_y_2 / armor_x_2);
            const double d_x_2_0    = -d_x_2_base * armor_y / armor_x_2;
            const double d_x_2_2    = d_x_2_base / armor_x;
            const double d_x_2_7 =
                d_x_2_base * (-armor_y / armor_x_2 * std::cos(theta) + std::sin(theta) / armor_x);
            const double d_x_2_9 = d_x_2_base * r
                * (std::sin(theta) * armor_y / armor_x_2 + std::cos(theta) / armor_x);

            const double d_x_3_base = 1.0 / armor_x_2 / (1.0 + z * z / armor_x_2);
            const double d_x_3_0    = d_x_3_base * z;
            const double d_x_3_5    = -d_x_3_base * armor_x;
            const double d_x_3_7    = d_x_3_0 * std::cos(theta);
            const double d_x_3_9    = -d_x_3_0 * r * std::sin(theta);

            const double d_x_4_base = 1.0 / std::sqrt(z * z + armor_x_2 + armor_y_2);
            const double d_x_4_0    = d_x_4_base * armor_x;
            const double d_x_4_2    = d_x_4_base * armor_y;
            const double d_x_4_5    = d_x_4_base * z;
            const double d_x_4_7 =
                d_x_4_base * (std::sin(theta) * armor_y + std::cos(theta) * armor_x);
            const double d_x_4_9 =
                d_x_4_base * r * (std::cos(theta) * armor_y - std::sin(theta) * armor_x);

            // clang-format off
            H_ <<      0., 0.,      0., 0., 0.,      0., 0.,      0., 0.,      1., 0.,
                  d_x_2_0, 0., d_x_2_2, 0., 0.,      0., 0., d_x_2_7, 0., d_x_2_9, 0., 
                  d_x_3_0, 0.,      0., 0., 0., d_x_3_5, 0., d_x_3_7, 0., d_x_3_9, 0.,
                  d_x_4_0, 0., d_x_4_2, 0., 0., d_x_4_5, 0., d_x_4_7, 0., d_x_4_9, 0.;

            // clang-format on
        } else {
            const double r         = x_k_n(8);
            const double z         = x_k_n(6);
            const double armor_x   = x_k_n(0) + r * std::cos(theta);
            const double armor_y   = x_k_n(2) + r * std::sin(theta);
            const double armor_x_2 = armor_x * armor_x;
            const double armor_y_2 = armor_y * armor_y;

            const double d_x_2_base = 1.0 / (1.0 + armor_y_2 / armor_x_2);
            const double d_x_2_0    = -d_x_2_base * armor_y / armor_x_2;
            const double d_x_2_2    = d_x_2_base / armor_x;
            const double d_x_2_8 =
                d_x_2_base * (-armor_y / armor_x_2 * std::cos(theta) + std::sin(theta) / armor_x);
            const double d_x_2_9 = d_x_2_base * r
                * (std::sin(theta) * armor_y / armor_x_2 + std::cos(theta) / armor_x);

            const double d_x_3_base = 1.0 / armor_x_2 / (1.0 + z * z / armor_x_2);
            const double d_x_3_0    = d_x_3_base * z;
            const double d_x_3_6    = -d_x_3_base * armor_x;
            const double d_x_3_8    = d_x_3_0 * std::cos(theta);
            const double d_x_3_9    = -d_x_3_0 * r * std::sin(theta);

            const double d_x_4_base = 1.0 / std::sqrt(z * z + armor_x_2 + armor_y_2);
            const double d_x_4_0    = d_x_4_base * armor_x;
            const double d_x_4_2    = d_x_4_base * armor_y;
            const double d_x_4_6    = d_x_4_base * z;
            const double d_x_4_8 =
                d_x_4_base * (std::sin(theta) * armor_y + std::cos(theta) * armor_x);
            const double d_x_4_9 =
                d_x_4_base * r * (std::cos(theta) * armor_y - std::sin(theta) * armor_x);

            // clang-format off
            H_ <<      0., 0.,      0., 0., 0., 0.,      0., 0.,      0.,      1., 0.,
                  d_x_2_0, 0., d_x_2_2, 0., 0., 0.,      0., 0., d_x_2_8, d_x_2_9, 0., 
                  d_x_3_0, 0.,      0., 0., 0., 0., d_x_3_6, 0., d_x_3_8, d_x_3_9, 0.,
                  d_x_4_0, 0., d_x_4_2, 0., 0., 0., d_x_4_6, 0., d_x_4_8, d_x_4_9, 0.;
            // clang-format on
        }
        return H_;
    };

    /** @brief 观测噪声雅可比矩阵 V */
    inline VMat V(const XVec&, const VVec&) const { return V_; };

    /** @brief 过程噪声协方差矩阵 Q */
    inline QMat Q(const double& dt) {
        double x;
        x            = 100;
        double t     = dt;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        if (side_flag_) {
            // clang-format off
            Q_ <<  q_x_x,  q_x_vx,     0.,      0., 0., 0., 0.,   0., 0.,  0.,  0.,
                  q_x_vx, q_vx_vx,     0.,      0., 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,  q_x_x,  q_x_vx, 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0., q_x_vx, q_vx_vx, 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 1., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   1e-10, 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   0., 0., 1.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   0., 0.,  0., 3.0;
            // clang-format on
        } else {
            // clang-format off
            Q_ <<  q_x_x,  q_x_vx,     0.,      0., 0., 0., 0., 0.,   0.,  0.,  0.,
                  q_x_vx, q_vx_vx,     0.,      0., 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,  q_x_x,  q_x_vx, 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0., q_x_vx, q_vx_vx, 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,     0.,      0., 1., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   1e-10,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   0., 1.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   0.,  0., 3.0;
            // clang-format on
        }
        return Q_;
    };

    inline RMat R(const ZVec& z) {
        const double yaw_center_difference_ = std::abs(std::sin(z(0))) + 1.;
        if (second_armor_flag) {
            second_armor_flag = false;
            // 同时识别到两块装甲板时，第二装甲板时忽略θ观测，因为第一个输入装甲板是比较近的且较准
            R_.diagonal() << 9999999999999., yaw_center_difference_ * r_theta_yaw_,
                yaw_center_difference_ * r_theta_pitch_, yaw_center_difference_ * r_theta_distance_;
        } else {
            R_.diagonal() << yaw_center_difference_ * r_theta_theta_,
                yaw_center_difference_ * r_theta_yaw_, yaw_center_difference_ * r_theta_pitch_,
                yaw_center_difference_ * r_theta_distance_;
        }
        return R_;
    };

    XVec f_ {};                   ///< 状态转移函数输出缓存
    ZVec h_ {};                   ///< 观测函数输出缓存
    AMat A_ {};                   ///< 状态转移雅可比矩阵
    WMat W_ { WMat::Identity() }; ///< 过程噪声雅可比矩阵
    HMat H_ {};                   ///< 观测雅可比矩阵
    VMat V_ { VMat::Identity() }; ///< 观测噪声雅可比矩阵
    QMat Q_ {};                   ///< 过程噪声协方差矩阵
    RMat R_ {};                   ///< 观测噪声协方差矩阵
    XVec normalized_x {};         ///< 归一化状态向量缓存

    bool debug_ { true };                    ///< 调试标志
    double last_theta_ { std::numbers::pi }; ///< 上次观测角度
    static constexpr double switch_angle_difference_ =
        std::numbers::pi * 70. / 180.;                ///< 装甲板切换角度阈值
    static constexpr double r_theta_theta_    = 0.1;  ///< θ观测噪声
    static constexpr double r_theta_yaw_      = 1e-3; ///< yaw观测噪声
    static constexpr double r_theta_pitch_    = 1e-3; ///< pitch观测噪声
    static constexpr double r_theta_distance_ = 1.;   ///< 距离观测噪声

    bool side_flag_ { true };         ///< 装甲板面标志位
    bool second_armor_flag { false }; ///< 第二装甲板标志位
};

} // namespace world_exe::v1::predictor