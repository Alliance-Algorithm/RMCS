#include <Eigen/Dense>

#include <cmath>
#include <cstdlib>

using namespace Eigen;
using AffineTransform = Affine3d; // 仿射变换矩阵 (4x4)
using Vector3D        = Vector3d;
using QuaternionType  = Quaterniond;

namespace world_exe::tests::mock {

class Camera2GimbalTransformer {
private:
    // --- 状态变量 ---
    // 当前的仿射变换 (包含位置和姿态)
    AffineTransform T_current;

    // 当前时间
    double current_time;

    // --- 输入参数 ---
    // 1. 水平平移速度 (在世界坐标系下)
    const Vector3D V_horizontal;
    // 2. 绕竖直轴 (Z轴) 的角速度 (在世界坐标系下)
    const double Omega_Yaw;
    // 3. 绕 Pitch 轴的角速度 (在世界坐标系下)
    double Omega_Pitch;
    // 4. 最大 Pitch 角度 (用于约束)
    const double Max_Pitch_Angle;

    // 用于追踪当前 Pitch 角度，以实现角度约束 (假设Pitch轴是Y轴)
    double current_pitch_angle;

public:
    /**
     * @brief 构造函数
     * @param v_horiz 水平速度向量 (例如: 1.0, 0.0, 0.0)
     * @param omega_yaw 绕 Z 轴角速度 (rad/s)
     * @param omega_pitch 绕 Pitch 轴角速度 (rad/s)
     * @param max_pitch_angle 绕 Pitch 轴的最大角度限制 (rad)
     */
    Camera2GimbalTransformer(
        const Vector3D& v_horiz, double omega_yaw, double omega_pitch, double max_pitch_angle)
        : V_horizontal(v_horiz)
        , Omega_Yaw(omega_yaw)
        , Omega_Pitch(omega_pitch)
        , Max_Pitch_Angle(std::abs(max_pitch_angle))
        , // 确保最大角度是正值
        T_current(AffineTransform::Identity())
        , current_time(0.0)
        , current_pitch_angle(0.0) { }

    /**
     * @brief 按照给定的时间步长进行一次状态积分，并返回更新后的 Affine 变换
     * @param dt 时间步长 (秒)
     * @return 更新后的 Affine 变换矩阵
     */
    AffineTransform updateAndGetTransform(double dt) {
        Vector3D delta_t = V_horizontal * dt;

        double delta_yaw = Omega_Yaw * dt;
        AngleAxisd R_yaw(delta_yaw, Vector3D::UnitZ());

        double delta_pitch      = Omega_Pitch * dt;
        double next_pitch_angle = current_pitch_angle + delta_pitch;
        if (next_pitch_angle > Max_Pitch_Angle || next_pitch_angle < -Max_Pitch_Angle) {

            Omega_Pitch = -Omega_Pitch;
            delta_pitch = Omega_Pitch * dt;
        }

        current_pitch_angle = current_pitch_angle + delta_pitch;

        AngleAxisd R_pitch(delta_pitch, Vector3D::UnitY());

        // 2c. 组合旋转增量：R_total = R_yaw * R_pitch
        // 假设 Pitch 和 Yaw 是在世界坐标系下连续作用的 (即增量也是在世界坐标系下)
        // 结果是一个 AngleAxisd 或 Quaternion，需要转换为 Affine
        AffineTransform T_rot_increment = AffineTransform(R_yaw * R_pitch);

        // --- 3. 组合总增量 T_delta ---

        // T_delta = T_平移 * T_旋转
        AffineTransform T_delta = Translation3d(delta_t) * T_rot_increment;

        // --- 4. 状态更新 ---

        // T_new = T_delta * T_old (因为 T_delta 是相对于世界坐标系W的增量)
        T_current = T_delta * T_current;
        current_time += dt;

        return T_current;
    }
};
}
