#pragma once

#include "eigen3/Eigen/Eigen"

namespace world_exe::util {
template <int xn, int zn, typename IEkf> class Ekf {
public:
    typedef Eigen::Matrix<double, xn, 1> XVec;
    typedef XVec UVec;
    typedef XVec WVec;

    typedef Eigen::Matrix<double, zn, 1> ZVec;
    typedef ZVec VVec;

    typedef Eigen::Matrix<double, xn, xn> PMat;
    typedef Eigen::Matrix<double, zn, zn> RMat;
    typedef Eigen::Matrix<double, xn, xn> AMat;
    typedef Eigen::Matrix<double, xn, xn> WMat;
    typedef Eigen::Matrix<double, zn, zn> VMat;
    typedef Eigen::Matrix<double, xn, xn> QMat;
    typedef Eigen::Matrix<double, zn, xn> HMat;
    typedef Eigen::Matrix<double, xn, zn> KMat;

    [[nodiscard]] inline XVec OutPut() const { return X_k; }

    inline void Update(const ZVec& z_k, const UVec& u_k, const double& dt) {
        dt_ = dt;
        P_k_n.setZero();
        S_k.setZero();
        y_k.setZero();
        K_t.setZero();
        tmpK.setZero();

        auto derived = static_cast<IEkf*>(this);

        const auto processed_z = derived->process_z(z_k);

        auto x_k_n = derived->f(X_k, u_k, w_zero, dt);
        auto A_k   = derived->A(X_k, u_k, w_zero, dt);
        auto W_k   = derived->W(X_k, u_k, w_zero);
        auto H_k   = derived->H(x_k_n, v_zero);
        auto V_k   = derived->V(x_k_n, v_zero);

        P_k_n = A_k * P_k * A_k.transpose() + W_k * derived->Q(dt) * W_k.transpose();

        y_k  = processed_z - derived->h(x_k_n, v_zero);
        S_k  = H_k * P_k_n * H_k.transpose() + V_k * derived->R(processed_z) * V_k.transpose();
        K_t  = P_k_n * H_k.transpose() * S_k.inverse();
        X_k  = x_k_n + K_t * y_k;
        X_k  = derived->normalize_x(X_k);
        tmpK = Eye_K - K_t * H_k;
        P_k  = tmpK * P_k_n;
    }

    // 派生类需要实现以下函数
    // [[nodiscard]] virtual ZVec process_z(const ZVec& z_k) { return z_k; }
    // [[nodiscard]] virtual XVec normalize_x(const XVec& X_k) { return X_k; }
    // [[nodiscard]] virtual XVec f(const XVec&, const UVec&, const WVec&, const double&) = 0;
    // [[nodiscard]] virtual ZVec h(const XVec&, const VVec&)                             = 0;
    //
    // [[nodiscard]] virtual AMat A(const XVec&, const UVec&, const WVec&, const double&) = 0;
    //
    // [[nodiscard]] virtual WMat W(const XVec&, const UVec&, const WVec&) = 0;
    //
    // [[nodiscard]] virtual HMat H(const XVec&, const VVec&) = 0;
    //
    // [[nodiscard]] virtual VMat V(const XVec&, const VVec&) = 0;
    //
    // [[nodiscard]] virtual QMat Q(const double& t) = 0;
    // [[nodiscard]] virtual RMat R(const ZVec& z)   = 0;

protected:
    XVec X_k { XVec::Zero() };
    PMat P_k { PMat::Identity() };

    double dt_;

private:
    PMat P_k_n {};
    RMat S_k {};
    ZVec y_k {};
    KMat K_t {};
    PMat tmpK {};

    static inline const WVec w_zero = Eigen::Matrix<double, xn, 1>::Zero();
    static inline const VVec v_zero = Eigen::Matrix<double, zn, 1>::Zero();
    static inline const PMat Eye_K  = Eigen::Matrix<double, xn, xn>::Identity();
};
}
