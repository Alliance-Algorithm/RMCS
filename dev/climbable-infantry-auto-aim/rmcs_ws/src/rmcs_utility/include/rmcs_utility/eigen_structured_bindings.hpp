#pragma once

#include <eigen3/Eigen/Dense>

// https://gist.github.com/mtao/58e6859ac4ac2300ddb9e1050b9ed701

// IWYU pragma: always_keep

namespace Eigen {

template <int D, typename Derived>
inline auto& get(Eigen::DenseCoeffsBase<Derived, WriteAccessors>& p) {
    return p.coeffRef(D);
}

template <int D, typename Derived>
inline auto get(const Eigen::DenseCoeffsBase<Derived, ReadOnlyAccessors>& p) {
    return p.coeff(D);
}

template <typename Derived>
concept StaticSizePlainObjectBase =
    std::derived_from<Derived, Eigen::PlainObjectBase<Derived>>
    && (Derived::RowsAtCompileTime > 0 && Derived::ColsAtCompileTime > 0);

template <typename Derived>
concept StaticSizeEigenBase = std::derived_from<Derived, Eigen::EigenBase<Derived>>
                           && (Derived::RowsAtCompileTime > 0 && Derived::ColsAtCompileTime > 0);

} // namespace Eigen

namespace std {

template <typename Derived>
requires Eigen::StaticSizeEigenBase<Derived> struct tuple_size<Derived> {
    constexpr static int value = Derived::RowsAtCompileTime * Derived::ColsAtCompileTime;
};
template <size_t D, typename Derived>
requires Eigen::StaticSizeEigenBase<Derived>
      && (D < Derived::RowsAtCompileTime * Derived::ColsAtCompileTime)
struct tuple_element<D, Derived> {
    using type = typename Derived::Scalar;
};

} // namespace std