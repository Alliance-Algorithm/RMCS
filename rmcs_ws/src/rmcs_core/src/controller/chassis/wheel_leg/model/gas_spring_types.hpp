#pragma once
namespace rmcs_core::controller::chassis::gas_spring_model {
struct GeometryKnot {
    double delta_rad, beta_deg, compression_m, hip_length_jacobian, knee_length_jacobian;
};
struct ForceKnot {
    double compression_m, force_n;
};
} // namespace rmcs_core::controller::chassis::gas_spring_model
