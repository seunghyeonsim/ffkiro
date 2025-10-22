#include "firefighter_flipper/FirefighterControl.hpp"
#include <algorithm>

namespace firefighter_flipper {

FirefighterControl::FirefighterControl(const FirefighterParams& params)
    : params_(params), R_b_(Eigen::Matrix3d::Identity()),
      rho_(Eigen::Vector4d::Zero()), omega_xy_(Eigen::Vector2d::Zero()) {}

void FirefighterControl::setRho(const Eigen::Vector4d& rho)
{
    Eigen::Vector4d clipped = rho;
    for (int i = 0; i < 4; ++i) {
        clipped(i) = std::max(params_.RHO_LO, std::min(params_.RHO_HI, clipped(i)));
    }
    rho_ = clipped;
}

Eigen::Matrix<double,2,4> FirefighterControl::jacobian() const
{
    double F = params_.F;
    double Lx = params_.Lx;
    double Ly = params_.Ly;
    Eigen::Matrix<double,2,4> J;
    J <<  -F/(2*Ly),  F/(2*Ly),  -F/(2*Ly),  F/(2*Ly),
           F/(2*Lx),  F/(2*Lx),  -F/(2*Lx), -F/(2*Lx);
    return J;
}

Eigen::Vector2d FirefighterControl::omegaFromRhoDot(const Eigen::Vector4d& rho_dot) const
{
    Eigen::Matrix<double,2,4> J = jacobian();
    Eigen::Vector4d c;
    for (int i = 0; i < 4; ++i) c(i) = std::cos(rho_(i));
    return J * (c.cwiseProduct(rho_dot));
}

void FirefighterControl::integrateRb(const Eigen::Vector4d& rho_dot, double dt)
{
    omega_xy_ = omegaFromRhoDot(rho_dot);
    Eigen::Vector3d Omega_b(omega_xy_(0), omega_xy_(1), 0.0);
    Eigen::Matrix3d so3 = VecToso3(dt * Omega_b);
    R_b_ = R_b_ * MatrixExp3(so3);
}

} // namespace firefighter_flipper


