#ifndef FIREFIGHTER_FLIPPER__FIREFIGHTER_CONTROL_HPP_
#define FIREFIGHTER_FLIPPER__FIREFIGHTER_CONTROL_HPP_

#include <Eigen/Dense>
#include "firefighter_flipper/Robotics.hpp"

namespace firefighter_flipper {

struct FirefighterParams {
    double F;   // force scale
    double Lx;  // geometry x
    double Ly;  // geometry y
    double RHO_LO; // lower bound
    double RHO_HI; // upper bound

    FirefighterParams()
        : F(0.51), Lx(0.7239), Ly(0.906), RHO_LO(-M_PI), RHO_HI(M_PI) {}
};

class FirefighterControl {
public:
    explicit FirefighterControl(const FirefighterParams& params = FirefighterParams());

    // setters/getters
    void setRho(const Eigen::Vector4d& rho);
    const Eigen::Vector4d& getRho() const { return rho_; }

    // Jacobian (2x4): [omega_x; omega_y] = J * (cos(rho) .* rho_dot)
    Eigen::Matrix<double,2,4> jacobian() const;

    // omega from rho_dot
    Eigen::Vector2d omegaFromRhoDot(const Eigen::Vector4d& rho_dot) const;

    // integrate R_b with body angular velocity Omega_b = [omega_x, omega_y, 0]
    void integrateRb(const Eigen::Vector4d& rho_dot, double dt);

    const Eigen::Matrix3d& bodyRotation() const { return R_b_; }

private:
    FirefighterParams params_;
    Eigen::Matrix3d R_b_;
    Eigen::Vector4d rho_;
    Eigen::Vector2d omega_xy_;
};

} // namespace firefighter_flipper

#endif // FIREFIGHTER_FLIPPER__FIREFIGHTER_CONTROL_HPP_


