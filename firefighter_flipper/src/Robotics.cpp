#include "firefighter_flipper/Robotics.hpp"
#include <cmath>

namespace firefighter_flipper {

bool NearZero(double near) {
    return std::abs(near) < 1e-6;
}

Eigen::Vector3d so3ToVec(const Eigen::Matrix3d& so3mat) {
    Eigen::Vector3d omg;
    omg << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
    return omg;
}

Eigen::Matrix3d VecToso3(const Eigen::Vector3d& omg) {
    Eigen::Matrix3d so3mat;
    so3mat << 0, -omg(2), omg(1),
              omg(2), 0, -omg(0),
              -omg(1), omg(0), 0;
    return so3mat;
}

Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d& R) {
    double acosinput = (R.trace() - 1.0) / 2.0;
    
    if (acosinput >= 1.0) {
        return Eigen::Matrix3d::Zero();
    } else if (acosinput <= -1.0) {
        Eigen::Vector3d omg;
        
        if (!NearZero(1.0 + R(2, 2))) {
            omg = (1.0 / std::sqrt(2.0 * (1.0 + R(2, 2)))) * 
                  Eigen::Vector3d(R(0, 2), R(1, 2), 1.0 + R(2, 2));
        } else if (!NearZero(1.0 + R(1, 1))) {
            omg = (1.0 / std::sqrt(2.0 * (1.0 + R(1, 1)))) * 
                  Eigen::Vector3d(R(0, 1), 1.0 + R(1, 1), R(2, 1));
        } else {
            omg = (1.0 / std::sqrt(2.0 * (1.0 + R(0, 0)))) * 
                  Eigen::Vector3d(1.0 + R(0, 0), R(1, 0), R(2, 0));
        }
        
        return VecToso3(M_PI * omg);
    } else {
        double theta = std::acos(acosinput);
        return theta * (1.0 / (2.0 * std::sin(theta))) * (R - R.transpose());
    }
}

Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3mat) {
    // Rodrigues' formula: exp([w]x * theta) = I + sin(theta)[w]x + (1-cos(theta))[w]x^2
    Eigen::Vector3d omg = so3ToVec(so3mat);
    double theta = omg.norm();
    if (NearZero(theta)) {
        return Eigen::Matrix3d::Identity() + so3mat; // first-order approx
    }
    Eigen::Matrix3d omg_hat = so3mat / theta; // normalized hat
    double s = std::sin(theta);
    double c = std::cos(theta);
    return Eigen::Matrix3d::Identity() + s * omg_hat + (1.0 - c) * (omg_hat * omg_hat);
}

} // namespace firefighter_flipper
