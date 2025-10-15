#ifndef FIREFIGHTER_FLIPPER__ROBOTICS_HPP_
#define FIREFIGHTER_FLIPPER__ROBOTICS_HPP_

#include <Eigen/Dense>
#include <cmath>

namespace firefighter_flipper {

/**
 * @brief Check if a scalar is small enough to be neglected
 * @param near The scalar value to check
 * @return true if the value is near zero, false otherwise
 */
bool NearZero(double near);

/**
 * @brief Convert a 3x3 skew-symmetric matrix to a 3-vector (angular velocity)
 * @param so3mat 3x3 skew-symmetric matrix (element of so(3))
 * @return 3-vector angular velocity
 */
Eigen::Vector3d so3ToVec(const Eigen::Matrix3d& so3mat);

/**
 * @brief Convert a 3-vector to a 3x3 skew-symmetric matrix
 * @param omg 3-vector angular velocity
 * @return 3x3 skew-symmetric matrix
 */
Eigen::Matrix3d VecToso3(const Eigen::Vector3d& omg);

/**
 * @brief Matrix logarithm of a rotation matrix (so(3) representation)
 * @param R 3x3 rotation matrix
 * @return 3x3 skew-symmetric matrix (so(3) representation)
 */
Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d& R);

/**
 * @brief Matrix exponential for so(3) element
 * @param so3mat 3x3 skew-symmetric matrix (omega_hat * theta)
 * @return 3x3 rotation matrix exp(so3mat)
 */
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3mat);

} // namespace firefighter_flipper

#endif // FIREFIGHTER_FLIPPER__ROBOTICS_HPP_
