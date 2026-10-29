#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>

namespace controller_pkg {

inline Eigen::Vector3d vee(const Eigen::Matrix3d& matrix) {
  return Eigen::Vector3d(matrix(2, 1), matrix(0, 2), matrix(1, 0));
}

inline double signedSqrt(const double value) {
  if (!std::isfinite(value)) {
    throw std::runtime_error("cannot take signed square root of non-finite value");
  }
  return value >= 0.0 ? std::sqrt(value) : -std::sqrt(-value);
}

// Simulator convention (top view):
// rotor 1: (+x,+y), positive yaw torque
// rotor 2: (-x,+y), negative yaw torque
// rotor 3: (-x,-y), positive yaw torque
// rotor 4: (+x,-y), negative yaw torque
// The input is u_i = omega_i * abs(omega_i), not omega_i^2.
inline Eigen::Matrix4d allocationMatrix(const double arm_length,
                                        const double lift_coefficient,
                                        const double drag_coefficient) {
  if (arm_length <= 0.0 || lift_coefficient <= 0.0 ||
      drag_coefficient <= 0.0) {
    throw std::invalid_argument("allocation parameters must be positive");
  }

  const double component = arm_length / std::sqrt(2.0);
  Eigen::Matrix4d matrix;
  matrix << lift_coefficient, lift_coefficient, lift_coefficient,
      lift_coefficient,
      component * lift_coefficient, component * lift_coefficient,
      -component * lift_coefficient, -component * lift_coefficient,
      -component * lift_coefficient, component * lift_coefficient,
      component * lift_coefficient, -component * lift_coefficient,
      drag_coefficient, -drag_coefficient, drag_coefficient,
      -drag_coefficient;
  return matrix;
}

inline Eigen::Matrix3d desiredRotation(const Eigen::Vector3d& desired_force,
                                       const double desired_yaw) {
  constexpr double kMinimumNorm = 1e-9;
  if (!desired_force.allFinite() || desired_force.norm() < kMinimumNorm) {
    throw std::runtime_error("desired force is degenerate");
  }

  const Eigen::Vector3d b3_desired = desired_force.normalized();
  Eigen::Vector3d b1_reference(std::cos(desired_yaw),
                               std::sin(desired_yaw), 0.0);
  Eigen::Vector3d cross = b3_desired.cross(b1_reference);

  if (cross.norm() < kMinimumNorm) {
    b1_reference = std::abs(b3_desired.x()) < 0.9
                       ? Eigen::Vector3d::UnitX()
                       : Eigen::Vector3d::UnitY();
    cross = b3_desired.cross(b1_reference);
  }

  const Eigen::Vector3d b2_desired = cross.normalized();
  const Eigen::Vector3d b1_desired =
      b2_desired.cross(b3_desired).normalized();

  Eigen::Matrix3d rotation;
  rotation.col(0) = b1_desired;
  rotation.col(1) = b2_desired;
  rotation.col(2) = b3_desired;
  return rotation;
}

// Preserve the commanded thrust direction while giving heading a configurable
// share of the available attitude response. Weight one is exactly the baseline.
inline Eigen::Matrix3d prioritizeThrustDirection(
    const Eigen::Matrix3d& current, const Eigen::Matrix3d& desired,
    double yaw_weight) {
  if (!std::isfinite(yaw_weight) || yaw_weight < 0. || yaw_weight > 1.)
    throw std::invalid_argument("yaw weight must lie in [0,1]");
  if (yaw_weight == 1.) return desired;
  const Eigen::Quaterniond alignment = Eigen::Quaterniond::FromTwoVectors(
      current.col(2), desired.col(2));
  const Eigen::Matrix3d reduced = alignment.toRotationMatrix() * current;
  const Eigen::Matrix3d remaining = reduced.transpose() * desired;
  const double yaw = std::atan2(remaining(1,0), remaining(0,0));
  return reduced * Eigen::AngleAxisd(yaw_weight*yaw,
                                    Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

inline Eigen::Vector4d solveRotorInputs(const Eigen::Matrix4d& allocation,
                                        const Eigen::Vector4d& wrench) {
  Eigen::FullPivLU<Eigen::Matrix4d> decomposition(allocation);
  if (decomposition.rank() != 4) {
    throw std::runtime_error("allocation matrix is rank deficient");
  }
  const Eigen::Vector4d inputs = decomposition.solve(wrench);
  if (!inputs.allFinite()) {
    throw std::runtime_error("non-finite rotor allocation result");
  }
  return inputs;
}

}  // namespace controller_pkg
