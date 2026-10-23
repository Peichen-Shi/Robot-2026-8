#include <controller_pkg/controller_math.h>

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

namespace {

void require(const bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  constexpr double kMass = 1.0;
  constexpr double kGravity = 9.81;
  constexpr double kArm = 0.3;
  constexpr double kLift = 1e-3;
  constexpr double kDrag = 1e-5;
  constexpr double kTolerance = 1e-9;

  const Eigen::Matrix4d allocation =
      controller_pkg::allocationMatrix(kArm, kLift, kDrag);
  require(allocation.fullPivLu().rank() == 4,
          "allocation matrix must have rank four");

  Eigen::Vector4d hover_wrench(kMass * kGravity, 0.0, 0.0, 0.0);
  const Eigen::Vector4d hover_inputs =
      controller_pkg::solveRotorInputs(allocation, hover_wrench);
  const double expected_input = kMass * kGravity / (4.0 * kLift);
  require((hover_inputs.array() - expected_input).abs().maxCoeff() <
              kTolerance,
          "hover inputs are not equal");

  for (int i = 0; i < 4; ++i) {
    const double speed = controller_pkg::signedSqrt(hover_inputs(i));
    require(std::abs(speed - std::sqrt(expected_input)) < kTolerance,
            "hover speed has the wrong unit or sign");
  }

  const Eigen::Matrix3d desired_rotation =
      controller_pkg::desiredRotation(Eigen::Vector3d(0.4, -0.2, 9.81), 0.7);
  require((desired_rotation.transpose() * desired_rotation -
           Eigen::Matrix3d::Identity())
                  .norm() < kTolerance,
          "desired rotation is not orthogonal");
  require(std::abs(desired_rotation.determinant() - 1.0) < kTolerance,
          "desired rotation determinant is not one");

  std::mt19937 generator(314159);
  std::uniform_real_distribution<double> distribution(-4000.0, 4000.0);
  double maximum_residual = 0.0;
  for (int sample = 0; sample < 1000; ++sample) {
    Eigen::Vector4d original_inputs;
    for (int i = 0; i < 4; ++i) {
      original_inputs(i) = distribution(generator);
    }
    const Eigen::Vector4d wrench = allocation * original_inputs;
    const Eigen::Vector4d recovered =
        controller_pkg::solveRotorInputs(allocation, wrench);
    maximum_residual =
        std::max(maximum_residual, (recovered - original_inputs).norm());
  }
  require(maximum_residual < 1e-8, "allocation round-trip residual is too large");

  std::cout << "allocation_rank=4\n"
            << "hover_rotor_speed_rad_s=" << std::sqrt(expected_input) << '\n'
            << "rotation_orthogonality_error="
            << (desired_rotation.transpose() * desired_rotation -
                Eigen::Matrix3d::Identity())
                   .norm()
            << '\n'
            << "rotation_determinant=" << desired_rotation.determinant() << '\n'
            << "maximum_allocation_roundtrip_residual=" << maximum_residual
            << '\n'
            << "status=PASS\n";
  return 0;
}
