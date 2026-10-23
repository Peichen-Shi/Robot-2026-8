#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

#include <cmath>
#include <iostream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "deliverable_1_api_test",
            ros::init_options::AnonymousName);

  tf2::Quaternion original_tf;
  original_tf.setRPY(0.2, -0.3, 0.7);
  original_tf.normalize();

  geometry_msgs::Quaternion quat_msg = tf2::toMsg(original_tf);
  tf2::Quaternion recovered_tf;
  tf2::fromMsg(quat_msg, recovered_tf);

  const double yaw = tf2::getYaw(recovered_tf);
  Eigen::Quaterniond quat_eigen;
  tf2::fromMsg(quat_msg, quat_eigen);
  const Eigen::Matrix3d rotation = quat_eigen.toRotationMatrix();

  const double quaternion_error =
      std::abs(original_tf.x() - recovered_tf.x()) +
      std::abs(original_tf.y() - recovered_tf.y()) +
      std::abs(original_tf.z() - recovered_tf.z()) +
      std::abs(original_tf.w() - recovered_tf.w());
  const double orthogonality_error =
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm();

  const bool pass = quaternion_error < 1e-12 &&
                    std::abs(yaw - 0.7) < 1e-12 &&
                    orthogonality_error < 1e-12 &&
                    std::abs(rotation.determinant() - 1.0) < 1e-12;

  std::cout << "scalar_w=" << recovered_tf.w() << '\n'
            << "yaw_rad=" << yaw << '\n'
            << "quaternion_roundtrip_error=" << quaternion_error << '\n'
            << "rotation_orthogonality_error=" << orthogonality_error << '\n'
            << "rotation_determinant=" << rotation.determinant() << '\n'
            << "status=" << (pass ? "PASS" : "FAIL") << '\n';
  return pass ? 0 : 1;
}
