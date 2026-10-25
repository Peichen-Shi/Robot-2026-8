#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

namespace controller_pkg {
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d out;
  out << 0.,-v.z(),v.y(),v.z(),0.,-v.x(),-v.y(),v.x(),0.;
  return out;
}

// Causal SO(3) reference differentiation, in desired-body coordinates.
// No differentiation of wrapped Euler angles and no partial yaw-only term.
class AttitudeFeedforward {
 public:
  void reset() { ready_=false; omega_.setZero(); alpha_.setZero(); }
  void update(const Eigen::Matrix3d& rotation,double stamp,double tau,
              bool clip_excess_rate=false) {
    const double dt=stamp-last_stamp_;
    if (!ready_ || !std::isfinite(dt) || dt<0.005 || dt>0.10) {
      reset(); ready_=true; previous_=rotation; last_stamp_=stamp; return;
    }
    const Eigen::AngleAxisd delta(previous_.transpose()*rotation);
    Eigen::Vector3d measured=delta.axis()*delta.angle()/dt;
    if (!measured.allFinite()) { reset(); return; }
    if (measured.norm()>8.) {
      if (!clip_excess_rate) { reset(); return; }
      // Preserve the rotation history through a fast corner. Resetting here
      // repeatedly switches the entire feedforward off and on. This limits
      // only the reference derivative; simulator and motor limits stay intact.
      measured *= 8./measured.norm();
    }
    const double gain=dt/(std::max(0.,tau)+dt);
    const Eigen::Vector3d next=omega_+gain*(measured-omega_);
    alpha_ += gain*((next-omega_)/dt-alpha_);
    omega_=next;
    previous_=rotation; last_stamp_=stamp;
  }
  const Eigen::Vector3d& omega() const {return omega_;}
  const Eigen::Vector3d& alpha() const {return alpha_;}
 private:
  bool ready_=false;
  double last_stamp_=0.;
  Eigen::Matrix3d previous_=Eigen::Matrix3d::Identity();
  Eigen::Vector3d omega_=Eigen::Vector3d::Zero(),alpha_=Eigen::Vector3d::Zero();
};
}
