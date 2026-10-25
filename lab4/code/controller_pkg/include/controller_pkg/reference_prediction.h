#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace controller_pkg {
class ReferenceJerk {
 public:
  void update(double stamp,const Eigen::Vector3d& acceleration) {
    const double dt=stamp-last_stamp_;
    if (!ready_ || dt<0. || dt>.10) {
      jerk_.setZero();
      ready_=true;
    } else if (dt<.005) {
      // A 100 Hz ROS timer can publish twice at the same 50 Hz sim epoch.
      // A duplicate contains no new derivative information; keep history.
      return;
    } else {
      jerk_=(acceleration-previous_acceleration_)/dt;
    }
    previous_acceleration_=acceleration;
    last_stamp_=stamp;
  }
  const Eigen::Vector3d& jerk() const {return jerk_;}
 private:
  bool ready_=false;
  double last_stamp_=0.;
  Eigen::Vector3d previous_acceleration_=Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk_=Eigen::Vector3d::Zero();
};

struct PredictedReference {
  Eigen::Vector3d position, velocity, acceleration;
  double yaw, dt;
};

// Advance only the commanded reference to the measurement's source epoch.
// Actual odometry, the recorded reference and simulation time are untouched.
inline PredictedReference predictReference(
    const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& acceleration, const Eigen::Vector3d& jerk,
    double yaw, double yaw_rate, double yaw_acceleration,
    double source_lag, double maximum_prediction) {
  const double dt=std::isfinite(source_lag)
      ? std::max(0.,std::min(maximum_prediction,source_lag)) : 0.;
  const double dt2=dt*dt,dt3=dt2*dt;
  return {position+velocity*dt+acceleration*(.5*dt2)+jerk*(dt3/6.),
          velocity+acceleration*dt+jerk*(.5*dt2),
          acceleration+jerk*dt,
          yaw+yaw_rate*dt+.5*yaw_acceleration*dt2,dt};
}
}
