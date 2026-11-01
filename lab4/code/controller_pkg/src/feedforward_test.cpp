#include <controller_pkg/attitude_feedforward.h>
#include <controller_pkg/controller_math.h>
#include <Eigen/Geometry>
#include <iostream>
int main() {
  controller_pkg::AttitudeFeedforward d;
  for(int i=0;i<=300;++i) {
    double t=i*.02;
    d.update(Eigen::AngleAxisd(1.2*t,Eigen::Vector3d::UnitZ()).toRotationMatrix(),t,.03);
  }
  if((d.omega()-Eigen::Vector3d(0,0,1.2)).norm()>1e-6 || d.alpha().norm()>1e-6) return 1;
  d.update(Eigen::Matrix3d::Identity(),0.,.03);
  if(d.omega().norm()>1e-10 || d.alpha().norm()>1e-10) return 2;
  const Eigen::Vector3d v(1,2,3),w(2,-1,.5);
  if((controller_pkg::hat(v)*w-v.cross(w)).norm()>1e-12) return 3;
  // A fast continuous rotation across the quaternion/axis-angle wrap must
  // converge to a bounded nonzero rate instead of repeatedly losing history.
  d.reset();
  const Eigen::Vector3d axis=Eigen::Vector3d(1.,2.,-1.).normalized();
  for(int i=0;i<=300;++i) {
    const double t=i*.02;
    d.update(Eigen::AngleAxisd(9.*t,axis).toRotationMatrix(),t,.03,true);
  }
  if((d.omega()-8.*axis).norm()>1e-6 || d.alpha().norm()>1e-6) return 4;
  d.update(Eigen::Matrix3d::Identity(),0.,.03,true);
  if(d.omega().norm()>1e-10 || d.alpha().norm()>1e-10) return 5;
  // With heading free, a target built from current attitude inherits a pure
  // measured yaw spin. It must not cancel the yaw damping term in the loop.
  d.reset();
  controller_pkg::ThrustFrameReference frame;
  for(int i=0;i<=300;++i) {
    const double t=i*.02;
    const Eigen::Matrix3d current=Eigen::AngleAxisd(t,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto reduced=frame.update(current,Eigen::Matrix3d::Identity(),0.);
    d.update(reduced,t,.01,true);
  }
  const Eigen::Vector3d actual_rate(0.,0.,1.);
  if((actual_rate-d.omega()).norm()<.99) return 6;
  frame.reset(); d.reset();
  for(int i=0;i<=300;++i) {
    const double t=i*.02;
    const Eigen::Matrix3d desired=Eigen::AngleAxisd(.8*t,Eigen::Vector3d::UnitY()).toRotationMatrix();
    const auto target=frame.update(Eigen::Matrix3d::Identity(),desired,0.);
    if((target.col(2)-desired.col(2)).norm()>1e-10) return 7;
    d.update(target,t,.01,true);
  }
  if((d.omega()-Eigen::Vector3d(0.,.8,0.)).norm()>1e-6) return 8;
  frame.reset();
  const Eigen::Matrix3d full=Eigen::AngleAxisd(.5,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  if((frame.update(Eigen::Matrix3d::Identity(),full,1.)-full).norm()>1e-12) return 9;
  std::cout<<"SO3 feedforward constant-rate, clipped-rate, reset and transport tests PASS\n";
}
