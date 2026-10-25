#include <controller_pkg/attitude_feedforward.h>
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
  std::cout<<"SO3 feedforward constant-rate, clipped-rate, reset and transport tests PASS\n";
}
