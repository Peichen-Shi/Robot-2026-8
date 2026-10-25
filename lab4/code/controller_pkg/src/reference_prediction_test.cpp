#include <controller_pkg/reference_prediction.h>
#include <iostream>
#include <limits>

int main() {
  const Eigen::Vector3d p0(1.,-2.,3.),v0(4.,3.,-1.),a0(-2.,5.,2.),j(8.,-6.,4.);
  const auto position=[&](double t){return Eigen::Vector3d(p0+v0*t+a0*(t*t/2.)+j*(t*t*t/6.));};
  const auto velocity=[&](double t){return Eigen::Vector3d(v0+a0*t+j*(t*t/2.));};
  const auto acceleration=[&](double t){return Eigen::Vector3d(a0+j*t);};
  const auto yaw=[](double t){return .3+1.2*t+.7*t*t/2.;};
  // Independently evaluate the same cubic at the current measurement time.
  for (double delay:{0.,.01,.02,.04}) {
    const double previous=.7,current=previous+delay;
    const auto r=controller_pkg::predictReference(position(previous),velocity(previous),acceleration(previous),j,
        yaw(previous),1.2+.7*previous,.7,delay,.04);
    if ((r.position-position(current)).norm()>1e-12 ||
        (r.velocity-velocity(current)).norm()>1e-12 ||
        (r.acceleration-acceleration(current)).norm()>1e-12 ||
        std::abs(r.yaw-yaw(current))>1e-12) return 1;
  }
  for(double delay:{-.02,std::numeric_limits<double>::quiet_NaN()}) {
    const auto r=controller_pkg::predictReference(p0,v0,a0,j,.3,1.2,.7,delay,.04);
    if(r.dt!=0. || (r.position-p0).norm()>1e-12)return 2;
  }
  const auto r=controller_pkg::predictReference(p0,v0,a0,j,.3,1.2,.7,1.,.04);
  if(r.dt!=.04 || (r.position-position(.04)).norm()>1e-12)return 3;
  controller_pkg::ReferenceJerk derivative;
  derivative.update(0.,a0);
  for (int i=1;i<30;++i) {
    const double t=i*.02;
    derivative.update(t,acceleration(t));
    derivative.update(t,acceleration(t));
    if((derivative.jerk()-j).norm()>1e-10)return 4;
  }
  derivative.update(0.,a0);
  if(derivative.jerk().norm()>1e-12)return 5;
  std::cout<<"Reference epoch alignment, finite horizon and invalid-lag tests PASS\n";
}
