#include <controller_pkg/flatness_feedforward.h>
#include <Eigen/Geometry>
#include <iostream>
controller_pkg::AttitudeJet sample(double t) {
  const Eigen::Vector3d f(2*std::sin(t),3*std::cos(.7*t),9.81+.2*std::sin(2*t));
  const Eigen::Vector3d fd(2*std::cos(t),-2.1*std::sin(.7*t),.4*std::cos(2*t));
  const Eigen::Vector3d fdd(-2*std::sin(t),-1.47*std::cos(.7*t),-.8*std::sin(2*t));
  return controller_pkg::attitudeJet(f,fd,fdd,.8*t+.1*t*t,.8+.2*t,.2);
}
int main() {
  double worst=0.;const double h=1e-4;
  for(int i=0;i<100;++i) {
    double t=i*.1;const auto a=sample(t),b=sample(t-h),c=sample(t+h);
    worst=std::max(worst,((c.R-b.R)/(2*h)-a.Rdot).norm());
    worst=std::max(worst,((c.R-2*a.R+b.R)/(h*h)-a.Rddot).norm());
    worst=std::max(worst,((c.omega-b.omega)/(2*h)-a.alpha).norm());
    if((a.R.transpose()*a.R-Eigen::Matrix3d::Identity()).norm()>1e-10)return 2;
  }
  std::cout<<"Analytic SO3 jets vs independent central differences: "<<worst<<"\n";
  return worst<2e-6?0:1;
}
