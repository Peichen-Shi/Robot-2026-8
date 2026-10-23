#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace controller_pkg {
struct VectorJet {
  Eigen::Vector3d p,d,dd;
};
inline VectorJet crossJet(const VectorJet& a,const VectorJet& b) {
  return {a.p.cross(b.p),a.d.cross(b.p)+a.p.cross(b.d),
      a.dd.cross(b.p)+2.*a.d.cross(b.d)+a.p.cross(b.dd)};
}
inline VectorJet normalizedJet(const VectorJet& a) {
  const double n=a.p.norm();
  if(!std::isfinite(n)||n<1e-6)throw std::runtime_error("degenerate flatness jet");
  const Eigen::Vector3d u=a.p/n;
  const double nd=u.dot(a.d);
  const Eigen::Vector3d ud=(a.d-u*nd)/n;
  return {u,ud,(a.dd-u*u.dot(a.dd)-2.*nd*ud)/n-u*ud.squaredNorm()};
}
struct AttitudeJet {
  Eigen::Matrix3d R,Rdot,Rddot;
  Eigen::Vector3d omega,alpha;
};
inline AttitudeJet attitudeJet(const Eigen::Vector3d& force,const Eigen::Vector3d& fd,
    const Eigen::Vector3d& fdd,double yaw,double yaw_rate,double yaw_accel) {
  const Eigen::Vector3d heading(std::cos(yaw),std::sin(yaw),0.);
  const Eigen::Vector3d tangent(-std::sin(yaw),std::cos(yaw),0.);
  const VectorJet b3=normalizedJet({force,fd,fdd});
  const VectorJet b1c{heading,yaw_rate*tangent,yaw_accel*tangent-yaw_rate*yaw_rate*heading};
  const VectorJet b2=normalizedJet(crossJet(b3,b1c));
  const VectorJet b1=crossJet(b2,b3);
  AttitudeJet out;
  out.R.col(0)=b1.p;out.R.col(1)=b2.p;out.R.col(2)=b3.p;
  out.Rdot.col(0)=b1.d;out.Rdot.col(1)=b2.d;out.Rdot.col(2)=b3.d;
  out.Rddot.col(0)=b1.dd;out.Rddot.col(1)=b2.dd;out.Rddot.col(2)=b3.dd;
  const Eigen::Matrix3d w=out.R.transpose()*out.Rdot;
  const Eigen::Matrix3d a=out.Rdot.transpose()*out.Rdot+out.R.transpose()*out.Rddot;
  out.omega=Eigen::Vector3d(w(2,1)-w(1,2),w(0,2)-w(2,0),w(1,0)-w(0,1))*.5;
  out.alpha=Eigen::Vector3d(a(2,1)-a(1,2),a(0,2)-a(2,0),a(1,0)-a(0,1))*.5;
  if(!out.omega.allFinite()||!out.alpha.allFinite())throw std::runtime_error("nonfinite flatness derivatives");
  return out;
}
}
