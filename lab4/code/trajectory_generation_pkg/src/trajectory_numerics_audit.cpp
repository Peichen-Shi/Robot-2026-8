#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

double factor(int power, int derivative) {
  double value = 1.;
  for (int i = 0; i < derivative; ++i) value *= power-i;
  return value;
}

double condition(const Eigen::MatrixXd& matrix) {
  const Eigen::VectorXd singular = matrix.jacobiSvd().singularValues();
  return singular[0]/singular[singular.size()-1];
}

void printArray(const std::array<double,5>& values) {
  std::cout << '[';
  for (int i=0;i<5;++i) std::cout << (i ? "," : "") << values[i];
  std::cout << ']';
}

int main(int argc, char** argv) {
  if (argc < 2 || (argc > 4 && argc != 6)) return 2;
  const bool fast_allocation=argc>=3 && std::string(argv[2])=="--fast-allocation";
  const double terminal_time=argc>=4?std::stod(argv[3]):0.;
  std::ifstream input(argv[1]);
  int count, sample_count;
  Eigen::Vector3d initial;
  double initial_yaw, planned;
  input >> count >> sample_count >> initial.x() >> initial.y() >> initial.z()
        >> initial_yaw >> planned;
  if (!input || count < 1) return 2;
  using namespace mav_trajectory_generation;
  using namespace mav_trajectory_generation::derivative_order;
  Vertex::Vector positions, yaws;
  Vertex start(3), yaw_start(1);
  start.makeStartOrEnd(initial,SNAP); yaw_start.makeStartOrEnd(initial_yaw,SNAP);
  positions.push_back(start); yaws.push_back(yaw_start);
  std::vector<Eigen::Vector3d> targets;
  std::vector<double> yaw_targets;
  Eigen::Vector3d last_position, last_normal;
  double unwrapped=initial_yaw;
  for (int i=0;i<count;++i) {
    Eigen::Vector3d p, normal; double yaw;
    input >> p.x() >> p.y() >> p.z() >> yaw >> normal.x() >> normal.y() >> normal.z();
    unwrapped += std::atan2(std::sin(yaw-unwrapped),std::cos(yaw-unwrapped));
    Vertex position_vertex(3), yaw_vertex(1);
    position_vertex.addConstraint(POSITION,p); yaw_vertex.addConstraint(ORIENTATION,unwrapped);
    positions.push_back(position_vertex); yaws.push_back(yaw_vertex);
    targets.push_back(p); yaw_targets.push_back(unwrapped);
    last_position=p; last_normal=normal;
  }
  Vertex end(3), yaw_end(1);
  end.makeStartOrEnd(last_position+last_normal,SNAP); yaw_end.makeStartOrEnd(unwrapped,SNAP);
  positions.push_back(end); yaws.push_back(yaw_end);
  std::vector<double> times=estimateSegmentTimes(positions,8.,4.);
  double weight_sum=0.;
  for(double& dt:times) {dt=std::max(.1,dt*(fast_allocation?1.:3.));weight_sum+=dt;}
  if(fast_allocation) {
    if(planned<=.1*times.size()) return 3;
    for(double& dt:times) dt=.1+(planned-.1*times.size())*dt/weight_sum;
    if(terminal_time>0.) {
      const size_t n=times.size()-1;double sum=0.;
      for(size_t i=0;i<n;++i)sum+=times[i];
      for(size_t i=0;i<n;++i)times[i]=.1+(planned-terminal_time-.1*n)*times[i]/sum;
      times.back()=terminal_time;
    }
  }
  // Independently reconstruct a configured segment schedule from the recording.
  if (argc == 6) {
    if (std::string(argv[4]) != "--segment-times") return 2;
    std::ifstream schedule(argv[5]);
    std::vector<double> configured;
    double value;
    while (schedule >> value) {
      if (!std::isfinite(value) || value < 0.1) return 3;
      configured.push_back(value);
    }
    if (configured.size() != times.size()) return 3;
    times = configured;
  }
  double total=0., maximum_condition=0., maximum_normalized_condition=0.;
  for (double& duration:times) {
    total+=duration;
    Eigen::MatrixXd a=Eigen::MatrixXd::Zero(10,10), normalized=a;
    for(int endpoint=0;endpoint<2;++endpoint) for(int derivative=0;derivative<5;++derivative)
      for(int power=derivative;power<10;++power) {
        const int row=endpoint*5+derivative;
        const double t=endpoint ? duration : 0.;
        a(row,power)=factor(power,derivative)*std::pow(t,power-derivative);
        normalized(row,power)=factor(power,derivative)*
          std::pow(double(endpoint),power-derivative)/std::pow(duration,derivative);
      }
    maximum_condition=std::max(maximum_condition,condition(a));
    maximum_normalized_condition=std::max(maximum_normalized_condition,condition(normalized));
  }
  PolynomialOptimization<10> optimizer(3), yaw_optimizer(1);
  if (!optimizer.setupFromVertices(positions,times,SNAP) ||
      !yaw_optimizer.setupFromVertices(yaws,times,SNAP) ||
      !optimizer.solveLinear() || !yaw_optimizer.solveLinear()) return 3;
  Trajectory trajectory,yaw_trajectory;
  optimizer.getTrajectory(&trajectory); yaw_optimizer.getTrajectory(&yaw_trajectory);
  Segment::Vector segments,yaw_segments;
  trajectory.getSegments(&segments); yaw_trajectory.getSegments(&yaw_segments);
  std::array<double,5> continuity{{0,0,0,0,0}}, yaw_continuity{{0,0,0,0,0}};
  for(size_t i=0;i+1<segments.size();++i) for(int derivative=0;derivative<5;++derivative) {
    continuity[derivative]=std::max(continuity[derivative],
      (segments[i].evaluate(times[i],derivative)-segments[i+1].evaluate(0.,derivative)).norm());
    yaw_continuity[derivative]=std::max(yaw_continuity[derivative],
      (yaw_segments[i].evaluate(times[i],derivative)-yaw_segments[i+1].evaluate(0.,derivative)).norm());
  }
  double elapsed=0., vertex_error=0., vertex_yaw_error=0.;
  for(int i=0;i<count;++i) {
    elapsed+=times[i];
    vertex_error=std::max(vertex_error,(trajectory.evaluate(elapsed,POSITION)-targets[i]).norm());
    vertex_yaw_error=std::max(vertex_yaw_error,
      std::abs(yaw_trajectory.evaluate(elapsed,ORIENTATION)[0]-yaw_targets[i]));
  }
  double position_match=0., velocity_match=0., acceleration_match=0., yaw_match=0.;
  for(int i=0;i<sample_count;++i) {
    double tau,yaw; Eigen::Vector3d p,v,a;
    input >> tau >> p.x() >> p.y() >> p.z() >> yaw >> v.x() >> v.y() >> v.z()
          >> a.x() >> a.y() >> a.z();
    if (!input) return 4;
    tau=std::min(tau,total);
    position_match=std::max(position_match,(trajectory.evaluate(tau,POSITION)-p).norm());
    velocity_match=std::max(velocity_match,(trajectory.evaluate(tau,VELOCITY)-v).norm());
    acceleration_match=std::max(acceleration_match,(trajectory.evaluate(tau,ACCELERATION)-a).norm());
    const double delta=yaw_trajectory.evaluate(tau,ORIENTATION)[0]-yaw;
    yaw_match=std::max(yaw_match,std::abs(std::atan2(std::sin(delta),std::cos(delta))));
  }
  const double duration_error=std::abs(total-planned);
  bool valid=duration_error<1e-5 && vertex_error<1e-5 && vertex_yaw_error<1e-5 &&
    position_match<1e-5 && velocity_match<1e-5 && acceleration_match<1e-5 && yaw_match<1e-5;
  for(int i=0;i<5;++i) valid=valid && continuity[i]<1e-5 && yaw_continuity[i]<1e-5;
  std::cout << std::setprecision(17) << "{\"passed\":" << (valid?"true":"false")
    << ",\"planned_duration_s\":" << planned << ",\"reconstructed_duration_error_s\":" << duration_error
    << ",\"maximum_physical_time_boundary_map_condition\":" << maximum_condition
    << ",\"maximum_normalized_time_boundary_map_condition\":" << maximum_normalized_condition
    << ",\"maximum_gate_position_residual_m\":" << vertex_error
    << ",\"maximum_gate_yaw_residual_rad\":" << vertex_yaw_error
    << ",\"recorded_position_match_max_m\":" << position_match
    << ",\"recorded_velocity_match_max_mps\":" << velocity_match
    << ",\"recorded_acceleration_match_max_mps2\":" << acceleration_match
    << ",\"recorded_yaw_match_max_rad\":" << yaw_match
    << ",\"position_continuity_residual_by_derivative_0_to_4\":";
  printArray(continuity); std::cout << ",\"yaw_continuity_residual_by_derivative_0_to_4\":";
  printArray(yaw_continuity); std::cout << "}\n";
  return valid?0:1;
}
