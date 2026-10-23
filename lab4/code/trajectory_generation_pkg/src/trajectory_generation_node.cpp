#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_generation/TrajectoryStatus.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

bool finitePose(const geometry_msgs::Pose& pose) {
  const auto& p = pose.position;
  const auto& q = pose.orientation;
  const double q_norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q_norm_sq) && q_norm_sq > 1e-12;
}

double unwrapYaw(double previous, double wrapped) {
  return previous +
         std::atan2(std::sin(wrapped - previous),
                    std::cos(wrapped - previous));
}

}  // namespace

class WaypointFollower {
 public:
  explicit WaypointFollower(ros::NodeHandle& nh)
      : nh_(nh),
        private_nh_("~"),
        current_position_(Eigen::Vector3d::Zero()),
        current_yaw_(0.0),
        have_current_state_(false),
        have_pending_vertices_(false),
        trajectory_ready_(false),
        phase_("WAITING"),
        trajectory_id_(0),
        publish_frequency_(100.0),
        v_max_(8.0),
        a_max_(4.0),
        time_scale_(1.0),
        minimum_segment_time_(0.10),
        exit_distance_(1.0),
        ready_hold_duration_(5.0),
        post_hold_duration_(60.0) {
    private_nh_.param("target_duration", target_duration_, 0.0);
    private_nh_.param("terminal_segment_time", terminal_segment_time_, 0.0);
    private_nh_.param("publish_frequency", publish_frequency_,
                      publish_frequency_);
    private_nh_.param("v_max", v_max_, v_max_);
    private_nh_.param("a_max", a_max_, a_max_);
    private_nh_.param("time_scale", time_scale_, time_scale_);
    private_nh_.param("minimum_segment_time", minimum_segment_time_,
                      minimum_segment_time_);
    private_nh_.param("exit_distance", exit_distance_, exit_distance_);
    private_nh_.param("ready_hold_duration", ready_hold_duration_,
                      ready_hold_duration_);
    private_nh_.param("post_hold_duration", post_hold_duration_,
                      post_hold_duration_);
    private_nh_.param<std::string>("run_id", run_id_, "");
    if (!(publish_frequency_ > 0.0 && v_max_ > 0.0 && a_max_ > 0.0 &&
          time_scale_ > 0.0 && minimum_segment_time_ > 0.0) ||
        exit_distance_ < 0.0 || ready_hold_duration_ < 0.0 ||
        post_hold_duration_ < 60.0) {
      throw std::invalid_argument(
          "trajectory parameters invalid; post_hold_duration must be >= 60 s");
    }
    if (run_id_.empty()) {
      std::ostringstream stream;
      stream << "lab4-" << ros::WallTime::now().toNSec();
      run_id_ = stream.str();
    }

    current_state_sub_ = nh_.subscribe(
        "/current_state", 10, &WaypointFollower::onCurrentState, this);
    pose_array_sub_ = nh_.subscribe(
        "/desired_traj_vertices", 1, &WaypointFollower::onVertices, this);
    desired_state_pub_ =
        nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/desired_state", 1);
    stamped_state_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        "/desired_state_stamped", 1);
    status_pub_ = nh_.advertise<trajectory_generation::TrajectoryStatus>(
        "/trajectory_status", 10, true);
    reset_service_ = private_nh_.advertiseService(
        "reset", &WaypointFollower::onReset, this);
    desired_state_timer_ = nh_.createTimer(
        ros::Duration(1.0 / publish_frequency_),
        &WaypointFollower::publishDesiredState, this);
    ROS_INFO("Trajectory follower waiting for valid current state and gates");
  }

 private:
  void onCurrentState(const nav_msgs::Odometry& state) {
    if (!finitePose(state.pose.pose)) {
      ROS_WARN_THROTTLE(1.0, "Ignored invalid current_state pose");
      return;
    }
    current_position_ << state.pose.pose.position.x,
        state.pose.pose.position.y, state.pose.pose.position.z;
    tf2::Quaternion orientation;
    tf2::fromMsg(state.pose.pose.orientation, orientation);
    orientation.normalize();
    current_yaw_ = tf2::getYaw(orientation);
    have_current_state_ = true;
    if (have_pending_vertices_ && !trajectory_ready_ && phase_ != "FAULT") {
      generateOptimizedTrajectory(pending_vertices_);
    }
  }

  void onVertices(const geometry_msgs::PoseArray& message) {
    if (message.poses.empty()) {
      ROS_WARN_THROTTLE(1.0, "Ignored empty desired_traj_vertices");
      return;
    }
    for (const auto& pose : message.poses) {
      if (!finitePose(pose)) {
        ROS_ERROR_THROTTLE(1.0,
                           "Ignored desired_traj_vertices with invalid pose");
        return;
      }
    }
    pending_vertices_ = message;
    have_pending_vertices_ = true;
    if (have_current_state_ && !trajectory_ready_ && phase_ != "FAULT") {
      generateOptimizedTrajectory(pending_vertices_);
    }
  }

  bool onReset(std_srvs::Trigger::Request&,
               std_srvs::Trigger::Response& response) {
    clearTrajectory("explicit reset service");
    response.success = true;
    response.message = "trajectory cleared; next valid state will replan";
    return true;
  }

  void clearTrajectory(const std::string& reason) {
    trajectory_.clear();
    yaw_trajectory_.clear();
    trajectory_ready_ = false;
    trajectory_start_time_ = ros::Time();
    mission_start_time_ = ros::Time();
    last_ros_time_ = ros::Time();
    phase_ = "WAITING";
    fault_reason_.clear();
    ROS_WARN_STREAM("Trajectory state reset: " << reason);
  }

  void enterFault(const std::string& reason) {
    phase_ = "FAULT";
    fault_reason_ = reason;
    trajectory_ready_ = false;
    ROS_ERROR_STREAM("Trajectory mission entered FAULT: " << reason);
    publishStatus(ros::Time::now(), 0.0, 0.0, false);
  }

  void generateOptimizedTrajectory(const geometry_msgs::PoseArray& poses) {
    if (!have_current_state_ || poses.poses.empty()) return;
    // With /use_sim_time, state and latched gate messages can arrive before
    // this process has received its first /clock sample.  Planning at ROS time
    // zero would make the first published setpoint jump forward by the entire
    // simulator uptime.  Defer; the next 50 Hz state callback will retry.
    if (ros::Time::now().isZero()) {
      ROS_WARN_THROTTLE(1.0,
                        "Waiting for non-zero ROS time before trajectory start");
      return;
    }

    using namespace mav_trajectory_generation::derivative_order;
    constexpr int kPositionDimension = 3;
    constexpr int kPolynomialCoefficients = 10;
    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex::Vector yaw_vertices;
    const bool append_exit = exit_distance_ > 1e-9;
    vertices.reserve(poses.poses.size() + 1 + (append_exit ? 1 : 0));
    yaw_vertices.reserve(poses.poses.size() + 1 + (append_exit ? 1 : 0));

    mav_trajectory_generation::Vertex start_position(kPositionDimension);
    start_position.makeStartOrEnd(current_position_, SNAP);
    vertices.push_back(start_position);
    mav_trajectory_generation::Vertex start_yaw(1);
    start_yaw.makeStartOrEnd(current_yaw_, SNAP);
    yaw_vertices.push_back(start_yaw);

    double unwrapped_yaw = current_yaw_;
    for (size_t index = 0; index < poses.poses.size(); ++index) {
      const geometry_msgs::Pose& pose = poses.poses[index];
      Eigen::Vector3d position(pose.position.x, pose.position.y,
                               pose.position.z);
      tf2::Quaternion quaternion;
      tf2::fromMsg(pose.orientation, quaternion);
      quaternion.normalize();
      unwrapped_yaw = unwrapYaw(unwrapped_yaw, tf2::getYaw(quaternion));

      mav_trajectory_generation::Vertex position_vertex(kPositionDimension);
      mav_trajectory_generation::Vertex yaw_vertex(1);
      const bool is_last = index + 1 == poses.poses.size() && !append_exit;
      if (is_last) {
        position_vertex.makeStartOrEnd(position, SNAP);
        yaw_vertex.makeStartOrEnd(unwrapped_yaw, SNAP);
      } else {
        position_vertex.addConstraint(POSITION, position);
        yaw_vertex.addConstraint(ORIENTATION, unwrapped_yaw);
      }
      vertices.push_back(position_vertex);
      yaw_vertices.push_back(yaw_vertex);
    }

    // A trajectory ending exactly at the final gate centre only proves arrival;
    // it does not prove that the vehicle crossed the gate plane.  The gate's
    // local +x axis is its forward normal, so append a short terminal point on
    // that axis.  Every official gate remains an exact position/yaw constraint.
    if (append_exit) {
      const geometry_msgs::Pose& final_gate = poses.poses.back();
      tf2::Quaternion gate_orientation;
      tf2::fromMsg(final_gate.orientation, gate_orientation);
      gate_orientation.normalize();
      const tf2::Vector3 exit_offset = tf2::quatRotate(
          gate_orientation, tf2::Vector3(exit_distance_, 0.0, 0.0));
      const Eigen::Vector3d exit_position(
          final_gate.position.x + exit_offset.x(),
          final_gate.position.y + exit_offset.y(),
          final_gate.position.z + exit_offset.z());
      mav_trajectory_generation::Vertex exit_position_vertex(
          kPositionDimension);
      exit_position_vertex.makeStartOrEnd(exit_position, SNAP);
      vertices.push_back(exit_position_vertex);
      mav_trajectory_generation::Vertex exit_yaw_vertex(1);
      exit_yaw_vertex.makeStartOrEnd(unwrapped_yaw, SNAP);
      yaw_vertices.push_back(exit_yaw_vertex);
    }

    std::vector<double> segment_times =
        estimateSegmentTimes(vertices, v_max_, a_max_);
    if (segment_times.size() + 1 != vertices.size() ||
        yaw_vertices.size() != vertices.size()) {
      ROS_ERROR("Vertex and segment counts are inconsistent");
      return;
    }
    for (double& duration : segment_times) {
      duration = std::max(minimum_segment_time_, duration * time_scale_);
      if (!std::isfinite(duration) || duration <= 0.0) {
        ROS_ERROR("Invalid segment duration encountered");
        return;
      }
    }
    // Keep every official position/yaw constraint. Time is allocated before
    // solving, never changed by accelerating playback or scaling /clock.
    if (!std::isfinite(target_duration_) || target_duration_ < 0.0) {
      enterFault("invalid requested target duration");
      return;
    }
    if (target_duration_ > 0.0) {
      if (target_duration_ <= minimum_segment_time_ * segment_times.size()) {
        enterFault("target duration cannot satisfy segment lower bounds");
        return;
      }
      double weight_sum = 0.0;
      for (const double dt : segment_times) weight_sum += dt;
      const double remaining = target_duration_ -
          minimum_segment_time_ * segment_times.size();
      for (double& dt : segment_times) {
        dt = minimum_segment_time_ + remaining * dt / weight_sum;
      }
      // Reserve a deliberate final braking interval instead of squeezing the
      // one-metre exit into the same distance-based timing as the race gates.
      // Gates/order/exit geometry are unchanged and the entire budget is fixed.
      if (terminal_segment_time_ > 0.0 && append_exit) {
        const size_t n=segment_times.size()-1;
        const double available=target_duration_-terminal_segment_time_-n*minimum_segment_time_;
        if (!std::isfinite(terminal_segment_time_) || available<=0.) {
          enterFault("invalid reserved terminal duration"); return;
        }
        double sum=0.; for(size_t i=0;i<n;++i)sum+=segment_times[i];
        for(size_t i=0;i<n;++i)segment_times[i]=minimum_segment_time_+available*segment_times[i]/sum;
        segment_times.back()=terminal_segment_time_;
      }
    }

    mav_trajectory_generation::PolynomialOptimization<
        kPolynomialCoefficients>
        position_optimizer(kPositionDimension);
    mav_trajectory_generation::PolynomialOptimization<
        kPolynomialCoefficients>
        yaw_optimizer(1);
    if (!position_optimizer.setupFromVertices(vertices, segment_times, SNAP) ||
        !yaw_optimizer.setupFromVertices(yaw_vertices, segment_times, SNAP)) {
      ROS_ERROR("Failed to set up polynomial optimization");
      return;
    }
    if (!position_optimizer.solveLinear() || !yaw_optimizer.solveLinear()) {
      ROS_ERROR("Polynomial optimization solver failed");
      return;
    }
    position_optimizer.getTrajectory(&trajectory_);
    yaw_optimizer.getTrajectory(&yaw_trajectory_);
    if (trajectory_.empty() || yaw_trajectory_.empty() ||
        !std::isfinite(trajectory_.getMaxTime()) ||
        trajectory_.getMaxTime() <= 0.0 ||
        std::abs(trajectory_.getMaxTime() - yaw_trajectory_.getMaxTime()) >
            1e-6) {
      ROS_ERROR("Optimized trajectory is empty or has inconsistent duration");
      clearTrajectory("invalid optimization result");
      return;
    }

    double cumulative_time = 0.0;
    double max_position_residual = 0.0;
    double max_yaw_residual = 0.0;
    for (size_t index = 0; index < poses.poses.size(); ++index) {
      cumulative_time += segment_times[index];
      const Eigen::Vector3d evaluated =
          trajectory_.evaluate(cumulative_time, POSITION);
      const auto& target = poses.poses[index].position;
      max_position_residual = std::max(
          max_position_residual,
          (evaluated - Eigen::Vector3d(target.x, target.y, target.z)).norm());
      Eigen::VectorXd yaw_value =
          yaw_trajectory_.evaluate(cumulative_time, ORIENTATION);
      tf2::Quaternion q;
      tf2::fromMsg(poses.poses[index].orientation, q);
      q.normalize();
      const double wrapped_residual =
          std::atan2(std::sin(yaw_value[0] - tf2::getYaw(q)),
                     std::cos(yaw_value[0] - tf2::getYaw(q)));
      max_yaw_residual = std::max(max_yaw_residual,
                                  std::abs(wrapped_residual));
    }
    if (max_position_residual > 1e-5 || max_yaw_residual > 1e-5) {
      ROS_ERROR("Waypoint residual too large: position %.3e m, yaw %.3e rad",
                max_position_residual, max_yaw_residual);
      clearTrajectory("constraint residual failure");
      return;
    }

    // Sample every 50 ms in trajectory time and reject any non-finite output.
    // This also records independent peak kinematics before the mission starts.
    double peak_speed = 0.0;
    double peak_acceleration = 0.0;
    double peak_jerk = 0.0;
    double peak_snap = 0.0;
    double peak_yaw_rate = 0.0;
    const double total_duration = trajectory_.getMaxTime();
    for (double sample = 0.0; sample <= total_duration + 1e-9;
         sample += 0.05) {
      const double tau = std::min(sample, total_duration);
      const Eigen::Vector3d p = trajectory_.evaluate(tau, POSITION);
      const Eigen::Vector3d v = trajectory_.evaluate(tau, VELOCITY);
      const Eigen::Vector3d a = trajectory_.evaluate(tau, ACCELERATION);
      const Eigen::Vector3d j = trajectory_.evaluate(tau, JERK);
      const Eigen::Vector3d s = trajectory_.evaluate(tau, SNAP);
      const Eigen::VectorXd yaw =
          yaw_trajectory_.evaluate(tau, ORIENTATION);
      const Eigen::VectorXd yaw_rate =
          yaw_trajectory_.evaluate(tau, VELOCITY);
      if (!p.allFinite() || !v.allFinite() || !a.allFinite() ||
          !j.allFinite() || !s.allFinite() || !yaw.allFinite() ||
          !yaw_rate.allFinite()) {
        enterFault("non-finite optimized trajectory sample");
        return;
      }
      peak_speed = std::max(peak_speed, v.norm());
      peak_acceleration = std::max(peak_acceleration, a.norm());
      peak_jerk = std::max(peak_jerk, j.norm());
      peak_snap = std::max(peak_snap, s.norm());
      peak_yaw_rate = std::max(peak_yaw_rate, std::abs(yaw_rate[0]));
    }

    ready_hold_position_ = current_position_;
    ready_hold_yaw_ = current_yaw_;
    mission_start_time_ = ros::Time::now();
    if (mission_start_time_.isZero()) {
      clearTrajectory("ROS time returned to zero during optimization");
      return;
    }
    trajectory_start_time_ = ros::Time();
    last_ros_time_ = mission_start_time_;
    ++trajectory_id_;
    phase_ = "READY_HOLD";
    fault_reason_.clear();
    trajectory_ready_ = true;
    ROS_INFO("Generated trajectory id=%u, %zu segments, T=%.3f s; peaks "
             "v/a/j/s/yaw_rate=%.3f/%.3f/%.3f/%.3f/%.3f; residuals %.3e m / %.3e rad",
             trajectory_id_, segment_times.size(), total_duration, peak_speed,
             peak_acceleration, peak_jerk, peak_snap, peak_yaw_rate,
             max_position_residual, max_yaw_residual);
  }

  void publishDesiredState(const ros::TimerEvent&) {
    if (!trajectory_ready_) return;
    const ros::Time now = ros::Time::now();
    if (now.isZero()) return;
    if (!last_ros_time_.isZero() && now + ros::Duration(1e-6) < last_ros_time_) {
      enterFault("ROS time moved backwards; explicit reset required");
      return;
    }
    last_ros_time_ = now;
    const double mission_elapsed = (now - mission_start_time_).toSec();
    if (!std::isfinite(mission_elapsed) || mission_elapsed < 0.0) {
      enterFault("invalid mission elapsed time");
      return;
    }

    if (phase_ == "READY_HOLD") {
      publishSetpoint(ready_hold_position_, Eigen::Vector3d::Zero(),
                      Eigen::Vector3d::Zero(), ready_hold_yaw_, 0.0, 0.0,
                      0.0);
      publishStatus(now, mission_elapsed, 0.0, false);
      if (mission_elapsed >= ready_hold_duration_) {
        trajectory_start_time_ = now;
        phase_ = "TRACKING";
        ROS_INFO("Trajectory id=%u entered TRACKING", trajectory_id_);
      }
      return;
    }

    if (trajectory_start_time_.isZero()) {
      enterFault("TRACKING reached without a valid trajectory start time");
      return;
    }
    const double elapsed = (now - trajectory_start_time_).toSec();
    if (!std::isfinite(elapsed) || elapsed < 0.0) {
      enterFault("invalid trajectory elapsed time");
      return;
    }
    const double total_duration = trajectory_.getMaxTime();
    const double sampling_time = std::min(elapsed, total_duration);

    using namespace mav_trajectory_generation::derivative_order;
    const Eigen::Vector3d position =
        trajectory_.evaluate(sampling_time, POSITION);
    const double yaw = yaw_trajectory_.evaluate(sampling_time, ORIENTATION)[0];
    if (phase_ == "TRACKING" && elapsed < total_duration) {
      publishSetpoint(
          position, trajectory_.evaluate(sampling_time, VELOCITY),
          trajectory_.evaluate(sampling_time, ACCELERATION), yaw,
          yaw_trajectory_.evaluate(sampling_time, VELOCITY)[0],
          yaw_trajectory_.evaluate(sampling_time, ACCELERATION)[0],
          sampling_time);
      publishStatus(now, mission_elapsed, elapsed, false);
      ROS_INFO_THROTTLE(2.0, "Trajectory id=%u progress %.1f%%",
                        trajectory_id_, 100.0 * sampling_time / total_duration);
      return;
    }

    if (phase_ == "TRACKING") {
      phase_ = "POST_RACE_HOLD";
      post_hold_start_time_ = now;
      ROS_INFO("Trajectory id=%u entered POST_RACE_HOLD for %.1f s",
               trajectory_id_, post_hold_duration_);
    }
    // Explicit zeros remove numerical polynomial residue, but the endpoint is
    // fixed and never replaced by the vehicle's drifting current position.
    publishSetpoint(position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                    yaw, 0.0, 0.0, total_duration);
    if (phase_ == "POST_RACE_HOLD" &&
        (now - post_hold_start_time_).toSec() >= post_hold_duration_) {
      phase_ = "FINISHED";
      ROS_INFO("Trajectory id=%u completed the required terminal hold",
               trajectory_id_);
    }
    publishStatus(now, mission_elapsed, elapsed, true);
  }

  void publishSetpoint(const Eigen::Vector3d& position,
                       const Eigen::Vector3d& velocity,
                       const Eigen::Vector3d& acceleration,
                       const double yaw, const double yaw_rate,
                       const double yaw_acceleration, const double tau) {
    trajectory_msgs::MultiDOFJointTrajectoryPoint output;
    output.transforms.resize(1);
    output.velocities.resize(1);
    output.accelerations.resize(1);
    output.transforms[0].translation.x = position.x();
    output.transforms[0].translation.y = position.y();
    output.transforms[0].translation.z = position.z();
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    output.transforms[0].rotation = tf2::toMsg(quaternion);
    output.velocities[0].linear.x = velocity.x();
    output.velocities[0].linear.y = velocity.y();
    output.velocities[0].linear.z = velocity.z();
    output.velocities[0].angular.z = yaw_rate;
    output.accelerations[0].linear.x = acceleration.x();
    output.accelerations[0].linear.y = acceleration.y();
    output.accelerations[0].linear.z = acceleration.z();
    output.accelerations[0].angular.z = yaw_acceleration;
    output.time_from_start = ros::Duration(tau);
    desired_state_pub_.publish(output);
    trajectory_msgs::MultiDOFJointTrajectory stamped;
    stamped.header.stamp = ros::Time::now();
    stamped.header.frame_id = "world";
    stamped.joint_names.push_back("base_link_gt");
    stamped.points.push_back(output);
    stamped_state_pub_.publish(stamped);
  }

  void publishStatus(const ros::Time& now, const double mission_elapsed,
                     const double trajectory_elapsed,
                     const bool reference_complete) {
    trajectory_generation::TrajectoryStatus output;
    output.header.stamp = now;
    output.header.frame_id = "world";
    output.trajectory_id = trajectory_id_;
    output.run_id = run_id_;
    output.phase = phase_;
    output.planned_duration_s = trajectory_.empty() ? 0.0 : trajectory_.getMaxTime();
    output.mission_elapsed_s = mission_elapsed;
    output.trajectory_elapsed_s = trajectory_elapsed;
    output.sample_tau_s = trajectory_.empty()
                              ? 0.0
                              : std::min(std::max(0.0, trajectory_elapsed),
                                         trajectory_.getMaxTime());
    output.ready_hold_required_s = ready_hold_duration_;
    output.post_hold_required_s = post_hold_duration_;
    output.trajectory_valid = trajectory_ready_ && phase_ != "FAULT";
    output.reference_complete = reference_complete;
    output.fault = phase_ == "FAULT";
    output.reason = fault_reason_;
    status_pub_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber current_state_sub_;
  ros::Subscriber pose_array_sub_;
  ros::Publisher desired_state_pub_;
  ros::Publisher stamped_state_pub_;
  ros::Publisher status_pub_;
  ros::ServiceServer reset_service_;
  ros::Timer desired_state_timer_;

  Eigen::Vector3d current_position_;
  double current_yaw_;
  bool have_current_state_;
  bool have_pending_vertices_;
  bool trajectory_ready_;
  std::string phase_;
  std::string run_id_;
  std::string fault_reason_;
  uint32_t trajectory_id_;
  geometry_msgs::PoseArray pending_vertices_;
  mav_trajectory_generation::Trajectory trajectory_;
  mav_trajectory_generation::Trajectory yaw_trajectory_;
  ros::Time trajectory_start_time_;
  ros::Time mission_start_time_;
  ros::Time post_hold_start_time_;
  ros::Time last_ros_time_;
  Eigen::Vector3d ready_hold_position_;
  double ready_hold_yaw_;

  double publish_frequency_;
  double v_max_;
  double a_max_;
  double time_scale_;
  double minimum_segment_time_;
  double exit_distance_;
  double ready_hold_duration_;
  double post_hold_duration_;
  double target_duration_;
  double terminal_segment_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generation_node");
  ros::NodeHandle nh;
  try {
    WaypointFollower follower(nh);
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("Trajectory generator initialization failed: %s",
              exception.what());
    return 1;
  }
  return 0;
}
