#include <controller_pkg/ControllerDiagnostics.h>
#include <controller_pkg/controller_math.h>
#include <controller_pkg/attitude_feedforward.h>
#include <controller_pkg/reference_prediction.h>
#include <trajectory_generation/TrajectoryStatus.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

geometry_msgs::Vector3 vectorMessage(const Eigen::Vector3d& value) {
  geometry_msgs::Vector3 message;
  message.x = value.x();
  message.y = value.y();
  message.z = value.z();
  return message;
}

geometry_msgs::Point pointMessage(const Eigen::Vector3d& value) {
  geometry_msgs::Point message;
  message.x = value.x();
  message.y = value.y();
  message.z = value.z();
  return message;
}

double clampScalar(const double value, const double lower, const double upper) {
  return std::max(lower, std::min(upper, value));
}

}  // namespace

class ControllerNode {
 public:
  ControllerNode()
      : private_nh_("~"),
        e3_(Eigen::Vector3d::UnitZ()),
        current_position_(Eigen::Vector3d::Zero()),
        current_velocity_world_(Eigen::Vector3d::Zero()),
        current_rotation_(Eigen::Matrix3d::Identity()),
        current_omega_body_(Eigen::Vector3d::Zero()),
        desired_position_(Eigen::Vector3d::Zero()),
        desired_velocity_(Eigen::Vector3d::Zero()),
        desired_acceleration_(Eigen::Vector3d::Zero()),
        desired_yaw_(0.0),
        desired_yaw_rate_(0.0),
        desired_time_from_start_(0.0),
        have_current_state_(false),
        have_desired_state_(false),
        fault_active_(true),
        force_limited_(false),
        moment_limited_(false),
        task_phase_(0),
        task_phase_name_("WAITING") {
    private_nh_.param("kx", kx_, 4.0);
    private_nh_.param("kv", kv_, 5.0);
    private_nh_.param("kx_z", kx_z_, kx_);
    private_nh_.param("kv_z", kv_z_, kv_);
    private_nh_.param("yaw_weight", yaw_weight_, 1.0);
    private_nh_.param("kr", kr_, 4.0);
    private_nh_.param("komega", komega_, 1.5);
    private_nh_.param("hold_kr", hold_kr_, kr_);
    private_nh_.param("hold_komega", hold_komega_, komega_);
    private_nh_.param("state_timeout", state_timeout_, 0.5);
    private_nh_.param("reference_timeout", reference_timeout_, 0.5);
    private_nh_.param("zero_command_period", zero_command_period_, 0.05);
    private_nh_.param("twist_linear_in_body", twist_linear_in_body_, false);
    private_nh_.param("twist_angular_in_body", twist_angular_in_body_, false);
    private_nh_.param("mass", mass_, 1.0);
    private_nh_.param("gravity", gravity_, 9.81);
    private_nh_.param("arm_length", arm_length_, 0.3);
    private_nh_.param("lift_coefficient", lift_coefficient_, 1e-3);
    private_nh_.param("drag_coefficient", drag_coefficient_, 1e-5);
    private_nh_.param("inertia_x", inertia_x_, 0.18);
    private_nh_.param("inertia_y", inertia_y_, 0.26);
    private_nh_.param("inertia_z", inertia_z_, 0.16);
    private_nh_.param("maximum_tilt_deg", maximum_tilt_deg_, 35.0);
    private_nh_.param("minimum_vertical_force", minimum_vertical_force_, 1.0);
    private_nh_.param("minimum_thrust", minimum_thrust_, 0.0);
    private_nh_.param("maximum_thrust", maximum_thrust_, 25.0);
    private_nh_.param("maximum_moment_x", maximum_moment_x_, 1.0);
    private_nh_.param("maximum_moment_y", maximum_moment_y_, 1.0);
    private_nh_.param("maximum_moment_z", maximum_moment_z_, 0.5);
    private_nh_.param("hold_maximum_moment_x", hold_maximum_moment_x_, maximum_moment_x_);
    private_nh_.param("hold_maximum_moment_y", hold_maximum_moment_y_, maximum_moment_y_);
    private_nh_.param("hold_maximum_moment_z", hold_maximum_moment_z_, maximum_moment_z_);
    private_nh_.param("attitude_feedforward", attitude_feedforward_, false);
    // During stationary holds, avoid differentiating feedback corrections.
    private_nh_.param("feedforward_tracking_only", feedforward_tracking_only_, false);
    private_nh_.param("feedforward_clip_rate", feedforward_clip_rate_, false);
    private_nh_.param("reference_prediction", reference_prediction_, false);
    private_nh_.param("maximum_reference_prediction", maximum_reference_prediction_, 0.04);
    private_nh_.param("actuation_prediction", actuation_prediction_, 0.0);
    private_nh_.param("acceleration_lead", acceleration_lead_, 0.0);
    private_nh_.param("braking_lead_after", braking_lead_after_, -1.0);
    private_nh_.param("braking_acceleration_lead", braking_acceleration_lead_, 0.0);
    private_nh_.param("preserve_horizontal_force", preserve_horizontal_force_, false);
    private_nh_.param("bound_vertical_thrust", bound_vertical_thrust_, false);
    private_nh_.param("corner_force_bias_after", corner_force_bias_after_, -1.0);
    private_nh_.param("corner_force_bias_x", corner_force_bias_x_, 0.0);
    private_nh_.param("corner_force_bias_y", corner_force_bias_y_, 0.0);
    private_nh_.param("feedforward_filter_tau", feedforward_filter_tau_, 0.03);
    private_nh_.param("use_stamped_reference", use_stamped_reference_, false);

    if (!std::isfinite(corner_force_bias_after_) || corner_force_bias_after_ < -1. ||
        !std::isfinite(corner_force_bias_x_) || std::abs(corner_force_bias_x_) > 10. ||
        !std::isfinite(corner_force_bias_y_) || std::abs(corner_force_bias_y_) > 10. ||
        !std::isfinite(braking_lead_after_) || braking_lead_after_ < -1. ||
        !std::isfinite(braking_acceleration_lead_) || braking_acceleration_lead_ < 0. ||
        braking_acceleration_lead_ > .20 ||
        !std::isfinite(acceleration_lead_) || acceleration_lead_ < 0. ||
        acceleration_lead_ > .12 ||
        !std::isfinite(actuation_prediction_) || actuation_prediction_ < 0. ||
        actuation_prediction_ > .04 ||
        !std::isfinite(hold_kr_) || hold_kr_ <= 0. ||
        !std::isfinite(hold_komega_) || hold_komega_ <= 0. ||
        !std::isfinite(hold_maximum_moment_x_) || hold_maximum_moment_x_ <= 0. ||
        !std::isfinite(hold_maximum_moment_y_) || hold_maximum_moment_y_ <= 0. ||
        !std::isfinite(hold_maximum_moment_z_) || hold_maximum_moment_z_ <= 0. ||
        !std::isfinite(kx_z_) || !std::isfinite(kv_z_) ||
        kx_z_ <= 0. || kv_z_ <= 0. || !std::isfinite(yaw_weight_) ||
        yaw_weight_ < 0. || yaw_weight_ > 1. ||
        state_timeout_ <= 0.0 || reference_timeout_ <= 0.0 ||
        zero_command_period_ <= 0.0 || mass_ <= 0.0 || gravity_ <= 0.0 ||
        arm_length_ <= 0.0 || lift_coefficient_ <= 0.0 ||
        drag_coefficient_ <= 0.0 || inertia_x_ <= 0.0 || inertia_y_ <= 0.0 ||
        inertia_z_ <= 0.0 || maximum_tilt_deg_ <= 0.0 ||
        maximum_tilt_deg_ >= 89.0 || minimum_vertical_force_ <= 0.0 ||
        maximum_thrust_ <= minimum_thrust_ || maximum_moment_x_ <= 0.0 ||
        maximum_moment_y_ <= 0.0 || maximum_moment_z_ <= 0.0 ||
        !std::isfinite(maximum_reference_prediction_) ||
        maximum_reference_prediction_ < 0.0 || maximum_reference_prediction_ > 0.10) {
      throw std::invalid_argument("invalid controller or safety-envelope parameter");
    }

    inertia_ = Eigen::Vector3d(inertia_x_, inertia_y_, inertia_z_).asDiagonal();
    allocation_ = controller_pkg::allocationMatrix(
        arm_length_, lift_coefficient_, drag_coefficient_);

    if (use_stamped_reference_) {
      desired_state_subscriber_ = nh_.subscribe("desired_state_stamped",1,
          &ControllerNode::onStampedDesiredState,this,ros::TransportHints().tcpNoDelay());
    } else {
      desired_state_subscriber_ = nh_.subscribe(
          "desired_state", 1, &ControllerNode::onDesiredState, this,
          ros::TransportHints().tcpNoDelay());
    }
    current_state_subscriber_ = nh_.subscribe(
        "current_state", 1, &ControllerNode::onCurrentState, this,
        ros::TransportHints().tcpNoDelay());
    trajectory_status_subscriber_ = nh_.subscribe(
        "trajectory_status", 1, &ControllerNode::onTrajectoryStatus, this);
    rotor_speed_publisher_ =
        nh_.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);
    diagnostic_publisher_ =
        nh_.advertise<controller_pkg::ControllerDiagnostics>(
            "controller_diagnostics", 10);

    ROS_INFO_STREAM("State-synchronous geometric controller initialized: kx="
                    << kx_ << ", kv=" << kv_ << ", kr=" << kr_
                    << ", komega=" << komega_ << ", J=[" << inertia_x_
                    << "," << inertia_y_ << "," << inertia_z_
                    << "], max tilt=" << maximum_tilt_deg_
                    << " deg, moment limits=[" << maximum_moment_x_ << ","
                    << maximum_moment_y_ << "," << maximum_moment_z_ << "]");
  }

  ~ControllerNode() {
    if (ros::isInitialized()) {
      publishZeroCommand("controller shutdown", true);
      ros::WallDuration(0.03).sleep();
      publishZeroCommand("controller shutdown", true);
    }
  }

 private:
  static bool isFiniteQuaternion(const geometry_msgs::Quaternion& quaternion) {
    const double squared_norm =
        quaternion.x * quaternion.x + quaternion.y * quaternion.y +
        quaternion.z * quaternion.z + quaternion.w * quaternion.w;
    return std::isfinite(squared_norm) && squared_norm > 1e-12;
  }

  void onTrajectoryStatus(
      const trajectory_generation::TrajectoryStatus& message) {
    trajectory_id_ = std::to_string(message.trajectory_id);
    task_phase_name_ = message.phase;
    if (message.phase == "READY_HOLD") task_phase_ = 1;
    else if (message.phase == "TRACKING") task_phase_ = 2;
    else if (message.phase == "POST_RACE_HOLD") task_phase_ = 3;
    else if (message.phase == "FINISHED") task_phase_ = 4;
    else if (message.phase == "FAULT") task_phase_ = 5;
    else task_phase_ = 0;
    if (!use_stamped_reference_) reference_source_ros_time_ = message.header.stamp;
  }

  void onDesiredState(
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& message) {
    if (message.transforms.empty() || message.velocities.empty() ||
        message.accelerations.empty()) {
      ROS_WARN_THROTTLE(1.0, "Rejected incomplete desired_state");
      return;
    }
    const geometry_msgs::Transform& transform = message.transforms.front();
    if (!isFiniteQuaternion(transform.rotation)) {
      ROS_WARN_THROTTLE(1.0, "Rejected desired_state: invalid quaternion");
      return;
    }
    const Eigen::Vector3d desired_position(transform.translation.x,
                                           transform.translation.y,
                                           transform.translation.z);
    const Eigen::Vector3d desired_velocity(message.velocities.front().linear.x,
                                           message.velocities.front().linear.y,
                                           message.velocities.front().linear.z);
    const Eigen::Vector3d desired_acceleration(
        message.accelerations.front().linear.x,
        message.accelerations.front().linear.y,
        message.accelerations.front().linear.z);
    if (!desired_position.allFinite() || !desired_velocity.allFinite() ||
        !desired_acceleration.allFinite() ||
        !std::isfinite(message.velocities.front().angular.z)) {
      ROS_WARN_THROTTLE(1.0, "Rejected desired_state: non-finite value");
      return;
    }
    tf2::Quaternion desired_quaternion;
    tf2::fromMsg(transform.rotation, desired_quaternion);
    desired_quaternion.normalize();
    // Use causal jerk differences and the published analytic yaw acceleration
    // for a short prediction to the current measurement's timestamp.
    reference_jerk_.update(message.time_from_start.toSec(),desired_acceleration);
    desired_yaw_acceleration_=std::isfinite(message.accelerations.front().angular.z)
        ? message.accelerations.front().angular.z : 0.;
    desired_position_ = desired_position;
    desired_velocity_ = desired_velocity;
    desired_acceleration_ = desired_acceleration;
    desired_yaw_ = tf2::getYaw(desired_quaternion);
    desired_yaw_rate_ = message.velocities.front().angular.z;
    if (message.time_from_start.toSec()+1e-6 < desired_time_from_start_) {
      attitude_derivatives_.reset();
    }
    desired_time_from_start_ = message.time_from_start.toSec();
    desired_state_wall_time_ = ros::WallTime::now();
    have_desired_state_ = true;
    ++reference_sequence_;
  }

  void onStampedDesiredState(const trajectory_msgs::MultiDOFJointTrajectory& message) {
    if (message.points.size()!=1 || message.header.stamp.isZero()) return;
    const auto before=reference_sequence_;
    onDesiredState(message.points.front());
    if (reference_sequence_!=before) reference_source_ros_time_=message.header.stamp;
  }

  void onCurrentState(const nav_msgs::Odometry& message) {
    if (!isFiniteQuaternion(message.pose.pose.orientation)) {
      ROS_WARN_THROTTLE(1.0, "Rejected current_state: invalid quaternion");
      return;
    }
    Eigen::Quaterniond quaternion;
    tf2::fromMsg(message.pose.pose.orientation, quaternion);
    quaternion.normalize();
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    const Eigen::Vector3d position(message.pose.pose.position.x,
                                   message.pose.pose.position.y,
                                   message.pose.pose.position.z);
    const Eigen::Vector3d raw_linear(message.twist.twist.linear.x,
                                     message.twist.twist.linear.y,
                                     message.twist.twist.linear.z);
    const Eigen::Vector3d raw_angular(message.twist.twist.angular.x,
                                      message.twist.twist.angular.y,
                                      message.twist.twist.angular.z);
    if (!position.allFinite() || !raw_linear.allFinite() ||
        !raw_angular.allFinite() || !rotation.allFinite()) {
      ROS_WARN_THROTTLE(1.0, "Rejected current_state: non-finite state");
      return;
    }
    const ros::WallTime wall_now = ros::WallTime::now();
    const ros::Time source_stamp = message.header.stamp.isZero()
                                       ? ros::Time::now()
                                       : message.header.stamp;
    state_interval_wall_ = last_state_wall_time_.isZero()
                               ? 0.0
                               : (wall_now - last_state_wall_time_).toSec();
    state_interval_ros_ = last_state_source_time_.isZero()
                              ? 0.0
                              : (source_stamp - last_state_source_time_).toSec();
    last_state_wall_time_ = wall_now;
    last_state_source_time_ = source_stamp;
    current_message_orientation_ = message.pose.pose.orientation;
    current_position_ = position;
    current_rotation_ = rotation;
    current_velocity_world_ =
        twist_linear_in_body_ ? rotation * raw_linear : raw_linear;
    // Baseline Run 05 finite differences match a world-frame angular velocity.
    current_omega_body_ =
        twist_angular_in_body_ ? raw_angular : rotation.transpose() * raw_angular;
    current_state_wall_time_ = wall_now;
    have_current_state_ = true;
    controlLoop(source_stamp);
  }

  Eigen::Vector3d limitDesiredForce(const Eigen::Vector3d& unconstrained) {
    Eigen::Vector3d limited = unconstrained;
    limited.z() = std::max(minimum_vertical_force_, limited.z());
    const double horizontal_norm = limited.head<2>().norm();
    const double maximum_horizontal =
        limited.z() * std::tan(maximum_tilt_deg_ * M_PI / 180.0);
    if (horizontal_norm > maximum_horizontal) {
      if (preserve_horizontal_force_ && task_phase_name_=="TRACKING") {
        // Keep braking authority in a fast corner. Raise only the minimum
        // vertical component required by the upright cone; never tilt past it.
        limited.z()=horizontal_norm/std::tan(maximum_tilt_deg_*M_PI/180.0);
      } else {
        limited.head<2>() *= maximum_horizontal / horizontal_norm;
      }
    }
    force_limited_ = (limited - unconstrained).norm() > 1e-9;
    return limited;
  }

  Eigen::Vector3d limitMoment(const Eigen::Vector3d& unconstrained) {
    const bool tracking=task_phase_name_=="TRACKING";
    const double mx=tracking?maximum_moment_x_:hold_maximum_moment_x_;
    const double my=tracking?maximum_moment_y_:hold_maximum_moment_y_;
    const double mz=tracking?maximum_moment_z_:hold_maximum_moment_z_;
    const Eigen::Vector3d limited(
        clampScalar(unconstrained.x(), -mx, mx),
        clampScalar(unconstrained.y(), -my, my),
        clampScalar(unconstrained.z(), -mz, mz));
    moment_limited_ = (limited - unconstrained).norm() > 1e-9;
    return limited;
  }

  void controlLoop(const ros::Time& source_stamp) {
    if (!have_current_state_ || !have_desired_state_) {
      publishZeroCommand("state not initialized");
      publishFaultDiagnostics(source_stamp, "state not initialized");
      return;
    }
    const ros::WallTime now = ros::WallTime::now();
    if ((now - current_state_wall_time_).toSec() > state_timeout_ ||
        (now - desired_state_wall_time_).toSec() > reference_timeout_) {
      publishZeroCommand("state or reference timeout");
      publishFaultDiagnostics(source_stamp, "state or reference timeout");
      return;
    }

    try {
      const double source_lag=reference_prediction_ && !reference_source_ros_time_.isZero()
          ? (source_stamp-reference_source_ros_time_).toSec() : 0.;
      const auto reference=controller_pkg::predictReference(
          desired_position_,desired_velocity_,desired_acceleration_,reference_jerk_.jerk(),
          desired_yaw_,desired_yaw_rate_,desired_yaw_acceleration_,
          source_lag+(task_phase_name_=="TRACKING"?actuation_prediction_:0.),
          maximum_reference_prediction_);
      const Eigen::Vector3d position_error =
          current_position_ - reference.position;
      const Eigen::Vector3d velocity_error =
          current_velocity_world_ - reference.velocity;
      const double force_lead=task_phase_name_=="TRACKING" ?
          ((braking_lead_after_>=0. && desired_time_from_start_>=braking_lead_after_)
              ?braking_acceleration_lead_:acceleration_lead_) : 0.;
      Eigen::Vector3d desired_force_unconstrained =
          -Eigen::Vector3d(kx_,kx_,kx_z_).cwiseProduct(position_error) -
          Eigen::Vector3d(kv_,kv_,kv_z_).cwiseProduct(velocity_error) +
          mass_ * gravity_ * e3_ + mass_ * (reference.acceleration +
              force_lead*reference_jerk_.jerk());
      if (task_phase_name_=="TRACKING" && corner_force_bias_after_>=0. &&
          desired_time_from_start_>=corner_force_bias_after_ &&
          (braking_lead_after_<0. || desired_time_from_start_<braking_lead_after_)) {
        // Small learned feedforward compensates repeatable final-turn drift.
        // Actual state and logged tracking errors remain unmodified.
        const double blend=std::min(1.,(desired_time_from_start_-corner_force_bias_after_)/.2);
        desired_force_unconstrained += blend*Eigen::Vector3d(corner_force_bias_x_,corner_force_bias_y_,0.);
      }
      const Eigen::Vector3d desired_force =
          limitDesiredForce(desired_force_unconstrained);
      const Eigen::Matrix3d desired_rotation =
          controller_pkg::prioritizeThrustDirection(current_rotation_,
              controller_pkg::desiredRotation(desired_force, reference.yaw),
              task_phase_name_ == "TRACKING" ? yaw_weight_ : 1.0);
      const Eigen::Vector3d rotation_error = 0.5 * controller_pkg::vee(
          desired_rotation.transpose() * current_rotation_ -
          current_rotation_.transpose() * desired_rotation);
      // The baseline switch retains simplified geometric PD. The speed profile
      // enables the complete SO(3) reference rate/acceleration transport pair,
      // not a yaw-only subtraction without the matching inertial terms.
      Eigen::Vector3d angular_rate_error = current_omega_body_;
      Eigen::Vector3d angular_feedforward = Eigen::Vector3d::Zero();
      if (attitude_feedforward_ &&
          (!feedforward_tracking_only_ || task_phase_name_ == "TRACKING")) {
        attitude_derivatives_.update(desired_rotation,source_stamp.toSec(),feedforward_filter_tau_,feedforward_clip_rate_);
        const Eigen::Vector3d omega=attitude_derivatives_.omega(),alpha=attitude_derivatives_.alpha();
        const Eigen::Matrix3d transport=current_rotation_.transpose()*desired_rotation;
        const Eigen::Vector3d desired_omega=transport*omega;
        angular_rate_error -= desired_omega;
        angular_feedforward=-inertia_*(controller_pkg::hat(current_omega_body_)*desired_omega-
            transport*alpha);
      } else {
        attitude_derivatives_.reset();
      }

      double thrust_unconstrained =
          desired_force.dot(current_rotation_ * e3_);
      if (bound_vertical_thrust_ && task_phase_name_=="TRACKING" && current_rotation_(2,2)>.05) {
        // During a bank reversal, force projection alone can inject excess
        // lift. Bound that vertical acceleration while attitude catches up.
        thrust_unconstrained=std::min(thrust_unconstrained,
            desired_force.z()/current_rotation_(2,2));
      }
      const double thrust = clampScalar(thrust_unconstrained,
                                        minimum_thrust_, maximum_thrust_);
      const Eigen::Vector3d moment_unconstrained =
          -(task_phase_name_=="TRACKING"?kr_:hold_kr_) * rotation_error -
          (task_phase_name_=="TRACKING"?komega_:hold_komega_) * angular_rate_error +
          current_omega_body_.cross(inertia_ * current_omega_body_) + angular_feedforward;
      const Eigen::Vector3d moment = limitMoment(moment_unconstrained);

      Eigen::Vector4d wrench;
      wrench << thrust, moment.x(), moment.y(), moment.z();
      const Eigen::Vector4d rotor_inputs =
          controller_pkg::solveRotorInputs(allocation_, wrench);
      const double allocation_residual =
          (allocation_ * rotor_inputs - wrench).norm();
      if (!std::isfinite(allocation_residual) || allocation_residual > 1e-7) {
        throw std::runtime_error("rotor allocation residual is too large");
      }

      mav_msgs::Actuators command;
      command.header.stamp = source_stamp;
      command.angular_velocities.resize(4);
      for (int index = 0; index < 4; ++index) {
        command.angular_velocities[index] =
            controller_pkg::signedSqrt(rotor_inputs(index));
      }
      rotor_speed_publisher_.publish(command);
      fault_active_ = false;
      // Keep diagnostic errors against the unmodified published reference.
      // Prediction must not make the recorded tracking metrics look better.
      publishDiagnostics(source_stamp, current_position_-desired_position_,
                         current_velocity_world_-desired_velocity_,
                         rotation_error, angular_rate_error,
                         desired_force_unconstrained, desired_force,
                         thrust_unconstrained, thrust, moment_unconstrained,
                         moment, command, allocation_residual, false, "");
    } catch (const std::exception& exception) {
      publishZeroCommand("control exception");
      publishFaultDiagnostics(source_stamp, exception.what());
    }
  }

  void publishDiagnostics(
      const ros::Time& stamp, const Eigen::Vector3d& position_error,
      const Eigen::Vector3d& velocity_error,
      const Eigen::Vector3d& attitude_error,
      const Eigen::Vector3d& angular_rate_error,
      const Eigen::Vector3d& desired_force_unconstrained,
      const Eigen::Vector3d& desired_force_limited,
      const double thrust_unconstrained, const double thrust,
      const Eigen::Vector3d& moment_unconstrained,
      const Eigen::Vector3d& moment, const mav_msgs::Actuators& command,
      const double allocation_residual, const bool fault,
      const std::string& reason) {
    controller_pkg::ControllerDiagnostics output;
    output.header.stamp = stamp;
    output.header.frame_id = "world";
    output.trajectory_id = trajectory_id_;
    output.task_phase = task_phase_;
    output.task_phase_name = task_phase_name_;
    output.position = pointMessage(current_position_);
    output.desired_position = pointMessage(desired_position_);
    output.velocity = vectorMessage(current_velocity_world_);
    output.desired_velocity = vectorMessage(desired_velocity_);
    output.desired_acceleration = vectorMessage(desired_acceleration_);
    output.attitude = current_message_orientation_;
    output.body_angular_velocity = vectorMessage(current_omega_body_);
    output.desired_yaw = desired_yaw_;
    output.desired_yaw_rate = desired_yaw_rate_;
    output.desired_time_from_start_s = desired_time_from_start_;
    output.reference_source_ros_s = reference_source_ros_time_.toSec();
    output.position_error = vectorMessage(position_error);
    output.velocity_error = vectorMessage(velocity_error);
    output.attitude_error = vectorMessage(attitude_error);
    output.angular_rate_error = vectorMessage(angular_rate_error);
    output.desired_force_unconstrained = vectorMessage(desired_force_unconstrained);
    output.desired_force_limited = vectorMessage(desired_force_limited);
    output.thrust_unconstrained = thrust_unconstrained;
    output.thrust_command = thrust;
    output.moment_unconstrained = vectorMessage(moment_unconstrained);
    output.moment_command = vectorMessage(moment);
    for (size_t index = 0; index < 4; ++index) {
      output.requested_rotor_speeds[index] =
          index < command.angular_velocities.size()
              ? command.angular_velocities[index]
              : 0.0;
    }
    output.allocation_residual = allocation_residual;
    const ros::WallTime now = ros::WallTime::now();
    output.state_age_wall_s = have_current_state_
                                  ? (now - current_state_wall_time_).toSec()
                                  : std::numeric_limits<double>::infinity();
    output.reference_age_wall_s = have_desired_state_
                                      ? (now - desired_state_wall_time_).toSec()
                                      : std::numeric_limits<double>::infinity();
    output.state_interval_ros_s = state_interval_ros_;
    output.state_interval_wall_s = state_interval_wall_;
    output.force_limited = force_limited_;
    output.moment_limited = moment_limited_;
    output.control_fault = fault;
    output.fault_reason = reason;
    diagnostic_publisher_.publish(output);
  }

  void publishFaultDiagnostics(const ros::Time& stamp,
                               const std::string& reason) {
    mav_msgs::Actuators zero;
    zero.angular_velocities.assign(4, 0.0);
    publishDiagnostics(stamp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.0,
                       0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                       zero, 0.0, true, reason);
  }

  void publishZeroCommand(const std::string& reason, bool force = false) {
    const ros::WallTime now = ros::WallTime::now();
    if (!force && !last_zero_wall_time_.isZero() &&
        (now - last_zero_wall_time_).toSec() < zero_command_period_) {
      return;
    }
    mav_msgs::Actuators command;
    command.header.stamp = ros::Time::now();
    command.angular_velocities.assign(4, 0.0);
    rotor_speed_publisher_.publish(command);
    last_zero_wall_time_ = now;
    if (!fault_active_ || force) {
      ROS_ERROR_STREAM("Fail-safe zero rotor command: " << reason);
    }
    fault_active_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber desired_state_subscriber_;
  ros::Subscriber current_state_subscriber_;
  ros::Subscriber trajectory_status_subscriber_;
  ros::Publisher rotor_speed_publisher_;
  ros::Publisher diagnostic_publisher_;
  double kx_, kv_, kr_, komega_;
  double hold_kr_, hold_komega_;
  double hold_maximum_moment_x_,hold_maximum_moment_y_,hold_maximum_moment_z_;
  double kx_z_, kv_z_, yaw_weight_;
  double state_timeout_, reference_timeout_, zero_command_period_;
  bool twist_linear_in_body_, twist_angular_in_body_;
  double mass_, gravity_, arm_length_, lift_coefficient_, drag_coefficient_;
  double inertia_x_, inertia_y_, inertia_z_;
  double maximum_tilt_deg_, minimum_vertical_force_;
  double minimum_thrust_, maximum_thrust_;
  double maximum_moment_x_, maximum_moment_y_, maximum_moment_z_;
  bool attitude_feedforward_, use_stamped_reference_;
  bool feedforward_tracking_only_;
  bool feedforward_clip_rate_;
  bool reference_prediction_;
  double maximum_reference_prediction_;
  double actuation_prediction_;
  double acceleration_lead_;
  double braking_lead_after_,braking_acceleration_lead_;
  bool preserve_horizontal_force_,bound_vertical_thrust_;
  double corner_force_bias_after_,corner_force_bias_x_,corner_force_bias_y_;
  uint64_t reference_sequence_=0;
  double feedforward_filter_tau_;
  controller_pkg::AttitudeFeedforward attitude_derivatives_;
  const Eigen::Vector3d e3_;
  Eigen::Matrix3d inertia_;
  Eigen::Matrix4d allocation_;
  Eigen::Vector3d current_position_, current_velocity_world_;
  Eigen::Matrix3d current_rotation_;
  Eigen::Vector3d current_omega_body_;
  geometry_msgs::Quaternion current_message_orientation_;
  Eigen::Vector3d desired_position_, desired_velocity_, desired_acceleration_;
  controller_pkg::ReferenceJerk reference_jerk_;
  double desired_yaw_acceleration_=0.;
  double desired_yaw_, desired_yaw_rate_, desired_time_from_start_;
  bool have_current_state_, have_desired_state_, fault_active_;
  bool force_limited_, moment_limited_;
  uint8_t task_phase_;
  std::string task_phase_name_, trajectory_id_;
  ros::Time reference_source_ros_time_, last_state_source_time_;
  ros::WallTime current_state_wall_time_, desired_state_wall_time_;
  ros::WallTime last_zero_wall_time_, last_state_wall_time_;
  double state_interval_ros_ = 0.0;
  double state_interval_wall_ = 0.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_node");
  try {
    ControllerNode controller;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("Failed to initialize controller: %s", exception.what());
    return 1;
  }
  return 0;
}
