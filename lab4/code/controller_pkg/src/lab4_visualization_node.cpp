#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_generation/TrajectoryStatus.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <sstream>
#include <string>

namespace {

std_msgs::ColorRGBA color(double red, double green, double blue,
                          double alpha = 1.0) {
  std_msgs::ColorRGBA output;
  output.r = red;
  output.g = green;
  output.b = blue;
  output.a = alpha;
  return output;
}

double squaredDistance(const geometry_msgs::Point& a,
                       const geometry_msgs::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

}  // namespace

class Lab4VisualizationNode {
 public:
  Lab4VisualizationNode()
      : private_nh_("~"),
        max_points_(2500),
        sample_period_(0.05),
        minimum_point_distance_(0.01),
        publish_rate_(20.0),
        have_current_(false),
        have_desired_(false),
        have_status_(false),
        delete_all_pending_(true),
        last_desired_elapsed_(-1.0),
        passed_gates_(0),
        current_speed_(0.0) {
    private_nh_.param("fixed_frame", fixed_frame_, std::string("world"));
    private_nh_.param("source_label", source_label_,
                      std::string("Live TESSE messages"));
    private_nh_.param("max_points", max_points_, max_points_);
    private_nh_.param("sample_period", sample_period_, sample_period_);
    private_nh_.param("minimum_point_distance", minimum_point_distance_,
                      minimum_point_distance_);
    private_nh_.param("publish_rate", publish_rate_, publish_rate_);
    private_nh_.param("gate_width", gate_width_, 2.4);
    private_nh_.param("gate_height", gate_height_, 2.4);
    private_nh_.param("gate_beam_thickness", gate_beam_thickness_, 0.14);

    max_points_ = std::max(100, max_points_);
    sample_period_ = std::max(0.01, sample_period_);
    minimum_point_distance_ = std::max(0.0, minimum_point_distance_);
    publish_rate_ = std::max(1.0, publish_rate_);

    marker_publisher_ = private_nh_.advertise<visualization_msgs::MarkerArray>(
        "markers", 1, true);
    current_subscriber_ = nh_.subscribe(
        "current_state", 10, &Lab4VisualizationNode::onCurrentState, this);
    desired_subscriber_ = nh_.subscribe(
        "desired_state", 10, &Lab4VisualizationNode::onDesiredState, this);
    gates_subscriber_ = nh_.subscribe(
        "desired_traj_vertices", 1, &Lab4VisualizationNode::onGates, this);
    status_subscriber_ = nh_.subscribe(
        "trajectory_status", 10, &Lab4VisualizationNode::onStatus, this);
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                     &Lab4VisualizationNode::publish, this);

    ROS_INFO_STREAM("LAB4 visualization initialized: source='"
                    << source_label_ << "', max_points=" << max_points_
                    << ", sample_period=" << sample_period_ << " s");
  }

 private:
  void resetHistory(const std::string& reason) {
    passed_gates_ = 0;
    have_current_ = false;  // do not connect the previous endpoint to a new start
    last_desired_elapsed_ = -1.0;
    actual_path_.clear();
    desired_path_.clear();
    last_actual_sample_stamp_ = ros::Time();
    last_desired_sample_elapsed_ = -1.0;
    delete_all_pending_ = true;
    ROS_WARN_STREAM("Visualization history cleared: " << reason);
  }

  bool shouldSample(const geometry_msgs::Point& point, const ros::Time& stamp,
                    const std::deque<geometry_msgs::Point>& path,
                    const ros::Time& last_stamp) const {
    if (path.empty() || last_stamp.isZero()) {
      return true;
    }
    if ((stamp - last_stamp).toSec() < sample_period_) {
      return false;
    }
    return squaredDistance(point, path.back()) >=
           minimum_point_distance_ * minimum_point_distance_;
  }

  void appendPoint(std::deque<geometry_msgs::Point>* path,
                   const geometry_msgs::Point& point) {
    path->push_back(point);
    while (static_cast<int>(path->size()) > max_points_) {
      path->pop_front();
    }
  }

  void onCurrentState(const nav_msgs::Odometry& message) {
    ros::Time stamp = message.header.stamp;
    if (stamp.isZero()) {
      stamp = ros::Time::now();
    }
    if (!last_current_stamp_.isZero() && stamp + ros::Duration(1e-6) <
                                               last_current_stamp_) {
      resetHistory("current_state timestamp moved backwards");
    }
    last_current_stamp_ = stamp;
    current_pose_ = message.pose.pose;
    current_speed_ = std::sqrt(
        message.twist.twist.linear.x * message.twist.twist.linear.x +
        message.twist.twist.linear.y * message.twist.twist.linear.y +
        message.twist.twist.linear.z * message.twist.twist.linear.z);
    if (have_current_) {
      updateGateCount(current_pose_.position);
    }
    previous_position_ = current_pose_.position;
    have_current_ = true;

    if (shouldSample(current_pose_.position, stamp, actual_path_,
                     last_actual_sample_stamp_)) {
      appendPoint(&actual_path_, current_pose_.position);
      last_actual_sample_stamp_ = stamp;
    }
  }

  void onDesiredState(
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& message) {
    if (message.transforms.empty()) {
      ROS_WARN_THROTTLE(1.0,
                        "Visualization ignored empty desired_state message");
      return;
    }
    const double elapsed = message.time_from_start.toSec();
    if (last_desired_elapsed_ >= 0.0 && elapsed + 1e-6 <
                                            last_desired_elapsed_) {
      resetHistory("desired_state time_from_start moved backwards");
    }
    last_desired_elapsed_ = elapsed;

    const geometry_msgs::Transform& transform = message.transforms.front();
    desired_point_.x = transform.translation.x;
    desired_point_.y = transform.translation.y;
    desired_point_.z = transform.translation.z;
    have_desired_ = true;

    if (desired_path_.empty() || last_desired_sample_elapsed_ < 0.0 ||
        elapsed - last_desired_sample_elapsed_ >= sample_period_) {
      if (desired_path_.empty() ||
          squaredDistance(desired_point_, desired_path_.back()) >=
              minimum_point_distance_ * minimum_point_distance_) {
        appendPoint(&desired_path_, desired_point_);
      }
      last_desired_sample_elapsed_ = elapsed;
    }
  }

  void onGates(const geometry_msgs::PoseArray& message) {
    if (message.poses.empty()) return;
    gates_ = message;
  }

  void onStatus(const trajectory_generation::TrajectoryStatus& message) {
    if (have_status_ && (message.trajectory_id != trajectory_id_ || message.run_id != run_id_)) {
      passed_gates_ = 0;
      resetHistory("new trajectory id");
    }
    have_status_ = true;
    trajectory_id_ = message.trajectory_id;
    run_id_ = message.run_id;
    phase_ = message.phase;
    mission_elapsed_ = message.mission_elapsed_s;
    trajectory_elapsed_ = message.trajectory_elapsed_s;
    sample_tau_ = message.sample_tau_s;
    planned_duration_ = message.planned_duration_s;
  }

  void updateGateCount(const geometry_msgs::Point& current) {
    if (passed_gates_ >= gates_.poses.size()) return;
    const geometry_msgs::Pose& gate = gates_.poses[passed_gates_];
    tf2::Quaternion q;
    tf2::fromMsg(gate.orientation, q);
    q.normalize();
    const tf2::Quaternion inverse = q.inverse();
    const tf2::Vector3 before = tf2::quatRotate(
        inverse, tf2::Vector3(previous_position_.x - gate.position.x,
                              previous_position_.y - gate.position.y,
                              previous_position_.z - gate.position.z));
    const tf2::Vector3 after = tf2::quatRotate(
        inverse, tf2::Vector3(current.x - gate.position.x,
                              current.y - gate.position.y,
                              current.z - gate.position.z));
    if (before.x() < 0.0 && after.x() >= 0.0 && after.x() > before.x()) {
      const double alpha = -before.x() / (after.x() - before.x());
      const double y = before.y() + alpha * (after.y() - before.y());
      const double z = before.z() + alpha * (after.z() - before.z());
      const double half_width = 0.5 * gate_width_ - 0.18;
      const double half_height = 0.5 * gate_height_ - 0.18;
      if (std::abs(y) <= half_width && std::abs(z) <= half_height) {
        ++passed_gates_;
      }
    }
  }

  visualization_msgs::Marker marker(int id, int type,
                                    const std::string& marker_namespace,
                                    const ros::Time& stamp) const {
    visualization_msgs::Marker output;
    output.header.frame_id = fixed_frame_;
    output.header.stamp = stamp;
    output.ns = marker_namespace;
    output.id = id;
    output.type = type;
    output.action = visualization_msgs::Marker::ADD;
    output.pose.orientation.w = 1.0;
    output.lifetime = ros::Duration(0.0);
    output.frame_locked = false;
    return output;
  }

  geometry_msgs::Pose bodyPose(double x, double y, double z,
                               double local_yaw = 0.0) const {
    geometry_msgs::Pose output = current_pose_;
    tf2::Quaternion orientation;
    tf2::fromMsg(current_pose_.orientation, orientation);
    orientation.normalize();
    const tf2::Vector3 offset =
        tf2::quatRotate(orientation, tf2::Vector3(x, y, z));
    output.position.x += offset.x();
    output.position.y += offset.y();
    output.position.z += offset.z();
    if (std::abs(local_yaw) > 1e-12) {
      tf2::Quaternion local_rotation;
      local_rotation.setRPY(0.0, 0.0, local_yaw);
      output.orientation = tf2::toMsg(orientation * local_rotation);
    } else {
      output.orientation = tf2::toMsg(orientation);
    }
    return output;
  }

  geometry_msgs::Pose offsetPose(const geometry_msgs::Pose& source,
                                 double x, double y, double z) const {
    geometry_msgs::Pose output = source;
    tf2::Quaternion orientation;
    tf2::fromMsg(source.orientation, orientation);
    orientation.normalize();
    const tf2::Vector3 offset =
        tf2::quatRotate(orientation, tf2::Vector3(x, y, z));
    output.position.x += offset.x();
    output.position.y += offset.y();
    output.position.z += offset.z();
    output.orientation = tf2::toMsg(orientation);
    return output;
  }

  void addGates(visualization_msgs::MarkerArray* output,
                const ros::Time& stamp) const {
    for (size_t gate_index = 0; gate_index < gates_.poses.size(); ++gate_index) {
      const geometry_msgs::Pose& gate_pose = gates_.poses[gate_index];
      for (int beam_index = 0; beam_index < 4; ++beam_index) {
        visualization_msgs::Marker beam = marker(
            static_cast<int>(gate_index * 10 + beam_index),
            visualization_msgs::Marker::CUBE, "race_gates", stamp);
        if (beam_index < 2) {
          const double z = (beam_index == 0 ? -0.5 : 0.5) * gate_height_;
          beam.pose = offsetPose(gate_pose, 0.0, 0.0, z);
          beam.scale.x = gate_beam_thickness_;
          beam.scale.y = gate_width_ + gate_beam_thickness_;
          beam.scale.z = gate_beam_thickness_;
        } else {
          const double y = (beam_index == 2 ? -0.5 : 0.5) * gate_width_;
          beam.pose = offsetPose(gate_pose, 0.0, y, 0.0);
          beam.scale.x = gate_beam_thickness_;
          beam.scale.y = gate_beam_thickness_;
          beam.scale.z = gate_height_ + gate_beam_thickness_;
        }
        beam.color = color(0.86, 0.16, 0.14, 0.88);
        output->markers.push_back(beam);
      }
      visualization_msgs::Marker label = marker(
          static_cast<int>(gate_index),
          visualization_msgs::Marker::TEXT_VIEW_FACING, "gate_numbers", stamp);
      label.pose = offsetPose(gate_pose, 0.0, 0.0,
                              0.5 * gate_height_ + 0.45);
      label.scale.z = 0.68;
      label.color = color(1.0, 0.92, 0.30);
      label.text = "Gate " + std::to_string(gate_index);
      output->markers.push_back(label);
    }
  }

  void addDrone(visualization_msgs::MarkerArray* output,
                const ros::Time& stamp) const {
    if (!have_current_) {
      return;
    }

    visualization_msgs::Marker body =
        marker(0, visualization_msgs::Marker::CUBE, "quadrotor", stamp);
    body.pose = bodyPose(0.0, 0.0, 0.0);
    body.scale.x = 0.30;
    body.scale.y = 0.21;
    body.scale.z = 0.11;
    body.color = color(0.12, 0.15, 0.19);
    output->markers.push_back(body);

    for (int arm_index = 0; arm_index < 2; ++arm_index) {
      visualization_msgs::Marker arm =
          marker(1 + arm_index, visualization_msgs::Marker::CUBE,
                 "quadrotor", stamp);
      arm.pose = bodyPose(0.0, 0.0, 0.0,
                          arm_index == 0 ? M_PI / 4.0 : -M_PI / 4.0);
      arm.scale.x = 0.90;
      arm.scale.y = 0.045;
      arm.scale.z = 0.032;
      arm.color = color(0.33, 0.37, 0.42);
      output->markers.push_back(arm);
    }

    const double rotor_offset = 0.32;
    const double signs[4][2] = {{1.0, 1.0}, {1.0, -1.0},
                                {-1.0, 1.0}, {-1.0, -1.0}};
    for (int index = 0; index < 4; ++index) {
      visualization_msgs::Marker rotor =
          marker(10 + index, visualization_msgs::Marker::CYLINDER,
                 "quadrotor", stamp);
      rotor.pose = bodyPose(signs[index][0] * rotor_offset,
                            signs[index][1] * rotor_offset, 0.025);
      rotor.scale.x = 0.27;
      rotor.scale.y = 0.27;
      rotor.scale.z = 0.022;
      rotor.color = (index < 2) ? color(0.0, 0.82, 0.95, 0.92)
                                : color(0.95, 0.48, 0.08, 0.92);
      output->markers.push_back(rotor);
    }

    visualization_msgs::Marker nose =
        marker(20, visualization_msgs::Marker::SPHERE, "quadrotor", stamp);
    nose.pose = bodyPose(0.20, 0.0, 0.025);
    nose.scale.x = 0.09;
    nose.scale.y = 0.09;
    nose.scale.z = 0.09;
    nose.color = color(1.0, 0.86, 0.14);
    output->markers.push_back(nose);

    visualization_msgs::Marker locator =
        marker(21, visualization_msgs::Marker::TEXT_VIEW_FACING, "quadrotor", stamp);
    locator.pose.position = current_pose_.position;
    locator.pose.position.z += 0.8;
    locator.scale.z = 0.65;
    locator.color = color(0.95, 0.97, 1.0);
    locator.text = "UAV";
    output->markers.push_back(locator);

    tf2::Quaternion orientation;
    tf2::fromMsg(current_pose_.orientation, orientation);
    orientation.normalize();
    const tf2::Vector3 directions[3] = {
        tf2::quatRotate(orientation, tf2::Vector3(0.32, 0.0, 0.0)),
        tf2::quatRotate(orientation, tf2::Vector3(0.0, 0.32, 0.0)),
        tf2::quatRotate(orientation, tf2::Vector3(0.0, 0.0, 0.32))};
    const std_msgs::ColorRGBA axis_colors[3] = {
        color(0.90, 0.12, 0.12), color(0.10, 0.78, 0.18),
        color(0.12, 0.36, 0.95)};
    for (int index = 0; index < 3; ++index) {
      visualization_msgs::Marker axis =
          marker(30 + index, visualization_msgs::Marker::ARROW, "body_axes",
                 stamp);
      axis.points.resize(2);
      axis.points[0] = current_pose_.position;
      axis.points[1].x = current_pose_.position.x + directions[index].x();
      axis.points[1].y = current_pose_.position.y + directions[index].y();
      axis.points[1].z = current_pose_.position.z + directions[index].z();
      axis.scale.x = 0.018;
      axis.scale.y = 0.045;
      axis.scale.z = 0.06;
      axis.color = axis_colors[index];
      output->markers.push_back(axis);
    }
  }

  void addPath(visualization_msgs::MarkerArray* output,
               const std::deque<geometry_msgs::Point>& points, int id,
               const std::string& marker_namespace,
               const std_msgs::ColorRGBA& marker_color, double width,
               const ros::Time& stamp) const {
    if (points.size() < 2) {
      return;
    }
    visualization_msgs::Marker path =
        marker(id, visualization_msgs::Marker::LINE_STRIP, marker_namespace,
               stamp);
    path.scale.x = width;
    path.color = marker_color;
    path.points.assign(points.begin(), points.end());
    output->markers.push_back(path);
  }

  void addLegend(visualization_msgs::MarkerArray* output,
                 const ros::Time& stamp) const {
    // Place the billboard in empty space above/below the course projection,
    // rather than stacking labels over the final gates in the far corner.
    const double anchor_x = -8.0;
    const double anchor_y = -12.0;
    const double anchor_z = 18.0;
    const std_msgs::ColorRGBA cyan = color(0.0, 0.86, 1.0);
    const std_msgs::ColorRGBA orange = color(1.0, 0.43, 0.05);

    for (int index = 0; index < 2; ++index) {
      visualization_msgs::Marker sample =
          marker(100 + index, visualization_msgs::Marker::LINE_STRIP,
                 "legend", stamp);
      sample.scale.x = 0.055;
      sample.color = index == 0 ? cyan : orange;
      geometry_msgs::Point start;
      start.x = anchor_x;
      start.y = anchor_y - 1.40 * index;
      start.z = anchor_z - 2.40 * index;
      geometry_msgs::Point end = start;
      end.x += 2.4;
      sample.points.push_back(start);
      sample.points.push_back(end);

      visualization_msgs::Marker label =
          marker(110 + index, visualization_msgs::Marker::TEXT_VIEW_FACING,
                 "legend", stamp);
      label.pose.position = start;
      label.scale.z = 0.90;
      label.color = index == 0 ? cyan : orange;
      label.text = index == 0 ? "Desired trajectory" : "Actual trajectory";
      output->markers.push_back(label);
    }

    visualization_msgs::Marker source =
        marker(120, visualization_msgs::Marker::TEXT_VIEW_FACING, "legend",
               stamp);
    source.pose.position.x = anchor_x;
    source.pose.position.y = anchor_y + 1.50;
    source.pose.position.z = anchor_z + 2.40;
    source.scale.z = 0.70;
    source.color = color(0.72, 0.76, 0.82);
    source.text = source_label_;
    output->markers.push_back(source);

    if (have_current_ && have_desired_) {
      const double error =
          std::sqrt(squaredDistance(current_pose_.position, desired_point_));
      std::ostringstream text;
      text << std::fixed << std::setprecision(3) << "Error " << error
           << " m   speed " << current_speed_ << " m/s\nGates "
           << passed_gates_ << "/" << gates_.poses.size();
      if (have_status_) {
        text << "  " << run_id_ << "  " << phase_ << "\ntau " << std::setprecision(1)
             << sample_tau_ << "/" << planned_duration_
             << " s   elapsed " << trajectory_elapsed_ << " s";
      }
      visualization_msgs::Marker status =
          marker(121, visualization_msgs::Marker::TEXT_VIEW_FACING, "legend",
                 stamp);
      status.pose.position.x = anchor_x;
      status.pose.position.y = anchor_y - 3.80;
      status.pose.position.z = anchor_z - 6.00;
      status.scale.z = 0.70;
      status.color = error <= 0.5 ? color(0.46, 0.93, 0.58)
                                  : color(1.0, 0.68, 0.20);
      status.text = text.str();
      output->markers.push_back(status);
    }
  }

  void publish(const ros::TimerEvent&) {
    const ros::Time stamp = ros::Time::now();
    visualization_msgs::MarkerArray output;
    if (delete_all_pending_) {
      visualization_msgs::Marker clear;
      clear.header.frame_id = fixed_frame_;
      clear.header.stamp = stamp;
      clear.action = visualization_msgs::Marker::DELETEALL;
      output.markers.push_back(clear);
      delete_all_pending_ = false;
    }

    addGates(&output, stamp);
    addPath(&output, desired_path_, 0, "desired_path",
            color(0.0, 0.86, 1.0), 0.035, stamp);
    addPath(&output, actual_path_, 0, "actual_path",
            color(1.0, 0.43, 0.05), 0.045, stamp);

    if (have_desired_) {
      visualization_msgs::Marker desired = marker(
          0, visualization_msgs::Marker::SPHERE, "desired_setpoint", stamp);
      desired.pose.position = desired_point_;
      desired.scale.x = 0.10;
      desired.scale.y = 0.10;
      desired.scale.z = 0.10;
      desired.color = color(0.0, 0.86, 1.0);
      output.markers.push_back(desired);
    }

    addDrone(&output, stamp);
    addLegend(&output, stamp);
    marker_publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber current_subscriber_;
  ros::Subscriber desired_subscriber_;
  ros::Subscriber gates_subscriber_;
  ros::Subscriber status_subscriber_;
  ros::Publisher marker_publisher_;
  ros::Timer publish_timer_;

  std::string fixed_frame_;
  std::string source_label_;
  int max_points_;
  double sample_period_;
  double minimum_point_distance_;
  double publish_rate_;
  double gate_width_;
  double gate_height_;
  double gate_beam_thickness_;

  bool have_current_;
  bool have_desired_;
  bool have_status_;
  bool delete_all_pending_;
  geometry_msgs::Pose current_pose_;
  geometry_msgs::Point desired_point_;
  geometry_msgs::Point previous_position_;
  geometry_msgs::PoseArray gates_;
  std::deque<geometry_msgs::Point> actual_path_;
  std::deque<geometry_msgs::Point> desired_path_;
  ros::Time last_current_stamp_;
  ros::Time last_actual_sample_stamp_;
  double last_desired_elapsed_;
  double last_desired_sample_elapsed_ = -1.0;
  size_t passed_gates_;
  double current_speed_;
  uint32_t trajectory_id_ = 0;
  std::string run_id_;
  double sample_tau_ = 0.0;
  std::string phase_ = "WAITING";
  double mission_elapsed_ = 0.0;
  double trajectory_elapsed_ = 0.0;
  double planned_duration_ = 0.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "lab4_visualization");
  Lab4VisualizationNode node;
  ros::spin();
  return 0;
}
