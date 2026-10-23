#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <cmath>
#include <string>

namespace {

ros::Publisher desired_state_pub;
tf::TransformBroadcaster* broadcaster = nullptr;

bool finitePose(const geometry_msgs::Pose& pose) {
  const geometry_msgs::Point& p = pose.position;
  const geometry_msgs::Quaternion& q = pose.orientation;
  const double q_norm_sq =
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q_norm_sq) && q_norm_sq > 1e-12;
}

geometry_msgs::Quaternion normalized(
    const geometry_msgs::Quaternion& quaternion) {
  const double norm =
      std::sqrt(quaternion.x * quaternion.x +
                quaternion.y * quaternion.y +
                quaternion.z * quaternion.z +
                quaternion.w * quaternion.w);
  geometry_msgs::Quaternion output = quaternion;
  output.x /= norm;
  output.y /= norm;
  output.z /= norm;
  output.w /= norm;
  return output;
}

void trajCB(const geometry_msgs::PoseArray& traj_msg) {
  if (traj_msg.poses.empty()) {
    ROS_ERROR_THROTTLE(1.0, "Rejected empty trajectory vertices message");
    return;
  }
  const geometry_msgs::Pose& first_gate = traj_msg.poses.front();
  if (!finitePose(first_gate)) {
    ROS_ERROR_THROTTLE(1.0, "Rejected invalid first-gate pose");
    return;
  }

  trajectory_msgs::MultiDOFJointTrajectoryPoint desired;
  desired.transforms.resize(1);
  desired.velocities.resize(1);
  desired.accelerations.resize(1);
  desired.transforms.front().translation.x = first_gate.position.x;
  desired.transforms.front().translation.y = first_gate.position.y;
  desired.transforms.front().translation.z = first_gate.position.z;
  desired.transforms.front().rotation = normalized(first_gate.orientation);
  desired.time_from_start = ros::Duration(0.0);
  desired_state_pub.publish(desired);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(first_gate.position.x,
                                  first_gate.position.y,
                                  first_gate.position.z));
  const geometry_msgs::Quaternion& q = desired.transforms.front().rotation;
  transform.setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));
  const std::string parent = traj_msg.header.frame_id.empty()
                                 ? "world"
                                 : traj_msg.header.frame_id;
  broadcaster->sendTransform(tf::StampedTransform(
      transform, ros::Time::now(), parent, "first_gate_desired"));
  ROS_INFO_THROTTLE(2.0,
                    "Publishing first-gate hold reference at [%.3f %.3f %.3f]",
                    first_gate.position.x, first_gate.position.y,
                    first_gate.position.z);
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_traj_planner");
  ros::NodeHandle nh;
  ros::Subscriber vertices_sub =
      nh.subscribe("desired_traj_vertices", 10, trajCB);
  desired_state_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
          "desired_state", 1);
  tf::TransformBroadcaster tf_broadcaster;
  broadcaster = &tf_broadcaster;
  ros::spin();
  return 0;
}
