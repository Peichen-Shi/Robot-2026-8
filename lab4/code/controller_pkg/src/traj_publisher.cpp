#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <cmath>

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_publisher");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  bool static_pose = false;
  double radius = 5.0;
  double time_scale = 2.0;
  double height = 2.0;
  double publish_frequency = 500.0;
  private_node_handle.param("static_pose", static_pose, static_pose);
  private_node_handle.param("radius", radius, radius);
  private_node_handle.param("time_scale", time_scale, time_scale);
  private_node_handle.param("height", height, height);
  private_node_handle.param("publish_frequency", publish_frequency,
                            publish_frequency);

  if (radius <= 0.0 || time_scale <= 0.0 || height < 0.0 ||
      publish_frequency <= 0.0) {
    ROS_FATAL("Invalid trajectory parameter");
    return 1;
  }

  ros::Publisher desired_state_publisher =
      node_handle.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
          "desired_state", 1);
  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(publish_frequency);

  // Under /use_sim_time, ros::Time::now() remains zero until the simulator
  // publishes /clock. Waiting avoids a false wall/simulation-time offset.
  ros::Time::waitForValid();
  const ros::Time start_time = ros::Time::now();

  ROS_INFO_STREAM("Trajectory publisher initialized: mode="
                  << (static_pose ? "static" : "circle")
                  << ", radius=" << radius << " m, height=" << height
                  << " m, time_scale=" << time_scale
                  << " s, rate=" << publish_frequency << " Hz");

  while (ros::ok()) {
    const ros::Time stamp = ros::Time::now();
    const double elapsed = (stamp - start_time).toSec();

    tf::Transform desired_pose(tf::Transform::getIdentity());
    geometry_msgs::Twist velocity;
    geometry_msgs::Twist acceleration;
    tf::Quaternion quaternion;

    if (static_pose) {
      desired_pose.setOrigin(tf::Vector3(0.0, 0.0, height));
      quaternion.setRPY(0.0, 0.0, M_PI / 4.0);
    } else {
      const double phase = elapsed / time_scale;
      desired_pose.setOrigin(tf::Vector3(radius * std::sin(phase),
                                         radius * std::cos(phase), height));
      quaternion.setRPY(0.0, 0.0, -phase);

      velocity.linear.x = radius * std::cos(phase) / time_scale;
      velocity.linear.y = -radius * std::sin(phase) / time_scale;
      velocity.angular.z = -1.0 / time_scale;

      acceleration.linear.x =
          -radius * std::sin(phase) / (time_scale * time_scale);
      acceleration.linear.y =
          -radius * std::cos(phase) / (time_scale * time_scale);
    }
    quaternion.normalize();
    desired_pose.setRotation(quaternion);

    trajectory_msgs::MultiDOFJointTrajectoryPoint message;
    message.transforms.resize(1);
    message.transforms[0].translation.x = desired_pose.getOrigin().x();
    message.transforms[0].translation.y = desired_pose.getOrigin().y();
    message.transforms[0].translation.z = desired_pose.getOrigin().z();
    message.transforms[0].rotation.x = desired_pose.getRotation().x();
    message.transforms[0].rotation.y = desired_pose.getRotation().y();
    message.transforms[0].rotation.z = desired_pose.getRotation().z();
    message.transforms[0].rotation.w = desired_pose.getRotation().w();
    message.velocities.push_back(velocity);
    message.accelerations.push_back(acceleration);
    message.time_from_start = ros::Duration(elapsed);
    desired_state_publisher.publish(message);

    broadcaster.sendTransform(tf::StampedTransform(
        desired_pose, stamp, "world", "av-desired"));
    ROS_INFO_THROTTLE(2.0, "Desired position: x=%.3f y=%.3f z=%.3f",
                      desired_pose.getOrigin().x(),
                      desired_pose.getOrigin().y(),
                      desired_pose.getOrigin().z());

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
