#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Transform.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <cmath>
#include <fstream>
#include <map>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include <csv.h>

namespace {

struct IndexedGate {
  int index;
  std::string object_name;
  geometry_msgs::Pose pose;
};

geometry_msgs::Pose unity2RosPose(const geometry_msgs::Transform& unity_T) {
  geometry_msgs::Pose ros_T;
  ros_T.position.x = unity_T.translation.x;
  ros_T.position.y = unity_T.translation.z;
  ros_T.position.z = unity_T.translation.y;
  ros_T.orientation.x = -unity_T.rotation.x;
  ros_T.orientation.y = -unity_T.rotation.z;
  ros_T.orientation.z = -unity_T.rotation.y;
  ros_T.orientation.w = unity_T.rotation.w;
  return ros_T;
}

bool finitePose(const geometry_msgs::Pose& pose) {
  const auto& p = pose.position;
  const auto& q = pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w);
}

void normalizeQuaternion(geometry_msgs::Quaternion* q) {
  const double norm =
      std::sqrt(q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w);
  if (!std::isfinite(norm) || norm < 1e-12) {
    throw std::runtime_error("gate contains a zero or invalid quaternion");
  }
  q->x /= norm;
  q->y /= norm;
  q->z /= norm;
  q->w /= norm;
}

std::vector<IndexedGate> loadGates(const std::string& data_directory) {
  const std::string csv_path =
      data_directory + "/StreamingAssets/vnav-2020-lab4_static_tfs.csv";
  io::CSVReader<8> input(csv_path);
  input.set_header("object_name", "x", "y", "z", "qx", "qy", "qz", "qw");

  const std::regex gate_pattern(
      "^red_square_drone_door(?: \\(([0-9]+)\\))?$");
  std::string object_name;
  double x, y, z, qx, qy, qz, qw;
  std::map<int, IndexedGate> gates;
  while (input.read_row(object_name, x, y, z, qx, qy, qz, qw)) {
    std::smatch match;
    if (!std::regex_match(object_name, match, gate_pattern)) continue;
    const int index = match[1].matched ? std::stoi(match[1].str()) : 0;
    if (gates.count(index) != 0u) {
      throw std::runtime_error("duplicate gate index " + std::to_string(index));
    }

    geometry_msgs::Transform unity_transform;
    unity_transform.translation.x = x;
    unity_transform.translation.y = y;
    unity_transform.translation.z = z;
    unity_transform.rotation.x = qx;
    unity_transform.rotation.y = qy;
    unity_transform.rotation.z = qz;
    unity_transform.rotation.w = qw;
    IndexedGate gate{index, object_name, unity2RosPose(unity_transform)};
    if (!finitePose(gate.pose)) {
      throw std::runtime_error("non-finite gate pose for " + object_name);
    }
    normalizeQuaternion(&gate.pose.orientation);
    gates.emplace(index, gate);
  }
  if (gates.empty()) {
    throw std::runtime_error("no race gates found in " + csv_path);
  }

  std::vector<IndexedGate> output;
  output.reserve(gates.size());
  for (const auto& item : gates) output.push_back(item.second);
  return output;
}

void writeGateTable(const std::string& output_path,
                    const std::vector<IndexedGate>& gates) {
  if (output_path.empty()) return;
  std::ofstream output(output_path);
  if (!output) {
    throw std::runtime_error("cannot open gate table output " + output_path);
  }
  output << "gate_index,object_name,x,y,z,qx,qy,qz,qw,yaw_raw_rad,"
            "yaw_unwrapped_rad\n";
  output.precision(17);
  double previous_yaw = 0.0;
  bool have_previous_yaw = false;
  for (const auto& gate : gates) {
    const auto& p = gate.pose.position;
    const auto& q = gate.pose.orientation;
    tf2::Quaternion quaternion;
    tf2::fromMsg(q, quaternion);
    quaternion.normalize();
    const double yaw_raw = tf2::getYaw(quaternion);
    const double yaw_unwrapped = have_previous_yaw
        ? previous_yaw + std::atan2(std::sin(yaw_raw - previous_yaw),
                                    std::cos(yaw_raw - previous_yaw))
        : yaw_raw;
    previous_yaw = yaw_unwrapped;
    have_previous_yaw = true;
    output << gate.index << ",\"" << gate.object_name << "\"," << p.x
           << ',' << p.y << ',' << p.z << ',' << q.x << ',' << q.y << ','
           << q.z << ',' << q.w << ',' << yaw_raw << ',' << yaw_unwrapped
           << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_vertices_publisher");
  ros::NodeHandle nh("~");
  std::string data_directory;
  std::string gates_output_csv;
  std::string frame_id;
  double publish_frequency;
  nh.param("simulator_data_directory", data_directory, std::string());
  nh.param("gates_output_csv", gates_output_csv, std::string());
  nh.param("frame_id", frame_id, std::string("world"));
  nh.param("publish_frequency", publish_frequency, 5.0);
  if (data_directory.empty() || publish_frequency <= 0.0) {
    ROS_FATAL("simulator_data_directory and positive publish_frequency are required");
    return 1;
  }

  std::vector<IndexedGate> gates;
  try {
    gates = loadGates(data_directory);
    writeGateTable(gates_output_csv, gates);
  } catch (const std::exception& exception) {
    ROS_FATAL("Failed to load LAB4 gates: %s", exception.what());
    return 1;
  }

  geometry_msgs::PoseArray message;
  message.header.frame_id = frame_id;
  for (const auto& gate : gates) message.poses.push_back(gate.pose);
  ros::Publisher publisher = nh.advertise<geometry_msgs::PoseArray>(
      "/desired_traj_vertices", 1, true);
  ros::Rate rate(publish_frequency);
  ROS_INFO("Loaded %zu numerically sorted LAB4 gates", gates.size());
  while (ros::ok()) {
    message.header.stamp = ros::Time::now();
    publisher.publish(message);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
