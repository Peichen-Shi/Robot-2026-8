// VNAV Lab6: ordered ROS bag evaluation and replayable PoseStamped output.
// Camera transforms always map current-camera coordinates into previous-camera coordinates.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include "sift_feature_tracker.h"

namespace {
using Pose = Eigen::Isometry3d;
using Mono = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
using Two = opengv::sac_problems::relative_pose::TranslationOnlySacProblem;
using Cloud = opengv::sac_problems::point_cloud::PointCloudSacProblem;
using Image = sensor_msgs::ImageConstPtr;
const std::string kRgb = "/tesse/left_cam/rgb/image_raw";
const std::string kDepth = "/tesse/depth_cam/mono/image_raw";
const std::string kInfo = "/tesse/left_cam/camera_info";
const std::array<std::string, 5> kMethods = {"NISTER5", "EIGHT8", "KNOWN2", "ARUN3", "NISTER5_RAW"};

struct Frame {
  Image rgb, depth;
  ros::Time bag_time;
};
struct Estimate {
  opengv::transformation_t model = opengv::transformation_t::Zero();
  std::vector<int> inliers;
  bool valid = false;
};
struct Features {
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
};

// Reuse Lab5 SIFT + FLANN + ratio test. Lab6 performs its own model-specific RANSAC.
class SiftMatcher : public SiftFeatureTracker {
 public:
  Features extract(const cv::Mat& image) const {
    Features result;
    detectKeypoints(image, &result.keypoints);
    describeKeypoints(image, &result.keypoints, &result.descriptors);
    return result;
  }
  std::vector<cv::DMatch> match(const Features& a, const Features& b) const {
    std::vector<std::vector<cv::DMatch>> raw;
    std::vector<cv::DMatch> good;
    matchDescriptors(a.descriptors, b.descriptors, &raw, &good);
    return good;
  }
};

opengv::bearingVectors_t calibrateKeypoints(const std::vector<cv::Point2f>& points,
                                          const cv::Mat& k, const cv::Mat& d) {
  opengv::bearingVectors_t rays;
  if (points.empty()) return rays;
  std::vector<cv::Point2f> normalized;
  cv::undistortPoints(points, normalized, k, d);
  for (const auto& p : normalized) rays.emplace_back(Eigen::Vector3d(p.x, p.y, 1).normalized());
  return rays;
}

Pose cameraPose(const nav_msgs::Odometry& message) {
  const auto& p = message.pose.pose;
  Pose body = Pose::Identity();
  body.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                    p.orientation.y, p.orientation.z).normalized().toRotationMatrix();
  body.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  // TESSE starter extrinsic: camera right/down/forward -> body right-handed axes.
  Pose extrinsic = Pose::Identity();
  extrinsic.linear() << 0,0,1, -1,0,0, 0,-1,0;
  extrinsic.translation() = Eigen::Vector3d(0, 0.05, 0);
  return body * extrinsic;
}

bool interpolatePose(const std::vector<nav_msgs::OdometryConstPtr>& poses,
                     const ros::Time& time, Pose& result) {
  const auto hi = std::lower_bound(poses.begin(), poses.end(), time,
      [](const auto& p, const ros::Time& t) { return p->header.stamp < t; });
  if (hi == poses.end()) return false;
  if ((*hi)->header.stamp == time) {
    result = cameraPose(**hi);
    return true;
  }
  if (hi == poses.begin()) return false;
  const auto lo = hi - 1;
  const double dt = ((*hi)->header.stamp - (*lo)->header.stamp).toSec();
  if (dt <= 0 || dt > 0.2) return false;
  const double a = (time - (*lo)->header.stamp).toSec() / dt;
  const Pose first = cameraPose(**lo), second = cameraPose(**hi);
  result.linear() = Eigen::Quaterniond(first.linear()).slerp(a,
      Eigen::Quaterniond(second.linear())).toRotationMatrix();
  result.translation() = (1 - a) * first.translation() + a * second.translation();
  return true;
}

double depthAt(const cv::Mat& depth, const cv::Point2f& pixel) {
  const int x = std::floor(pixel.x), y = std::floor(pixel.y);
  if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) return 0;
  if (depth.type() == CV_32FC1) return depth.at<float>(y, x);
  if (depth.type() == CV_16UC1) return depth.at<uint16_t>(y, x) * 0.001;
  return 0;
}

template <typename Problem>
Estimate consensus(const std::shared_ptr<Problem>& problem, double threshold, size_t count) {
  Estimate estimate;
  if (count < static_cast<size_t>(problem->getSampleSize())) return estimate;
  opengv::sac::Ransac<Problem> ransac(300, threshold, 0.99);
  ransac.sac_model_ = problem;
  if (!ransac.computeModel() || ransac.inliers_.size() <
      static_cast<size_t>(problem->getSampleSize())) return estimate;
  estimate.model = ransac.model_coefficients_;
  estimate.inliers = ransac.inliers_;
  estimate.valid = estimate.model.allFinite();
  return estimate;
}

geometry_msgs::PoseStamped stampedPose(const Pose& transform, const ros::Time& time) {
  geometry_msgs::PoseStamped p;
  p.header.stamp = time;
  p.header.frame_id = "world";
  const Eigen::Quaterniond q(transform.linear());
  p.pose.position.x = transform.translation().x();
  p.pose.position.y = transform.translation().y();
  p.pose.position.z = transform.translation().z();
  p.pose.orientation.x = q.x(); p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z(); p.pose.orientation.w = q.w();
  return p;
}

struct Rpe { double chordal, degrees, translation; };
Rpe evaluateRPE(const Pose& gt, const Pose& est, bool metric) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double chordal = (gt.linear() - est.linear()).norm();
  const double degrees = Eigen::AngleAxisd(gt.linear().transpose() * est.linear()).angle()
                         * 180 / std::acos(-1.0);
  double translation = nan;
  if (metric) translation = (gt.translation() - est.translation()).norm();
  else if (gt.translation().norm() >= 0.001 && est.translation().norm() > 1e-9)
    translation = (gt.translation().normalized() - est.translation().normalized()).norm();
  return {chordal, degrees, translation};
}

class Evaluation {
 public:
  Evaluation(const std::string& output, const cv::Mat& k, const cv::Mat& d)
      : output_(output), k_(k), d_(d), csv_(output + "/rpe.csv") {
    if (!csv_) throw std::runtime_error("Create output directory before running");
    csv_ << "pair,time_s,method,matches,inliers,status,rotation_frobenius,rotation_deg,"
            "translation_rpe,gt_baseline_m,solve_ms\n" << std::setprecision(10);
    for (size_t i=0;i<kMethods.size();++i)
      bags_[i].open(output + "/" + kMethods[i] + "_poses.bag", rosbag::bagmode::Write);
  }

  void cameraCallback(const Frame& frame, const Pose& gt) {
    const auto color = cv_bridge::toCvCopy(frame.rgb, "bgr8")->image;
    const auto depth = frame.depth ? cv_bridge::toCvCopy(frame.depth)->image : cv::Mat();
    const auto features = matcher_.extract(color);
    if (!previous_.empty()) {
      const Pose truth = previous_gt_.inverse() * gt;
      const auto matches = matcher_.match(previous_features_, features);
      std::vector<cv::Point2f> p1, p2;
      for (const auto& m : matches) {
        p1.push_back(previous_features_.keypoints[m.queryIdx].pt);
        p2.push_back(features.keypoints[m.trainIdx].pt);
      }
      const auto b1 = calibrateKeypoints(p1, k_, d_), b2 = calibrateKeypoints(p2, k_, d_);
      opengv::points_t cloud1, cloud2;
      std::vector<int> cloud_indices;
      if (previous_depth_.size() == previous_.size() && depth.size() == color.size()) {
        for (size_t i=0;i<p1.size();++i) {
          const double z1=depthAt(previous_depth_,p1[i]), z2=depthAt(depth,p2[i]);
          if (!(z1 > 0.05 && z1 < 200 && z2 > 0.05 && z2 < 200)) continue;
          cloud1.push_back((z1 / b1[i].z()) * b1[i]);
          cloud2.push_back((z2 / b2[i].z()) * b2[i]);
          cloud_indices.push_back(i);
        }
      }
      for (size_t i=0;i<kMethods.size();++i) {
        const auto start = std::chrono::steady_clock::now();
        Estimate est;
        const bool metric = i == 3;
        opengv::relative_pose::CentralRelativeAdapter mono(b1, b2);
        if (metric) {
          opengv::point_cloud::PointCloudAdapter adapter(cloud1, cloud2);
          auto problem = std::make_shared<Cloud>(adapter, false);
          est = consensus(problem, 0.04, cloud1.size());
          if (est.valid && est.inliers.size() >= 3) {
            opengv::transformation_t refined;
            problem->optimizeModelCoefficients(est.inliers, est.model, refined);
            if (refined.allFinite()) est.model = refined;
          }
        } else if (i == 2) {
          mono.setR12(truth.linear());
          est = consensus(std::make_shared<Two>(mono, false), 5e-6, b1.size());
        } else {
          auto problem = std::make_shared<Mono>(mono, i==1 ? Mono::EIGHTPT : Mono::NISTER, false);
          if (i==4) {
            // One deterministic hypothesis using the best descriptor matches, no RANSAC.
            if (matches.size() >= static_cast<size_t>(problem->getSampleSize())) {
              std::vector<int> indices(matches.size());
              std::iota(indices.begin(), indices.end(), 0);
              std::stable_sort(indices.begin(),indices.end(),[&](int a,int b) {
                return matches[a].distance < matches[b].distance;
              });
              indices.resize(problem->getSampleSize());
              est.valid = problem->computeModelCoefficients(indices,est.model) && est.model.allFinite();
              if (est.valid) { std::vector<double> distances; problem->selectWithinDistance(est.model,5e-6,est.inliers,distances); }
            }
          } else est = consensus(problem, 5e-6, b1.size());
        }
        const auto ms = std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-start).count();
        const double nan = std::numeric_limits<double>::quiet_NaN();
        Rpe rpe{nan,nan,nan};
        if (est.valid) {
          Pose relative = Pose::Identity();
          relative.linear()=est.model.leftCols<3>();
          relative.translation()=est.model.col(3);
          rpe=evaluateRPE(truth,relative,metric);
          // Use truth scale only to draw a local 2D–2D pose, after evaluating RPE.
          if (!metric && relative.translation().norm()>1e-9)
            relative.translation().normalize(), relative.translation() *= truth.translation().norm();
          const auto estimate = stampedPose(previous_gt_*relative,frame.rgb->header.stamp);
          bags_[i].write("/camera_pose",frame.bag_time,estimate);
          if (!sample_[i] && truth.translation().norm()>0.03 && est.inliers.size()>=12) {
            std::vector<cv::DMatch> selected;
            for(int j:est.inliers) selected.push_back(matches[metric ? cloud_indices[j] : j]);
            cv::Mat view;
            cv::drawMatches(previous_,previous_features_.keypoints,color,features.keypoints,selected,
                            view,cv::Scalar(60,210,60),cv::Scalar::all(-1),{},
                            cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            cv::imwrite(output_+"/"+kMethods[i]+"_inliers.png",view);
            sample_[i]=true;
          }
        }
        bags_[i].write("/gt_camera_pose",frame.bag_time,stampedPose(gt,frame.rgb->header.stamp));
        csv_<<pair_<<','<<frame.rgb->header.stamp.toSec()-first_time_<<','<<kMethods[i]<<','
            <<(metric?cloud1.size():matches.size())<<','<<est.inliers.size()<<','
            <<(est.valid?"ok":"failed")<<','<<rpe.chordal<<','<<rpe.degrees<<','
            <<rpe.translation<<','<<truth.translation().norm()<<','<<ms<<'\n';
      }
      ++pair_;
      if(pair_%50==0) std::cout<<"Processed "<<pair_<<" frame pairs"<<std::endl;
    } else first_time_=frame.rgb->header.stamp.toSec();
    previous_=color;
    previous_depth_=depth;
    previous_features_=features;
    previous_gt_=gt;
  }
 private:
  std::string output_;
  cv::Mat k_,d_,previous_,previous_depth_;
  std::ofstream csv_;
  std::array<rosbag::Bag,5> bags_;
  std::array<bool,5> sample_{};
  SiftMatcher matcher_;
  Features previous_features_;
  Pose previous_gt_;
  size_t pair_=0;
  double first_time_=0;
};

int run(const std::string& path,const std::string& output) {
  rosbag::Bag bag(path,rosbag::bagmode::Read);
  std::vector<nav_msgs::OdometryConstPtr> odom;
  cv::Mat k=(cv::Mat_<double>(3,3)<<415.69219381653,0,360,0,415.69219381653,240,0,0,1);
  cv::Mat d=cv::Mat::zeros(1,5,CV_64F);
  rosbag::View meta(bag,rosbag::TopicQuery({"/tesse/odom","/current_state",kDepth,kInfo}));
  // Index only timestamps and message handles; deserialize depth lazily in the RGB pass.
  std::vector<std::pair<ros::Time,std::shared_ptr<rosbag::MessageInstance>>> depth_index;
  std::vector<nav_msgs::OdometryConstPtr> fallback_odom;
  bool calibration_from_bag=false;
  for(const auto& item:meta) {
    if(auto o=item.instantiate<nav_msgs::Odometry>()) {
      (item.getTopic()=="/tesse/odom"?odom:fallback_odom).push_back(o);
    } else if(auto c=item.instantiate<sensor_msgs::CameraInfo>()) {
      if(c->K[0]>0 && c->K[4]>0) {
        k=cv::Mat(3,3,CV_64F,const_cast<double*>(c->K.data())).clone();
        if(!c->D.empty()) d=cv::Mat(c->D,true).reshape(1,1);
        calibration_from_bag=true;
      }
    } else if(auto image=item.instantiate<sensor_msgs::Image>())
      depth_index.emplace_back(image->header.stamp,std::make_shared<rosbag::MessageInstance>(item));
  }
  if(odom.empty()) odom.swap(fallback_odom);
  if(odom.size()<2) throw std::runtime_error("No usable odometry topic");
  std::stable_sort(odom.begin(),odom.end(),[](const auto& a,const auto& b){return a->header.stamp<b->header.stamp;});
  std::stable_sort(depth_index.begin(),depth_index.end(),[](const auto& a,const auto& b){return a.first<b.first;});
  std::cout<<"Calibration "<<(calibration_from_bag?"CameraInfo":"course default")<<" K="<<k<<" D="<<d
           <<"\nOdometry samples="<<odom.size()<<"; depth samples="<<depth_index.size()<<std::endl;
  Evaluation evaluation(output,k,d);
  rosbag::View rgb(bag,rosbag::TopicQuery({kRgb}));
  ros::Time last;
  size_t used=0, no_pose=0, no_depth=0;
  for(const auto& item:rgb) {
    const auto image=item.instantiate<sensor_msgs::Image>();
    if(!image || (used && image->header.stamp<=last)) continue;
    Pose truth;
    if(!interpolatePose(odom,image->header.stamp,truth)) { ++no_pose; continue; }
    auto hi=std::lower_bound(depth_index.begin(),depth_index.end(),image->header.stamp,
        [](const auto& a,const auto& t){return a.first<t;});
    if(hi!=depth_index.begin() && (hi==depth_index.end() ||
       (image->header.stamp-(hi-1)->first).toSec()<(hi->first-image->header.stamp).toSec())) --hi;
    Image depth_msg;
    if(hi!=depth_index.end() && std::abs((hi->first-image->header.stamp).toSec())<=0.015)
      depth_msg=hi->second->instantiate<sensor_msgs::Image>();
    else ++no_depth;
    evaluation.cameraCallback({image,depth_msg,item.getTime()},truth);
    last=image->header.stamp;
    ++used;
  }
  std::cout<<"RGB used="<<used<<"; no bracketing pose="<<no_pose<<"; no aligned depth="<<no_depth<<std::endl;
  return used>=2?0:1;
}
}  // namespace

int main(int argc,char** argv) {
  ros::Time::init();
  cv::setNumThreads(1);
  cv::setRNGSeed(42);
  if(argc!=3) { std::cerr<<"Usage: pose_estimation input.bag existing_output_directory\n"; return 2; }
  try { return run(argv[1],argv[2]); }
  catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
