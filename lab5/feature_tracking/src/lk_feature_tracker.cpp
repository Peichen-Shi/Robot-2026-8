// 基于 MIT-SPARK VNAV-labs VNAV2021 教学模板。
// Harris + LK 视频跟踪：保存前帧和存活角点，角点不足时重新检测。
// Stateful Harris corner tracking with pyramidal Lucas-Kanade optical flow.

#include "lk_feature_tracker.h"

#include <numeric>
#include <algorithm>
#include <vector>

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>

using namespace cv;
using namespace cv::xfeatures2d;

LKFeatureTracker::LKFeatureTracker() {
  cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
  cv::resizeWindow(window_name_, 900, 600);
  cv::moveWindow(window_name_, 180, 100);
}

LKFeatureTracker::~LKFeatureTracker() { cv::destroyWindow(window_name_); }

void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {
  // 统一转为灰度，第一帧只初始化，不与自身配对。
  cv::Mat gray;
  if (frame.channels() == 3)
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  else
    gray = frame;
  if (prev_frame_.empty()) {
    prev_frame_ = gray.clone();
    cv::goodFeaturesToTrack(gray, prev_corners_, 500, 0.01, 8, cv::noArray(), 3, true, 0.04);
    return;
  }
  std::vector<cv::Point2f> next, a, b;
  std::vector<uchar> status;
  std::vector<float> error;
  if (!prev_corners_.empty())
    cv::calcOpticalFlowPyrLK(prev_frame_, gray, prev_corners_, next, status, error,
                             cv::Size(21, 21), 3);
  for (size_t i = 0; i < status.size(); ++i)
    if (status[i] && next[i].x >= 0 && next[i].y >= 0 && next[i].x < gray.cols &&
        next[i].y < gray.rows) {
      a.push_back(prev_corners_[i]);
      b.push_back(next[i]);
    }
  std::vector<uchar> mask(a.size(), 0);
  if (a.size() >= 8) inlierMaskComputation(a, b, &mask);
  unsigned inliers = std::count(mask.begin(), mask.end(), 1);
  float n = num_samples_, d = n + 1;
  avg_num_keypoints_img1_ = (avg_num_keypoints_img1_ * n + prev_corners_.size()) / d;
  avg_num_keypoints_img2_ = (avg_num_keypoints_img2_ * n + b.size()) / d;
  avg_num_matches_ = (avg_num_matches_ * n + b.size()) / d;
  avg_num_inliers_ = (avg_num_inliers_ * n + inliers) / d;
  avg_inlier_ratio_ = (avg_inlier_ratio_ * n + (b.empty() ? 0.0 : double(inliers) / b.size())) / d;
  ++num_samples_;
  show(frame, a, b);
  prev_frame_ = gray.clone();
  prev_corners_ = b;
  // 剩余角点太少时补充检测，保证后续仍有可跟踪的点。
  if (prev_corners_.size() < 150)
    cv::goodFeaturesToTrack(gray, prev_corners_, 500, 0.01, 8, cv::noArray(), 3, true, 0.04);
}

void LKFeatureTracker::show(const cv::Mat& frame, std::vector<cv::Point2f>& prev,
                            std::vector<cv::Point2f>& curr) {
  cv::Mat vis;
  if (frame.channels() == 1)
    cv::cvtColor(frame, vis, cv::COLOR_GRAY2BGR);
  else
    vis = frame.clone();
  for (size_t i = 0; i < prev.size(); ++i) {
    cv::line(vis, prev[i], curr[i], cv::Scalar(0, 255, 0));
    cv::circle(vis, curr[i], 2, cv::Scalar(0, 0, 255), -1);
  }
  cv::imshow(window_name_, vis);
  cv::waitKey(1);
}

void LKFeatureTracker::inlierMaskComputation(const std::vector<cv::Point2f>& pts1,
                                             const std::vector<cv::Point2f>& pts2,
                                             std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, cv::FM_RANSAC, max_dist_from_epi_line_in_px, confidence_prob,
                       *inlier_mask);
  } catch (...) {
    ROS_WARN(
        "Inlier Mask could not be computed, this can happen if there"
        "are not enough features tracked.");
  }
}
