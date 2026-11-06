// 基于 MIT-SPARK VNAV-labs VNAV2021 教学模板。
// 公开接口：声明输入、输出和跟踪器持有的状态。实现位于同名 src 文件。
// Public tracker interface. See the corresponding src file for implementation.

#pragma once

#include <vector>
#include "feature_tracker.h"

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

class LKFeatureTracker {
 public:
  LKFeatureTracker();

  ~LKFeatureTracker();

  void trackFeatures(const cv::Mat& frame);

  void inlierMaskComputation(const std::vector<cv::Point2f>& pts1,
                             const std::vector<cv::Point2f>& pts2,
                             std::vector<uchar>* inlier_mask) const;

 private:
  void show(const cv::Mat& frame, std::vector<cv::Point2f>& prev, std::vector<cv::Point2f>& curr);

  static constexpr const char* window_name_ = "LK";
  cv::Mat prev_frame_;
  std::vector<cv::Point2f> prev_corners_;
  float avg_num_keypoints_img1_ = 0;
  float avg_num_keypoints_img2_ = 0;
  float avg_num_matches_ = 0;
  float avg_num_inliers_ = 0;
  float avg_inlier_ratio_ = 0;
  unsigned int num_samples_ = 0;
};
