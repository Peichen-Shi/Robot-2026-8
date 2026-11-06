// 基于 MIT-SPARK VNAV-labs VNAV2021 教学模板。
// 公开接口：声明输入、输出和跟踪器持有的状态。实现位于同名 src 文件。
// Public tracker interface. See the corresponding src file for implementation.

#pragma once

#include "feature_tracker.h"
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d.hpp>

class FeatureTracker {
 public:
  FeatureTracker() = default;

  virtual ~FeatureTracker() = default;

  void trackFeatures(
      const cv::Mat& img_1, const cv::Mat& img_2,
      std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>* matched_kp_1_kp_2 = nullptr);

  void printStats() const;

 protected:
  virtual void detectKeypoints(const cv::Mat& img, std::vector<cv::KeyPoint>* keypoints) const = 0;

  virtual void describeKeypoints(const cv::Mat& img, std::vector<cv::KeyPoint>* keypoints,
                                 cv::Mat* descriptors) const = 0;

  virtual void matchDescriptors(const cv::Mat& descriptors_1, const cv::Mat& descriptors_2,
                                std::vector<std::vector<cv::DMatch>>* matches,
                                std::vector<cv::DMatch>* good_matches) const = 0;

  void inlierMaskComputation(const std::vector<cv::KeyPoint>& keypoints_1,
                             const std::vector<cv::KeyPoint>& keypoints_2,
                             std::vector<uchar>* inlier_mask) const;

  void drawMatches(const cv::Mat& img_1, const cv::Mat& img_2,
                   const std::vector<cv::KeyPoint>& keypoints_1,
                   const std::vector<cv::KeyPoint>& keypoints_2,
                   const std::vector<std::vector<cv::DMatch>>& matches);

 private:
  float avg_num_keypoints_img1_ = 0;
  float avg_num_keypoints_img2_ = 0;
  float avg_num_matches_ = 0;
  float avg_num_good_matches_ = 0;
  float avg_num_inliers_ = 0;
  float avg_inlier_ratio_ = 0;
  unsigned int num_samples_ = 0;
};
