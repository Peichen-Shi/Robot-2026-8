// 基于 MIT-SPARK VNAV-labs VNAV2021 教学模板。
// 描述子方法的公共流程：检测、描述、匹配、几何验证、绘图和累计统计。
// Shared pipeline for SIFT, SURF, ORB and FAST+BRIEF.

#include "feature_tracker.h"

#include <vector>
#include <numeric>
#include <algorithm>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <ros/ros.h>

using namespace cv;
using namespace cv::xfeatures2d;

// 输入两幅图像；可选输出只包含通过 RANSAC 的对应点，两个向量索引一一对应。
void FeatureTracker::trackFeatures(
    const cv::Mat& img_1, const cv::Mat& img_2,
    std::pair<std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>>* matched_kp_1_kp_2) {
  // 1. 分别检测并描述。描述计算可能删除边界附近的关键点。
  std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
  cv::Mat d1, d2;
  detectKeypoints(img_1, &keypoints_1);
  detectKeypoints(img_2, &keypoints_2);
  describeKeypoints(img_1, &keypoints_1, &d1);
  describeKeypoints(img_2, &keypoints_2, &d2);
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::DMatch> good_matches;
  // 2. 二近邻匹配，并用距离比删除容易混淆的候选。
  matchDescriptors(d1, d2, &matches, &good_matches);
  std::vector<cv::KeyPoint> a, b;
  for (auto m : good_matches) {
    a.push_back(keypoints_1[m.queryIdx]);
    b.push_back(keypoints_2[m.trainIdx]);
  }
  // 3. 至少需要 8 个候选点才估计基本矩阵；失败时保持全零掩码。
  std::vector<uchar> mask(good_matches.size(), 0);
  if (a.size() >= 8) inlierMaskComputation(a, b, &mask);
  if (mask.size() != good_matches.size()) mask.assign(good_matches.size(), 0);
  unsigned num_inliers = std::count(mask.begin(), mask.end(), 1);
  if (matched_kp_1_kp_2) {
    matched_kp_1_kp_2->first.clear();
    matched_kp_1_kp_2->second.clear();
    for (size_t i = 0; i < mask.size(); ++i)
      if (mask[i]) {
        matched_kp_1_kp_2->first.push_back(a[i]);
        matched_kp_1_kp_2->second.push_back(b[i]);
      }
  }
  cv::Mat vis;
  cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, vis, cv::Scalar(0, 0, 255),
                  cv::Scalar::all(-1), std::vector<char>(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::drawMatches(
      img_1, keypoints_1, img_2, keypoints_2, good_matches, vis, cv::Scalar(0, 255, 0),
      cv::Scalar::all(-1), std::vector<char>(mask.begin(), mask.end()),
      cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::imshow("tracked_features", vis);
  cv::waitKey(1);
  // 4. 累积每帧对的算术平均。内点率的分母是筛选后数量，空集合记为 0。
  float n = static_cast<float>(num_samples_);
  float den = n + 1;
  avg_num_keypoints_img1_ = (avg_num_keypoints_img1_ * n + keypoints_1.size()) / den;
  avg_num_keypoints_img2_ = (avg_num_keypoints_img2_ * n + keypoints_2.size()) / den;
  avg_num_matches_ = (avg_num_matches_ * n + matches.size()) / den;
  avg_num_good_matches_ = (avg_num_good_matches_ * n + good_matches.size()) / den;
  avg_num_inliers_ = (avg_num_inliers_ * n + num_inliers) / den;
  avg_inlier_ratio_ = (avg_inlier_ratio_ * n +
                       (good_matches.empty() ? 0.0f : float(num_inliers) / good_matches.size())) /
                      den;
  ++num_samples_;
}

void FeatureTracker::inlierMaskComputation(const std::vector<KeyPoint>& keypoints_1,
                                           const std::vector<KeyPoint>& keypoints_2,
                                           std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);
  const size_t size = keypoints_1.size();
  CHECK_EQ(keypoints_2.size(), size) << "Size of keypoint vectors "
                                        "should be the same!";

  std::vector<Point2f> pts1(size);
  std::vector<Point2f> pts2(size);
  for (size_t i = 0; i < keypoints_1.size(); i++) {
    pts1[i] = keypoints_1[i].pt;
    pts2[i] = keypoints_2[i].pt;
  }

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

void FeatureTracker::drawMatches(const cv::Mat& img_1, const cv::Mat& img_2,
                                 const std::vector<KeyPoint>& keypoints_1,
                                 const std::vector<KeyPoint>& keypoints_2,
                                 const std::vector<std::vector<DMatch>>& matches) {
  cv::namedWindow("tracked_features", cv::WINDOW_NORMAL);
  cv::Mat img_matches;
  cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_matches, Scalar::all(-1),
                  Scalar::all(-1), std::vector<std::vector<char>>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  imshow("tracked_features", img_matches);
  while (ros::ok() && waitKey(10) == -1) {
  }
}

void FeatureTracker::printStats() const {
  std::cout << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_ << std::endl;
  std::cout << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_ << std::endl;
  std::cout << "Avg. Number of matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of good matches: " << avg_num_good_matches_ << std::endl;
  std::cout << "Avg. Number of Inliers: " << avg_num_inliers_ << std::endl;
  std::cout << "Avg. Inliers ratio: " << avg_inlier_ratio_ << std::endl;
  std::cout << "Num. of samples: " << num_samples_ << std::endl;
}
