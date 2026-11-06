// 基于 MIT-SPARK VNAV-labs VNAV2021 教学模板。
// SIFT 适配器：使用 OpenCV 检测、描述和二近邻匹配。
// SIFT adapter for keypoint detection, description and nearest-neighbor matching.

#include "sift_feature_tracker.h"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
SiftFeatureTracker::SiftFeatureTracker() : detector(cv::xfeatures2d::SIFT::create()) {}
void SiftFeatureTracker::detectKeypoints(const cv::Mat& img,
                                         std::vector<cv::KeyPoint>* keypoints) const {
  detector->detect(img, *keypoints);
}
void SiftFeatureTracker::describeKeypoints(const cv::Mat& img, std::vector<cv::KeyPoint>* keypoints,
                                           cv::Mat* descriptors) const {
  detector->compute(img, *keypoints, *descriptors);
}
void SiftFeatureTracker::matchDescriptors(const cv::Mat& a, const cv::Mat& b,
                                          std::vector<std::vector<cv::DMatch>>* matches,
                                          std::vector<cv::DMatch>* good) const {
  // 输出先清空；训练描述子不足 2 个时无法做二近邻比值检查。
  matches->clear();
  good->clear();
  if (a.empty() || b.rows < 2) return;
  cv::FlannBasedMatcher matcher;
  matcher.knnMatch(a, b, *matches, 2);
  // 仅保留 best_distance < 0.8 * second_distance 的明确匹配。
  for (const auto& pair : *matches)
    if (pair.size() == 2 && pair[0].distance < 0.8f * pair[1].distance) good->push_back(pair[0]);
}
