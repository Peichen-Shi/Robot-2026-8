// 基于 MIT-SPARK VNAV-labs VNAV2021 教学模板。
// 公开接口：声明输入、输出和跟踪器持有的状态。实现位于同名 src 文件。
// Public tracker interface. See the corresponding src file for implementation.

#include "feature_tracker.h"

class FastFeatureTracker : public FeatureTracker {
 public:
  FastFeatureTracker();

  ~FastFeatureTracker() override = default;

 protected:
  void detectKeypoints(const cv::Mat& img, std::vector<cv::KeyPoint>* keypoints) const override;

  void describeKeypoints(const cv::Mat& img, std::vector<cv::KeyPoint>* keypoints,
                         cv::Mat* descriptors) const override;

  void matchDescriptors(const cv::Mat& descriptors_1, const cv::Mat& descriptors_2,
                        std::vector<std::vector<cv::DMatch>>* matches,
                        std::vector<cv::DMatch>* good_matches) const override;

 private:
  cv::Ptr<cv::FeatureDetector> detector;
};
