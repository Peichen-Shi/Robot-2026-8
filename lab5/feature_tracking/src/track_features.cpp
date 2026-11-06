// 基于 MIT-SPARK VNAV-labs VNAV2021 教学模板。
// ROS 程序入口：双图模式用于观察匹配，视频模式订阅图像话题。
// ROS entry point for image-pair matching and video tracking.

#include <stdio.h>
#include <iostream>
#include <memory>

#include <glog/logging.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "feature_tracker.h"
#include "sift_feature_tracker.h"
#include "surf_feature_tracker.h"
#include "orb_feature_tracker.h"
#include "fast_feature_tracker.h"
#include "lk_feature_tracker.h"

using namespace std;
using namespace cv::xfeatures2d;

void imageCallback(const sensor_msgs::ImageConstPtr& msg,
                   std::shared_ptr<FeatureTracker> feature_tracker) {
  try {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("view", image);
    static cv::Mat prev_image;
    if (prev_image.empty()) {
      prev_image = image.clone();
      return;
    }
    feature_tracker->trackFeatures(prev_image, image);
    cv::waitKey(1);
    prev_image = image.clone();
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void lkImageCallback(const sensor_msgs::ImageConstPtr& msg,
                     std::shared_ptr<LKFeatureTracker> lk_tracker) {
  try {
    lk_tracker->trackFeatures(cv_bridge::toCvShare(msg, "bgr8")->image);
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("%s", e.what());
  }
}

bool retrieveImages(const ros::NodeHandle& nh, cv::Mat* img_1, cv::Mat* img_2) {
  CHECK_NOTNULL(img_1);
  CHECK_NOTNULL(img_2);
  std::string img1_path;
  if (nh.getParam("img1_path", img1_path)) {
    ROS_INFO("Got param: %s", img1_path.c_str());
  } else {
    ROS_FATAL("Failed to get param 'img1_path'");
    return false;
  }

  std::string img2_path;
  if (nh.getParam("img2_path", img2_path)) {
    ROS_INFO("Got param: %s", img2_path.c_str());
  } else {
    ROS_FATAL("Failed to get param 'img2_path'");
    return false;
  }

  *img_1 = imread(img1_path, cv::IMREAD_GRAYSCALE);
  *img_2 = imread(img2_path, cv::IMREAD_GRAYSCALE);
  if (!img_1->data || !img_2->data) {
    ROS_FATAL(" --(!) Error reading images ");
    return false;
  }

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "keypoint_trackers");
  ros::NodeHandle local_nh("~");

  int const mode = local_nh.param<int>("mode", 0);
  std::string const descriptor = local_nh.param<std::string>("descriptor", "");

  std::shared_ptr<FeatureTracker> feature_tracker;
  std::shared_ptr<LKFeatureTracker> lk_tracker;
  if (descriptor == "SIFT") {
    feature_tracker.reset(new SiftFeatureTracker());
  } else if (descriptor == "SURF") {
    feature_tracker.reset(new SurfFeatureTracker());
  } else if (descriptor == "ORB") {
    feature_tracker.reset(new OrbFeatureTracker());
  } else if (descriptor == "FAST") {
    feature_tracker.reset(new FastFeatureTracker());
  } else if (descriptor == "LK") {
    lk_tracker.reset(new LKFeatureTracker());
  } else {
    ROS_ERROR("Unknown descriptor %s", descriptor.c_str());
    return EXIT_FAILURE;
  }

  if (mode == 0) {
    if (!feature_tracker) {
      ROS_ERROR("Cannot use descriptor \"%s\" for images", descriptor.c_str());
      return EXIT_FAILURE;
    }
    cv::Mat img_1, img_2;
    if (!retrieveImages(local_nh, &img_1, &img_2)) {
      ROS_ERROR("Failed to retrieve images");
      return EXIT_FAILURE;
    }
    feature_tracker->trackFeatures(img_1, img_2);
    feature_tracker->printStats();
    while (ros::ok() && cv::waitKey(30) < 0) {
    }
  } else if (mode == 1) {
    if (descriptor != "LK") cv::namedWindow("view", cv::WINDOW_NORMAL);
    image_transport::ImageTransport it(local_nh);
    auto callback = [feature_tracker, lk_tracker,
                     descriptor](const sensor_msgs::ImageConstPtr& msg) {
      if (descriptor == "LK") {
        lkImageCallback(msg, lk_tracker);
      } else {
        imageCallback(msg, feature_tracker);
      }
    };
    image_transport::Subscriber sub = it.subscribe("/images_topic", 100, callback);
    ros::spin();
    if (descriptor != "LK") cv::destroyWindow("view");
  } else {
    ROS_ERROR("Unrecognized mode argument");
    return EXIT_FAILURE;
  }

  if (feature_tracker) feature_tracker->printStats();

  return EXIT_SUCCESS;
}
