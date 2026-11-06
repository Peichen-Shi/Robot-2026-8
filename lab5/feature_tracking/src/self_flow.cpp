// 基于 MIT-SPARK VNAV-labs VNAV2021 教学模板。
// 稠密光流 ROS 节点：每个像素估计运动，色相表示方向，亮度表示帧内归一化幅值。
// Dense Farneback optical flow; brightness is normalized separately for each frame.

#include <memory>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    static cv::Mat previous;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    if (!previous.empty()) {
      cv::Mat flow;
      cv::calcOpticalFlowFarneback(previous, gray, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
      cv::Mat xy[2], mag, angle, hsv, bgr;
      cv::split(flow, xy);
      cv::cartToPolar(xy[0], xy[1], mag, angle, true);
      cv::Mat channels[3];
      angle.convertTo(channels[0], CV_8U, 0.5);
      channels[1] = cv::Mat(gray.size(), CV_8U, cv::Scalar(255));
      cv::normalize(mag, channels[2], 0, 255, cv::NORM_MINMAX, CV_8U);
      cv::merge(channels, 3, hsv);
      cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
      cv::imshow("view", bgr);
      cv::waitKey(1);
    }
    previous = gray.clone();

  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "optical_flow");
  ros::NodeHandle local_nh("~");

  cv::namedWindow("view", cv::WINDOW_NORMAL);
  image_transport::ImageTransport it(local_nh);
  image_transport::Subscriber sub = it.subscribe("/images_topic", 100, imageCallback);

  ros::spin();
  cv::destroyWindow("view");
  return EXIT_SUCCESS;
}
