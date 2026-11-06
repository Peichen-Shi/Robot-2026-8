// 基于 MIT-SPARK VNAV-labs VNAV2021 教学模板。
// 离线评估入口：逐个读取真实 rosbag 的相邻 RGB 图像，避免 GUI 或 ROS 队列丢帧影响统计。
// Offline evaluation of consecutive RGB frames; the CSV files are the report data source.

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <fstream>
#include <iostream>
#include <chrono>
using namespace cv;
// k1/k2: 两幅图的特征数；raw: 最近邻数；good: 筛后数；inliers: 几何内点数。
// ratio 是本帧对 inliers/good；ms 只计计算，不含文件读取和绘图。
struct Result {
  int k1 = 0, k2 = 0, raw = 0, good = 0, inliers = 0;
  double ratio = 0, ms = 0;
};
// 每个 Eval 处理一个方法和一个数据集。finish() 生成均值表。
// LK 没有描述子距离比：raw 和 good 都是状态有效且未越界的跟踪点。
struct Eval {
  std::string name, out;
  Ptr<Feature2D> detector, descriptor;
  bool binary = false;
  int count = 0;
  double sums[7] = {};
  std::ofstream csv;
  Eval(std::string n, std::string o) : name(n), out(o) {
    if (n == "SIFT") detector = xfeatures2d::SIFT::create();
    if (n == "SURF") detector = xfeatures2d::SURF::create(400);
    if (n == "ORB") {
      detector = ORB::create(1000);
      binary = true;
    }
    if (n == "FAST_BRIEF") {
      detector = FastFeatureDetector::create(20, true);
      descriptor = xfeatures2d::BriefDescriptorExtractor::create(32);
      binary = true;
    }
    if (!descriptor) descriptor = detector;
    csv.open(out + "/" + name + "_frames.csv");
    csv << "frame,keypoints1,keypoints2,matches,good,inliers,inlier_ratio,ms\n";
  }
  // 处理一个相邻帧对；save=true 时额外保存说明用的图像。
  Result pair(const Mat& a, const Mat& b, bool save) {
    auto start = std::chrono::steady_clock::now();
    Result r;
    std::vector<KeyPoint> k, l;
    std::vector<DMatch> good, raw;
    std::vector<Point2f> p, q;
    Mat d, e;
    // 离线比较每对帧重新检测 Harris；ROS 跟踪类则会持续保存角点。
    if (name == "LK") {
      goodFeaturesToTrack(a, p, 500, .01, 8, noArray(), 3, true, .04);
      r.k1 = p.size();
      std::vector<Point2f> next;
      std::vector<uchar> status;
      std::vector<float> error;
      if (!p.empty()) calcOpticalFlowPyrLK(a, b, p, next, status, error, Size(21, 21), 3);
      std::vector<Point2f> valid;
      for (size_t i = 0; i < status.size(); i++)
        if (status[i] && next[i].x >= 0 && next[i].x < b.cols && next[i].y >= 0 &&
            next[i].y < b.rows) {
          valid.push_back(p[i]);
          q.push_back(next[i]);
        }
      p = valid;
      for (size_t i = 0; i < p.size(); i++) {
        k.emplace_back(p[i], 3);
        l.emplace_back(q[i], 3);
        good.emplace_back(i, i, 0);
      }
      r.k2 = q.size();
      raw = good;
    } else {
      // 浮点描述子使用 FLANN；二值描述子使用 Hamming，不能混用距离度量。
      detector->detect(a, k);
      detector->detect(b, l);
      descriptor->compute(a, k, d);
      descriptor->compute(b, l, e);
      r.k1 = k.size();
      r.k2 = l.size();
      std::vector<std::vector<DMatch>> pairs;
      if (!d.empty() && e.rows >= 2) {
        if (binary)
          BFMatcher(NORM_HAMMING).knnMatch(d, e, pairs, 2);
        else
          FlannBasedMatcher().knnMatch(d, e, pairs, 2);
      }
      for (auto& v : pairs) {
        if (!v.empty()) raw.push_back(v[0]);
        if (v.size() == 2 && v[0].distance < .8 * v[1].distance) good.push_back(v[0]);
      }
      for (auto m : good) {
        p.push_back(k[m.queryIdx].pt);
        q.push_back(l[m.trainIdx].pt);
      }
    }
    // RANSAC 内点表示符合所估计的极线模型，不等同于人工标注的准确率。
    std::vector<uchar> mask(good.size(), 0);
    if (p.size() >= 8) {
      try {
        findFundamentalMat(p, q, FM_RANSAC, 3, .99, mask);
      } catch (const cv::Exception&) {
      }
    }
    if (mask.size() != good.size()) mask.assign(good.size(), 0);
    r.raw = raw.size();
    r.good = good.size();
    r.inliers = std::count(mask.begin(), mask.end(), 1);
    r.ratio = r.good ? double(r.inliers) / r.good : 0;
    r.ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    count++;
    double vals[] = {double(r.k1),      double(r.k2), double(r.raw), double(r.good),
                     double(r.inliers), r.ratio,      r.ms};
    for (int i = 0; i < 7; i++) sums[i] += vals[i];
    csv << count;
    for (double v : vals) csv << "," << v;
    csv << "\n";
    if (save) {
      Mat vis, ka, kb;
      drawKeypoints(a, k, ka, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      drawKeypoints(b, l, kb, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      imwrite(out + "/" + name + "_keypoints1.png", ka);
      imwrite(out + "/" + name + "_keypoints2.png", kb);
      drawMatches(a, k, b, l, raw, vis, Scalar::all(-1), Scalar::all(-1), std::vector<char>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      imwrite(out + "/" + name + "_raw.png", vis);
      drawMatches(a, k, b, l, good, vis, Scalar::all(-1), Scalar::all(-1), std::vector<char>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      imwrite(out + "/" + name + "_ratio.png", vis);
      drawMatches(a, k, b, l, good, vis, Scalar(0, 0, 255), Scalar::all(-1), std::vector<char>(),
                  DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      drawMatches(a, k, b, l, good, vis, Scalar(0, 255, 0), Scalar::all(-1),
                  std::vector<char>(mask.begin(), mask.end()),
                  DrawMatchesFlags::DRAW_OVER_OUTIMG | DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      imwrite(out + "/" + name + "_inliers.png", vis);
      // 离线比较每对帧重新检测 Harris；ROS 跟踪类则会持续保存角点。
      if (name == "LK") {
        Mat tracks;
        cvtColor(b, tracks, COLOR_GRAY2BGR);
        for (size_t i = 0; i < p.size(); i++) {
          line(tracks, p[i], q[i], Scalar(0, 255, 0), 1);
          circle(tracks, q[i], 2, Scalar(0, 0, 255), -1);
        }
        imwrite(out + "/LK_tracks.png", tracks);
      }
    }
    return r;
  }
  // 先算每帧对的 ratio，再对 ratio 取均值；不是累计内点/累计筛后数。
  void finish() {
    std::ofstream f(out + "/" + name + "_summary.csv");
    f << "method,pairs,keypoints1,keypoints2,matches,good,inliers,inlier_ratio,ms\n";
    f << name << "," << count;
    for (double v : sums) f << "," << (count ? v / count : 0);
    f << "\n";
    std::cout << name << " pairs=" << count;
    for (double v : sums) std::cout << " " << (count ? v / count : 0);
    std::cout << std::endl;
  }
};
int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr
        << "usage: lab5_evaluate pair img1 img2 output OR bag file output method [max_pairs=0]\n";
    return 2;
  }
  // 固定 OpenCV 工作线程和随机种子，减少重复执行的差异。
  setNumThreads(1);
  setRNGSeed(42);
  std::string mode = argv[1];
  if (mode == "pair") {
    Mat a = imread(argv[2], 0), b = imread(argv[3], 0);
    if (a.empty() || b.empty()) return 3;
    for (auto n : {"SIFT", "SURF", "ORB", "FAST_BRIEF"}) {
      Eval e(n, argv[4]);
      e.pair(a, b, true);
      e.finish();
    }
    return 0;
  }
  std::string path = argv[2], out = argv[3], method = argv[4];
  int maxPairs = argc > 5 ? std::stoi(argv[5]) : 0;
  Eval e(method, out);
  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);
  rosbag::View all(bag);
  std::string topic;
  // 只选择一个 RGB 话题，防止深度图、左右相机交叉配对。
  for (auto c : all.getConnections())
    if (c->datatype == "sensor_msgs/Image") {
      if (topic.empty()) topic = c->topic;
      if (c->topic == "/camera/color/image_raw" || c->topic == "/tesse/left_cam/rgb/image_raw") {
        topic = c->topic;
        break;
      }
    }
  if (topic.empty()) {
    std::cerr << "No image topic";
    return 4;
  }
  std::cout << "Image topic: " << topic << std::endl;
  rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{topic}));
  Mat prev;
  int seen = 0;
  for (auto const& m : view) {
    auto image = m.instantiate<sensor_msgs::Image>();
    if (!image) continue;
    Mat frame = cv_bridge::toCvCopy(image, "mono8")->image;
    if (!prev.empty()) {
      // FLOW 在第 31 个相邻帧对保存一幅可视化；self_flow 节点可连续运行。
      if (method == "FLOW") {
        if (seen == 30) {
          Mat flow, xy[2], mag, ang, hsv, bgr;
          calcOpticalFlowFarneback(prev, frame, flow, .5, 3, 15, 3, 5, 1.2, 0);
          split(flow, xy);
          cartToPolar(xy[0], xy[1], mag, ang, true);
          Mat ch[3];
          ang.convertTo(ch[0], CV_8U, .5);
          ch[1] = Mat(frame.size(), CV_8U, Scalar(255));
          normalize(mag, ch[2], 0, 255, NORM_MINMAX, CV_8U);
          merge(ch, 3, hsv);
          cvtColor(hsv, bgr, COLOR_HSV2BGR);
          imwrite(out + "/Farneback.png", bgr);
          break;
        }
      } else
        e.pair(prev, frame, seen == 30);
      seen++;
      if (maxPairs && seen >= maxPairs) break;
    }
    prev = frame.clone();
  }
  if (method != "FLOW") e.finish();
  bag.close();
  return 0;
}
