// VNAV Lab 6: calibrated SIFT correspondences and OpenGV relative pose.
// T12 maps coordinates from current camera (2) into previous camera (1).
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "nonfree.hpp"
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opengv/sac_problems/relative_pose/TranslationOnlySacProblem.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/point_cloud/PointCloudAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/point_cloud/methods.hpp>
#include <fstream>
#include <filesystem>
#include <cstdlib>
#include <iomanip>
#include <map>
#include <chrono>
#include <algorithm>

using Pose = Eigen::Isometry3d;
using Central = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
using Translation = opengv::sac_problems::relative_pose::TranslationOnlySacProblem;
using Cloud = opengv::sac_problems::point_cloud::PointCloudSacProblem;
struct Frame { cv::Mat rgb, depth, descriptors; std::vector<cv::KeyPoint> kp; Pose gt; double time; };
struct Result { bool ok=false; Pose pose=Pose::Identity(); int inliers=0; };
struct Errors { double translation, chordal, degrees; };

Pose bodyPose(const geometry_msgs::Pose& p) {
  Pose T=Pose::Identity();
  T.linear()=Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z).normalized().toRotationMatrix();
  T.translation()=Eigen::Vector3d(p.position.x,p.position.y,p.position.z); return T;
}
Pose cameraExtrinsics() {
  Pose T=Pose::Identity(); T.linear()<<0,0,1,-1,0,0,0,-1,0; T.translation()<<0,0.05,0; return T;
}
geometry_msgs::PoseStamped poseMessage(const Pose& T, double time) {
  geometry_msgs::PoseStamped m; m.header.frame_id="world"; m.header.stamp=ros::Time(time);
  m.pose.position.x=T.translation().x();m.pose.position.y=T.translation().y();m.pose.position.z=T.translation().z();
  Eigen::Quaterniond q(T.linear()); m.pose.orientation.w=q.w();m.pose.orientation.x=q.x();m.pose.orientation.y=q.y();m.pose.orientation.z=q.z();return m;
}

// Unit direction errors for monocular methods; metric errors for RGB-D.
// Translation direction is undefined at zero baseline: record NaN, never zero.
Errors evaluateRPE(const Pose& previous_gt,const Pose& current_gt,const Pose& relative,int method) {
  Pose truth=previous_gt.inverse()*current_gt;
  Eigen::Vector3d a=truth.translation(),b=relative.translation();
  double te=std::numeric_limits<double>::quiet_NaN();
  if(method==3) te=(a-b).norm();
  else if(a.norm()>1e-6 && b.norm()>1e-9) te=(a.normalized()-b.normalized()).norm();
  double chord=(truth.linear()-relative.linear()).norm();
  double cosine=((truth.linear().transpose()*relative.linear()).trace()-1)*0.5;
  return {te,chord,std::acos(std::max(-1.,std::min(1.,cosine)))*180/M_PI};
}

class Estimator {
 public:
  cv::Mat K=(cv::Mat_<double>(3,3)<<415.69219381653056,0,360,0,415.69219381653056,240,0,0,1);
  cv::Mat D=cv::Mat::zeros(1,5,CV_64F);
  cv::Ptr<cv::Feature2D> sift=cv::xfeatures2d::SIFT::create(1600);
  Frame previous; bool initialized=false; int frames=0,selected=-1;
  std::ofstream csv; std::string output; double start=0;
  ros::Publisher publishers[4],gtpub,matchpub;
  Estimator(const std::string& directory,int method):selected(method),output(directory) {
    std::filesystem::create_directories(output);
    csv.open(output+"/rpe.csv"); if(!csv) throw std::runtime_error("Cannot open output/rpe.csv");
    csv<<"stamp,elapsed,frame,method,status,matches,inliers,baseline_m,translation_error,rotation_chordal,rotation_deg,solve_ms\n"<<std::setprecision(12);
  }
  void calibrateKeypoints(const std::vector<cv::Point2f>& a,const std::vector<cv::Point2f>& b,opengv::bearingVectors_t& u,opengv::bearingVectors_t& v) {
    std::vector<cv::Point2f> x,y; cv::undistortPoints(a,x,K,D);cv::undistortPoints(b,y,K,D);
    for(size_t i=0;i<x.size();++i) {u.push_back(Eigen::Vector3d(x[i].x,x[i].y,1).normalized());v.push_back(Eigen::Vector3d(y[i].x,y[i].y,1).normalized());}
  }
  template<class Problem> Result solve(std::shared_ptr<Problem> problem,double threshold) {
    Result r; opengv::sac::Ransac<Problem> ran;ran.sac_model_=problem;ran.threshold_=threshold;ran.max_iterations_=1000;ran.probability_=0.999;
    if(!ran.computeModel() || ran.inliers_.size()<size_t(problem->getSampleSize())) return r;
    auto model=ran.model_coefficients_;
    // Refit all RGB-D inliers. Keep the fixed rotation for the two-point method.
    if(std::is_same<Problem,Cloud>::value) problem->optimizeModelCoefficients(ran.inliers_,ran.model_coefficients_,model);
    if(!model.allFinite() || std::abs(model.block(0,0,3,3).determinant()-1)>1e-3) return r;
    r.pose.linear()=model.block(0,0,3,3);r.pose.translation()=model.col(3);r.inliers=ran.inliers_.size();r.ok=true;return r;
  }
  float depthAt(const cv::Mat& d,const cv::Point2f& p) {
    int x=std::floor(p.x),y=std::floor(p.y);if(x<0||y<0||x>=d.cols||y>=d.rows)return NAN;
    if(d.type()==CV_32FC1)return d.at<float>(y,x);
    if(d.type()==CV_16UC1)return d.at<uint16_t>(y,x)*0.001f;return NAN;
  }
  Result estimate(int method,opengv::bearingVectors_t& u,opengv::bearingVectors_t& v,const std::vector<cv::Point2f>& a,const std::vector<cv::Point2f>& b,const Frame& f) {
    opengv::relative_pose::CentralRelativeAdapter adapter(u,v);
    // OpenGV uses an extra correspondence for pose disambiguation (8/9 samples).
    double angular=2*(1-std::cos(std::atan(0.25/K.at<double>(0,0))));
    if(method<2) {
      if(u.size()<size_t(method==0?8:9))return {};
      return solve(std::make_shared<Central>(adapter,method==0?Central::NISTER:Central::EIGHTPT,false),angular);
    }
    if(method==2) {
      if(u.size()<2)return {};
      Pose relative=previous.gt.inverse()*f.gt;adapter.setR12(relative.linear());
      return solve(std::make_shared<Translation>(adapter,false),angular);
    }
    opengv::points_t x,y;
    for(size_t i=0;i<u.size();++i) {
      double d1=depthAt(previous.depth,a[i]),d2=depthAt(f.depth,b[i]);
      if(!std::isfinite(d1)||!std::isfinite(d2)||d1<=0||d2<=0||d1>100||d2>100)continue;
      x.push_back(u[i]*(d1/u[i].z()));y.push_back(v[i]*(d2/v[i].z()));
    }
    if(x.size()<3)return {};opengv::point_cloud::PointCloudAdapter cloud(x,y);
    // PointCloudSacProblem scores Euclidean distances, in metres (not squared).
    return solve(std::make_shared<Cloud>(cloud,false),0.05);
  }
  void cameraCallback(const sensor_msgs::ImageConstPtr& rgb,const sensor_msgs::ImageConstPtr& depth,const Pose& gt) {
    Frame f;f.time=rgb->header.stamp.toSec();f.gt=gt;
    f.rgb=cv_bridge::toCvCopy(rgb,"bgr8")->image;f.depth=cv_bridge::toCvCopy(depth)->image;
    cv::Mat gray;cv::cvtColor(f.rgb,gray,cv::COLOR_BGR2GRAY);sift->detectAndCompute(gray,cv::noArray(),f.kp,f.descriptors);
    ++frames;
    if(!initialized) {previous=f;initialized=true;start=f.time;return;}
    std::vector<cv::Point2f> a,b;std::vector<cv::DMatch> good;
    if(!previous.descriptors.empty()&&f.descriptors.rows>=2) {
      std::vector<std::vector<cv::DMatch>> knn;
      cv::FlannBasedMatcher matcher;matcher.knnMatch(previous.descriptors,f.descriptors,knn,2);
      // Lowe ratio, plus unique target indices to avoid repeated correspondences.
      std::map<int,cv::DMatch> unique;
      for(const auto& pair:knn)if(pair.size()==2&&pair[0].distance<0.7*pair[1].distance) {
        auto m=pair[0];auto it=unique.find(m.trainIdx);if(it==unique.end()||m.distance<it->second.distance)unique[m.trainIdx]=m;
      }
      for(auto& kv:unique) {auto m=kv.second;good.push_back(m);a.push_back(previous.kp[m.queryIdx].pt);b.push_back(f.kp[m.trainIdx].pt);}
    }
    opengv::bearingVectors_t u,v;if(!a.empty())calibrateKeypoints(a,b,u,v);
    double baseline=(previous.gt.inverse()*f.gt).translation().norm();
    for(int method=0;method<4;++method) {
      if(selected>=0&&method!=selected)continue;
      auto tick=std::chrono::steady_clock::now();Result r;
      try {r=estimate(method,u,v,a,b,f);}catch(const std::exception& ex) {std::cerr<<ex.what()<<std::endl;}
      double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
      Errors e{NAN,NAN,NAN};if(r.ok)e=evaluateRPE(previous.gt,f.gt,r.pose,method);
      csv<<f.time<<','<<f.time-start<<','<<frames-1<<','<<method<<','<<(r.ok?"ok":"failed")<<','<<a.size()<<','<<r.inliers<<','<<baseline<<','<<e.translation<<','<<e.chordal<<','<<e.degrees<<','<<ms<<'\n';
      if(r.ok&&publishers[method]) {
        Pose rel=r.pose;if(method<3&&rel.translation().norm()>1e-9)rel.translation()=rel.translation().normalized()*baseline;
        publishers[method].publish(poseMessage(previous.gt*rel,f.time));
      }
    }
    if(gtpub)gtpub.publish(poseMessage(f.gt,f.time));
    if(frames==100||frames==1000) {
      cv::Mat view;cv::drawMatches(previous.rgb,previous.kp,f.rgb,f.kp,good,view);
      cv::imwrite(output+"/sift_matches_"+std::to_string(frames)+".png",view);
    }
    previous=f;
    if(frames%50==0) {csv.flush();std::cout<<"Processed "<<frames<<" frames; matches="<<a.size()<<" time="<<f.time-start<<std::endl;}
  }
};

int main(int argc,char** argv) {
  ros::init(argc,argv,"lab6_pose_estimation");cv::setNumThreads(2);cv::setRNGSeed(6);
  std::string bagpath,output=std::string(std::getenv("HOME") ? std::getenv("HOME") : ".")+"/lab6_results";int limit=0,method=-1;
  for(int i=1;i<argc;++i) {std::string s=argv[i];if(s=="--bag"&&i+1<argc)bagpath=argv[++i];else if(s=="--output"&&i+1<argc)output=argv[++i];else if(s=="--limit"&&i+1<argc)limit=std::stoi(argv[++i]);else if(s=="--pose_estimator"&&i+1<argc)method=std::stoi(argv[++i]);}
  try {
    Estimator estimator(output,method);
    if(!bagpath.empty()) {
      rosbag::Bag bag(bagpath,rosbag::bagmode::Read);
      std::map<double,Pose> truth;
      rosbag::View odomview(bag,rosbag::TopicQuery(std::vector<std::string>{"/tesse/odom","/tesse/left_cam/camera_info"}));
      for(const auto& item:odomview) {
        if(auto o=item.instantiate<nav_msgs::Odometry>())truth[o->header.stamp.toSec()]=bodyPose(o->pose.pose);
        if(auto c=item.instantiate<sensor_msgs::CameraInfo>()) {
          for(int i=0;i<9;++i)estimator.K.at<double>(i/3,i%3)=c->K[i];
          if(!c->D.empty())estimator.D=cv::Mat(c->D).reshape(1,1).clone();
        }
      }
      if(truth.empty())throw std::runtime_error("No odometry in bag");
      std::cout<<"Camera K="<<estimator.K<<"; odom="<<truth.size()<<std::endl;
      std::map<uint64_t,sensor_msgs::ImageConstPtr> rgbs,depths;
      rosbag::View view(bag,rosbag::TopicQuery(std::vector<std::string>{"/tesse/left_cam/rgb/image_raw","/tesse/depth_cam/mono/image_raw"}));
      for(const auto& item:view) {
        auto image=item.instantiate<sensor_msgs::Image>();if(!image)continue;
        uint64_t t=image->header.stamp.toNSec();
        if(item.getTopic()=="/tesse/left_cam/rgb/image_raw")rgbs[t]=image;else depths[t]=image;
        auto r=rgbs.find(t),d=depths.find(t);if(r==rgbs.end()||d==depths.end())continue;
        double time=image->header.stamp.toSec();auto hi=truth.lower_bound(time);
        if(hi!=truth.end()) {
          Pose gt=hi->second;
          if(hi!=truth.begin()&&hi->first>time) {
            auto lo=std::prev(hi);double alpha=(time-lo->first)/(hi->first-lo->first);
            gt.translation()=(1-alpha)*lo->second.translation()+alpha*hi->second.translation();
            gt.linear()=Eigen::Quaterniond(lo->second.linear()).slerp(alpha,Eigen::Quaterniond(hi->second.linear())).toRotationMatrix();
          }
          estimator.cameraCallback(r->second,d->second,gt*cameraExtrinsics());
        }
        rgbs.erase(r);depths.erase(d);
        if(rgbs.size()>20)rgbs.erase(rgbs.begin());if(depths.size()>20)depths.erase(depths.begin());
        if(limit>0&&estimator.frames>=limit)break;
      }
      std::cout<<"COMPLETE frames="<<estimator.frames<<std::endl;
    } else {
      ros::NodeHandle nh;
      const char* names[]={"5pt","8pt","2pt","arun"};
      for(int i=0;i<4;++i)estimator.publishers[i]=nh.advertise<geometry_msgs::PoseStamped>(std::string("/lab6/pose_")+names[i],10);
      estimator.gtpub=nh.advertise<geometry_msgs::PoseStamped>("/gt_camera_pose",10);
      auto info=nh.subscribe<sensor_msgs::CameraInfo>("/tesse/left_cam/camera_info",1,[&](const sensor_msgs::CameraInfoConstPtr& c){for(int i=0;i<9;++i)estimator.K.at<double>(i/3,i%3)=c->K[i];if(!c->D.empty())estimator.D=cv::Mat(c->D).reshape(1,1).clone();});
      message_filters::Subscriber<sensor_msgs::Image> rgb(nh,"/tesse/left_cam/rgb/image_raw",5),depth(nh,"/tesse/depth_cam/mono/image_raw",5);
      message_filters::Subscriber<nav_msgs::Odometry> odom(nh,"/tesse/odom",100);
      using Policy=message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,nav_msgs::Odometry>;
      message_filters::Synchronizer<Policy> sync(Policy(100),rgb,depth,odom);
      boost::function<void(const sensor_msgs::ImageConstPtr&,const sensor_msgs::ImageConstPtr&,const nav_msgs::OdometryConstPtr&)> callback=[&](const sensor_msgs::ImageConstPtr& r,const sensor_msgs::ImageConstPtr& d,const nav_msgs::OdometryConstPtr& o){estimator.cameraCallback(r,d,bodyPose(o->pose.pose)*cameraExtrinsics());};
      sync.registerCallback(boost::bind(callback,_1,_2,_3));ros::spin();
    }
  } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
  return 0;
}
