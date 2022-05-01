#ifndef FRAMEBASE_H
#define FRAMEBASE_H

#include "slam_pipeline_export.h"

#include <mutex>
#include <opencv2/opencv.hpp>

#include "KeyPointMap.h"

namespace SLAM_PIPELINE {

class SLAM_PIPELINE_EXPORT FrameBase {
 public:
  FrameBase() = default;
  FrameBase(const cv::Mat &imGray, const cv::Mat &K);
  FrameBase(const FrameBase &frame);
  FrameBase &operator=(const FrameBase &frame);

  virtual long unsigned int id() const = 0;

  float fx() const;
  float fy() const;
  float cx() const;
  float cy() const;

  // Pose functions
  void SetPose(const cv::Mat &Tcw);
  cv::Mat GetPose();
  cv::Mat GetPoseInverse();
  cv::Mat GetCameraCenter();
  cv::Mat GetRotation();
  cv::Mat GetRotationInverse();
  cv::Mat GetTranslation();

 public:
  // intrisic camera params
  cv::Mat mK;

  // Image Bounds
  float mnMinX{-1};
  float mnMaxX{-1};
  float mnMinY{-1};
  float mnMaxY{-1};

  // Frame image
  cv::Mat imGray;

  // Camera pose.
  cv::Mat mTcw;

  // MapPoints associated to keypoints
  KeyPointMap mKeyPointMap;

 protected:
  // Rotation, translation and camera center
  cv::Mat mTwc;  // pose inverse
  cv::Mat mRcw;  // roation
  cv::Mat mtcw;  // translation
  cv::Mat mRwc;  // rot inverse
  cv::Mat mOw;   // camera center

  std::mutex mMutexPose;
};

}  // namespace SLAM_PIPELINE

#endif  // FRAMEBASE_H
