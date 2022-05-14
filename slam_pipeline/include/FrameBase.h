#ifndef FRAMEBASE_H
#define FRAMEBASE_H

#include <opencv2/opencv.hpp>

#include "KeyPointMap.h"
#include "slam_pipeline_export.h"

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
  cv::Mat GetPose() const;
  cv::Mat GetPoseInverse() const;
  cv::Mat GetCameraCenter() const;
  cv::Mat GetRotation() const;
  cv::Mat GetRotationInverse() const;
  cv::Mat GetTranslation() const;

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
};

}  // namespace SLAM_PIPELINE

#endif  // FRAMEBASE_H
