#include "FrameBase.h"

namespace SLAM_PIPELINE {

FrameBase::FrameBase(const cv::Mat &imGray, const cv::Mat &K)
    : mKeyPointMap(imGray.cols, imGray.rows),
      imGray(imGray.clone()),
      mK(K.clone()) {
  // image bounds
  mnMinX = 0.0f;
  mnMaxX = static_cast<float>(imGray.cols);
  mnMinY = 0.0f;
  mnMaxY = static_cast<float>(imGray.rows);
}

FrameBase::FrameBase(const FrameBase &frame)
    : mKeyPointMap(frame.mKeyPointMap),
      imGray(frame.imGray.clone()),
      mK(frame.mK.clone()) {
  // image bounds
  mnMinX = 0.0f;
  mnMaxX = static_cast<float>(imGray.cols);
  mnMinY = 0.0f;
  mnMaxY = static_cast<float>(imGray.rows);

  if (!frame.mTcw.empty()) SetPose(frame.mTcw);
}

FrameBase &FrameBase::operator=(const FrameBase &frame) {
  mKeyPointMap = frame.mKeyPointMap;
  imGray = frame.imGray.clone();

  // image bounds
  mnMinX = 0.0f;
  mnMaxX = static_cast<float>(imGray.cols);
  mnMinY = 0.0f;
  mnMaxY = static_cast<float>(imGray.rows);

  mK = frame.mK.clone();

  if (!frame.mTcw.empty()) SetPose(frame.mTcw);
  return *this;
}

float FrameBase::fx() const { return mK.at<float>(0, 0); }

float FrameBase::fy() const { return mK.at<float>(1, 1); }

float FrameBase::cx() const { return mK.at<float>(0, 2); }

float FrameBase::cy() const { return mK.at<float>(1, 2); }

void FrameBase::SetPose(const cv::Mat &Tcw_) {
  std::unique_lock<std::mutex> lock(mMutexPose);
  Tcw_.copyTo(mTcw);
  mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
  mRwc = mRcw.t();
  mtcw = mTcw.rowRange(0, 3).col(3);
  mOw = -mRwc * mtcw;

  mTwc = cv::Mat::eye(4, 4, mTcw.type());
  mRwc.copyTo(mTwc.rowRange(0, 3).colRange(0, 3));
  mOw.copyTo(mTwc.rowRange(0, 3).col(3));
}

cv::Mat FrameBase::GetPose() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mTcw.clone();
}

cv::Mat FrameBase::GetPoseInverse() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mTwc.clone();
}

cv::Mat FrameBase::GetCameraCenter() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mOw.clone();
}

cv::Mat FrameBase::GetRotation() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mRcw.clone();
}

cv::Mat FrameBase::GetTranslation() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mtcw.clone();
}

cv::Mat FrameBase::GetRotationInverse() {
  std::unique_lock<std::mutex> lock(mMutexPose);
  return mRwc.clone();
}

}  // namespace SLAM_PIPELINE