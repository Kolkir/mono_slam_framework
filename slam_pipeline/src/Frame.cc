/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Frame.h"

#include <thread>

#include "Converter.h"
#include "FeatureMatcher.h"

namespace SLAM_PIPELINE {

long unsigned int Frame::nNextId = 0;

Frame::Frame() {}

// Copy Constructor
Frame::Frame(const Frame &frame)
    : FrameBase(frame),
      mTimeStamp(frame.mTimeStamp),
      mnId(frame.mnId),
      mpReferenceKF(frame.mpReferenceKF) {
}

Frame::Frame(const cv::Mat &imGray, const double &timeStamp, cv::Mat &K)
    : FrameBase(imGray, K), mTimeStamp(timeStamp) {
  // Frame ID
  mnId = nNextId++;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit) {
  // 3D in absolute coordinates
  cv::Mat P = pMP->GetWorldPos();

  // 3D in camera coordinates
  const cv::Mat Pc = mRcw * P + mtcw;
  const float &PcX = Pc.at<float>(0);
  const float &PcY = Pc.at<float>(1);
  const float &PcZ = Pc.at<float>(2);

  // Check positive depth
  if (PcZ < 0.0f) return false;

  // Project in image and check it is not outside
  const float invz = 1.0f / PcZ;
  const float u = fx() * PcX * invz + cx();
  const float v = fy() * PcY * invz + cy();

  if (u < mnMinX || u > mnMaxX) return false;
  if (v < mnMinY || v > mnMaxY) return false;

  // Check distance is in the scale invariance region of the MapPoint
  const float distance = pMP->GetDistanceInvariance();
  const cv::Mat PO = P - mOw;
  const float dist = static_cast<float>(cv::norm(PO));

  if (dist < 0 || dist > distance) return false;

  // Check viewing angle
  cv::Mat Pn = pMP->GetNormal();

  const float viewCos = static_cast<float>(PO.dot(Pn) / dist);

  if (viewCos < viewingCosLimit) return false;

  return true;
}

Frame *FrameFactory::Create(const cv::Mat &imGray, const double &timeStamp,
                            cv::Mat &K) const {
  return new Frame(imGray, timeStamp, K);
}
Frame *FrameFactory::Clone(Frame *frame) const { return new Frame(*frame); }

}  // namespace SLAM_PIPELINE
