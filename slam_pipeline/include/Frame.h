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

#ifndef FRAME_H
#define FRAME_H

#include "slam_pipeline_export.h"

#include <opencv2/opencv.hpp>
#include <vector>

#include "FrameBase.h"
#include "KeyFrame.h"
#include "KeyPointMap.h"
#include "MapPoint.h"

namespace SLAM_PIPELINE {
class MapPoint;
class KeyFrame;
class FrameFactory;

class SLAM_PIPELINE_EXPORT Frame : public FrameBase {
 protected:
  friend class FrameFactory;
  Frame();

  // Constructor for Monocular cameras.
  Frame(const cv::Mat &imGray, const double &timeStamp, cv::Mat &K);

 public:
  // Copy constructor.
  Frame(const Frame &frame);

  virtual ~Frame() {}

  // Check if a MapPoint is in the frustum of the camera
  // and fill variables of the MapPoint to be used by the tracking
  bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

 public:
  // Frame timestamp.
  double mTimeStamp;

  // Current and Next Frame id.
  static long unsigned int nNextId;
  long unsigned int mnId;

  // Reference Keyframe.
  KeyFrame *mpReferenceKF;
};

class SLAM_PIPELINE_EXPORT FrameFactory {
 public:
  virtual ~FrameFactory() {}
  virtual Frame *Create(const cv::Mat &imGray, const double &timeStamp,
                        cv::Mat &K) const;
  virtual Frame *Clone(Frame *frame) const;
};

}  // namespace SLAM_PIPELINE

#endif  // FRAME_H
