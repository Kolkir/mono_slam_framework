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

#include <opencv2/opencv.hpp>
#include <vector>

#include "types.h"
#include "FrameBase.h"
#include "slam_pipeline_export.h"

namespace SLAM_PIPELINE {

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

  long unsigned int id() const override;

  // Check if a MapPoint is in the frustum of the camera
  bool isInFrustum(const MapPoint& MP, float viewingCosLimit);

 public:
  // Frame timestamp.
  double mTimeStamp;

  // Reference Keyframe.
  KeyFramePtr mpReferenceKF;

 private:
  // Current and Next Frame id.
  static long unsigned int nNextId;
  long unsigned int mnId;
};

class SLAM_PIPELINE_EXPORT FrameFactory {
 public:
  virtual ~FrameFactory() {}
  virtual FramePtr Create(const cv::Mat &imGray, const double &timeStamp,
                          cv::Mat &K) const;
  virtual FramePtr Clone(const Frame& frame) const;
};

}  // namespace SLAM_PIPELINE

#endif  // FRAME_H
