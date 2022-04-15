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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "slam_pipeline_export.h"

#include <mutex>
#include <opencv2/core/core.hpp>

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"

namespace SLAM_PIPELINE {

class KeyFrame;
class Map;
class Frame;

class SLAM_PIPELINE_EXPORT MapPoint {
 public:
  MapPoint(const cv::Mat& Pos, KeyFrame* pRefKF, Map* pMap);
  MapPoint(const cv::Mat& Pos, Map* pMap, Frame* pFrame);

  void SetWorldPos(const cv::Mat& Pos);
  cv::Mat GetWorldPos();

  cv::Mat GetNormal();
  KeyFrame* GetReferenceKeyFrame();

  const std::map<KeyFrame*, cv::Point2i>& GetObservations();
  int Observations();

  void AddObservation(KeyFrame* pKF, const cv::Point2i& key);
  void EraseObservation(KeyFrame* pKF);

  bool GetKeyPointInKeyFrame(KeyFrame* pKF, cv::Point2i& key);
  bool IsInKeyFrame(KeyFrame* pKF);

  void SetBadFlag();
  bool isBad();

  void Replace(MapPoint* pMP);
  MapPoint* GetReplaced();

  void IncreaseVisible(int n = 1);
  void IncreaseFound(int n = 1);
  float GetFoundRatio();
  inline int GetFound() { return mnFound; }

  void UpdateNormalAndDepth();

  float GetDistanceInvariance();

 public:
  long unsigned int mnId;
  static long unsigned int nNextId;
  long int mnFirstKFid;
  long int mnFirstFrame;
  int nObs;

  // Variables used by the tracking
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnLastFrameSeen;

  // Variables used by local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnFuseCandidateForKF;

  // Variables used by loop closing
  long unsigned int mnLoopPointForKF;
  long unsigned int mnCorrectedByKF;
  long unsigned int mnCorrectedReference;
  cv::Mat mPosGBA;
  long unsigned int mnBAGlobalForKF;

  static std::mutex mGlobalMutex;

 protected:
  // Position in absolute coordinates
  cv::Mat mWorldPos;

  // Keyframes observing the point and associated index in keyframe
  std::map<KeyFrame*, cv::Point2i> mObservations;

  // Mean viewing direction
  cv::Mat mNormalVector;

  // Reference KeyFrame
  KeyFrame* mpRefKF;

  // Tracking counters
  int mnVisible;
  int mnFound;

  // Bad flag (we do not currently erase MapPoint from memory)
  bool mbBad;
  MapPoint* mpReplaced;

  // invariance distance
  float mfDistance;

  Map* mpMap;

  std::mutex mMutexPos;
  std::mutex mMutexFeatures;
};

}  // namespace SLAM_PIPELINE

#endif  // MAPPOINT_H
