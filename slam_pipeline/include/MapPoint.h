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

#include <map>
#include <opencv2/core/core.hpp>

#include "slam_pipeline_export.h"
#include "types.h"

namespace SLAM_PIPELINE {

class Map;

class SLAM_PIPELINE_EXPORT MapPoint {
 public:
  typedef std::map<KeyFramePtr, cv::Point2i, std::less<>> ObservationsMap;

  MapPoint(const cv::Mat& Pos, KeyFramePtr pRefKF, Map* pMap);
  MapPoint(const cv::Mat& Pos, Map* pMap, const Frame& frame);

  void SetWorldPos(const cv::Mat& Pos);
  cv::Mat GetWorldPos() const;

  cv::Mat GetNormal() const;
  KeyFramePtr GetReferenceKeyFrame();

  const ObservationsMap& GetObservations();
  int Observations();

  void AddObservation(KeyFramePtr pKF, const cv::Point2i& key);
  void EraseObservation(KeyFramePtr pKF);

  bool GetKeyPointInKeyFrame(const KeyFrame* pKF, cv::Point2i& key) const;
  bool IsInKeyFrame(const KeyFrame* pKF) const;
  bool IsInKeyFrame(KeyFramePtr pKF) const;

  void SetBadFlag();
  bool isBad();

  void Replace(MapPointPtr pMP);
  std::shared_ptr<MapPoint> GetReplaced();

  void IncreaseVisible(int n = 1);
  void IncreaseFound(int n = 1);
  float GetFoundRatio();
  inline int GetFound() { return mnFound; }

  void UpdateNormalAndDepth();

  float GetDistanceInvariance() const;

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

 protected:
  // Position in absolute coordinates
  cv::Mat mWorldPos;

  // Keyframes observing the point and associated index in keyframe
  ObservationsMap mObservations;

  // Mean viewing direction
  cv::Mat mNormalVector;

  // Reference KeyFrame
  KeyFramePtr mpRefKF;

  // Tracking counters
  int mnVisible;
  int mnFound;

  // Bad flag (we do not currently erase MapPoint from memory)
  bool mbBad;
  std::shared_ptr<MapPoint> mpReplaced;

  // invariance distance
  float mfDistance;

  Map* mpMap;
};

}  // namespace SLAM_PIPELINE

#endif  // MAPPOINT_H
