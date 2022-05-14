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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <memory>

#include "FrameBase.h"
#include "KeyPointMap.h"
#include "slam_pipeline_export.h"
#include "types.h"

namespace SLAM_PIPELINE {

class KeyFrameDatabase;
class Map;

class SLAM_PIPELINE_EXPORT KeyFrame
    : public FrameBase,
      public std::enable_shared_from_this<KeyFrame> {
 protected:
  friend class KeyFrameFactory;
  KeyFrame(Frame& F, Map* pMap, KeyFrameDatabase* pKFDB);

 public:
  virtual ~KeyFrame() {}

  long unsigned int id() const override;

  // Covisibility graph functions
  void AddConnection(KeyFramePtr pKF, const int& weight);
  void EraseConnection(KeyFramePtr pKF);
  void UpdateConnections();
  void UpdateBestCovisibles();
  std::set<KeyFramePtr> GetConnectedKeyFrames() const;
  std::vector<KeyFramePtr> GetVectorCovisibleKeyFrames();
  std::vector<KeyFramePtr> GetBestCovisibilityKeyFrames(const int& N);
  std::vector<KeyFramePtr> GetCovisiblesByWeight(const int& w);
  int GetWeight(KeyFramePtr pKF);

  // Spanning tree functions
  void AddChild(KeyFramePtr pKF);
  void EraseChild(KeyFramePtr pKF);
  void ChangeParent(KeyFramePtr pKF);
  std::set<KeyFramePtr> GetChilds();
  KeyFramePtr GetParent();
  bool hasChild(KeyFramePtr pKF);

  // MapPoint observation functions
  void AddMapPoint(MapPointPtr pMP, const cv::Point2i& keyPoint);
  void EraseMapPointMatch(const cv::Point2i& keyPoint);
  void EraseMapPointMatch(MapPointPtr pMP);
  void ReplaceMapPointMatch(const cv::Point2i& key, MapPointPtr pMP);
  std::set<MapPointPtr> GetMapPoints();
  KeyPointMap GetMapPointMatches();
  int TrackedMapPoints(const int& minObs);
  MapPointPtr GetMapPoint(const cv::Point2i& keyPoint);

  // Image
  bool IsInImage(const float& x, const float& y) const;

  // Enable/Disable bad flag changes
  void SetNotErase();
  void SetErase();

  // Set/check bad flag
  void SetBadFlag();
  bool isBad();

  // Compute Scene Depth (q=2 median). Used in monocular.
  float ComputeSceneMedianDepth(const int q);

  static bool weightComp(int a, int b) { return a > b; }

  static bool lId(KeyFramePtr pKF1, KeyFramePtr pKF2) {
    return pKF1->id() < pKF2->id();
  }

 public:
  const long unsigned int mnFrameId;

  const double mTimeStamp;

  // Variables used by the tracking
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnFuseTargetForKF;

  // Variables used by the local mapping
  long unsigned int mnBALocalForKF;
  long unsigned int mnBAFixedForKF;

  // Variables used by the keyframe database
  long unsigned int mnLoopQuery;
  size_t mnLoopMatches;

  long unsigned int mnRelocQuery;
  float mRelocScore;

  // Variables used by loop closing
  cv::Mat mTcwGBA;
  cv::Mat mTcwBefGBA;
  long unsigned int mnBAGlobalForKF;

  // Pose relative to parent (this is computed when bad flag is activated)
  cv::Mat mTcp;

 private:
  KeyFrameDatabase* mpKeyFrameDB;

  std::map<KeyFramePtr, int> mConnectedKeyFrameWeights;
  std::vector<KeyFramePtr> mvpOrderedConnectedKeyFrames;
  std::vector<int> mvOrderedWeights;

  // Spanning Tree and Loop Edges
  bool mbFirstConnection;
  KeyFramePtr mpParent;
  std::set<KeyFramePtr> mspChildrens;

  // Bad flags
  bool mbNotErase;
  bool mbToBeErased;
  bool mbBad;

  Map* mpMap;

  static long unsigned int nNextId;
  long unsigned int mnId;
};

typedef KeyFramePtr KeyFramePtr;

class SLAM_PIPELINE_EXPORT KeyFrameFactory {
 public:
  virtual ~KeyFrameFactory() {}
  virtual KeyFramePtr Create(Frame& F, Map* pMap,
                             KeyFrameDatabase* pKFDB) const;
};
}  // namespace SLAM_PIPELINE

#endif  // KEYFRAME_H
