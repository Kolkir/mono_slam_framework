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

#include "Frame.h"
#include "FrameBase.h"
#include "KeyFrameDatabase.h"
#include "KeyPointMap.h"
#include "MapPoint.h"
#include "slam_pipeline_export.h"

namespace SLAM_PIPELINE {

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;
class KeyFrameFactory;

class SLAM_PIPELINE_EXPORT KeyFrame : public FrameBase {
 protected:
  friend class KeyFrameFactory;
  KeyFrame(Frame& F, Map* pMap, KeyFrameDatabase* pKFDB);

 public:
  virtual ~KeyFrame() {}

  long unsigned int id() const override;

  // Covisibility graph functions
  void AddConnection(KeyFrame* pKF, const int& weight);
  void EraseConnection(KeyFrame* pKF);
  void UpdateConnections();
  void UpdateBestCovisibles();
  std::set<KeyFrame*> GetConnectedKeyFrames();
  std::vector<KeyFrame*> GetVectorCovisibleKeyFrames();
  std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int& N);
  std::vector<KeyFrame*> GetCovisiblesByWeight(const int& w);
  int GetWeight(KeyFrame* pKF);

  // Spanning tree functions
  void AddChild(KeyFrame* pKF);
  void EraseChild(KeyFrame* pKF);
  void ChangeParent(KeyFrame* pKF);
  std::set<KeyFrame*> GetChilds();
  KeyFrame* GetParent();
  bool hasChild(KeyFrame* pKF);

  // Loop Edges
  void AddLoopEdge(KeyFrame* pKF);
  std::set<KeyFrame*> GetLoopEdges();

  // MapPoint observation functions
  void AddMapPoint(MapPoint* pMP, const cv::Point2i& keyPoint);
  void EraseMapPointMatch(const cv::Point2i& keyPoint);
  void EraseMapPointMatch(MapPoint* pMP);
  void ReplaceMapPointMatch(const cv::Point2i& key, MapPoint* pMP);
  std::set<MapPoint*> GetMapPoints();
  KeyPointMap GetMapPointMatches();
  int TrackedMapPoints(const int& minObs);
  MapPoint* GetMapPoint(const cv::Point2i& keyPoint);

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

  static bool lId(KeyFrame* pKF1, KeyFrame* pKF2) {
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

  std::map<KeyFrame*, int> mConnectedKeyFrameWeights;
  std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
  std::vector<int> mvOrderedWeights;

  // Spanning Tree and Loop Edges
  bool mbFirstConnection;
  KeyFrame* mpParent;
  std::set<KeyFrame*> mspChildrens;
  std::set<KeyFrame*> mspLoopEdges;

  // Bad flags
  bool mbNotErase;
  bool mbToBeErased;
  bool mbBad;

  Map* mpMap;

  static long unsigned int nNextId;
  long unsigned int mnId;
};

class SLAM_PIPELINE_EXPORT KeyFrameFactory {
 public:
  virtual ~KeyFrameFactory() {}
  virtual KeyFrame* Create(Frame& F, Map* pMap, KeyFrameDatabase* pKFDB) const;
};
}  // namespace SLAM_PIPELINE

#endif  // KEYFRAME_H
