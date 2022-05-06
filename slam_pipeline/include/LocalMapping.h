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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "slam_pipeline_export.h"

namespace SLAM_PIPELINE {

class LoopClosing;
class Map;
struct FeatureParameters;

class SLAM_PIPELINE_EXPORT LocalMapping {
 public:
  LocalMapping(Map* pMap, FeatureMatcher* featureMatcher,
               const FeatureParameters& parameters);

  void SetLoopCloser(LoopClosing* pLoopCloser);

  // Main function
  void Run();

  void InsertKeyFrame(KeyFrame* pKF);

  void Reset();
  void Release();

  size_t KeyframesInQueue() { return mlNewKeyFrames.size(); }

 protected:
  bool CheckNewKeyFrames();
  void ProcessNewKeyFrame();
  void CreateNewMapPoints();

  void MapPointCulling();
  void SearchInNeighbors();

  void KeyFrameCulling();

  Map* mpMap;

  LoopClosing* mpLoopCloser;

  std::list<KeyFrame*> mlNewKeyFrames;

  KeyFrame* mpCurrentKeyFrame;

  std::list<MapPoint*> mlpRecentAddedMapPoints;

  FeatureMatcher* mFeatureMatcher;

  double mMinParallax;
};

}  // namespace SLAM_PIPELINE

#endif  // LOCALMAPPING_H
