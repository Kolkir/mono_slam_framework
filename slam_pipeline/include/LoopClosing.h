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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include <list>

#include "SlamParameters.h"
#include "types.h"

namespace SLAM_PIPELINE {
class Map;
class FeatureMatcher;
class KeyFrameDatabase;
class LocalMapping;

class LoopClosing {
 public:
  LoopClosing(Map* pMap, KeyFrameDatabase* pDB, FeatureMatcher* featureMatcher,
              const SlamParameters& parameters);

  void SetLocalMapper(LocalMapping* pLocalMapper);

  // Main function
  void Run();

  void InsertKeyFrame(KeyFramePtr pKF);

  void Reset();

  // This function will run in a separate thread
  void RunGlobalBundleAdjustment(unsigned long nLoopKF);

 protected:
  bool CheckNewKeyFrames();

  bool DetectLoop();

  void CorrectLoop();

  Map* mpMap;

  KeyFrameDatabase* mpKeyFrameDB;

  LocalMapping* mpLocalMapper;

  std::list<KeyFramePtr> mlpLoopKeyFrameQueue;

  size_t mMinNumMPMatches{15};

  // Loop detector variables
  KeyFramePtr mpCurrentKF;
  KeyFramePtr mpMatchedKF;

  long unsigned int mLastLoopKFid;

  // Fix scale is not possible for monocular SLAM
  const bool mbFixScale{false};

  int mLoopDetectionMaxFrames{10};

  bool mnFullBAIdx;

  FeatureMatcher* mFeatureMatcher;
};

}  // namespace SLAM_PIPELINE

#endif  // LOOPCLOSING_H
