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

#include "FeatureMatcher.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "Map.h"
#include "Tracking.h"

#ifdef _MSC_VER
#pragma warning(push, 0)
#else
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wall"
#pragma clang diagnostic ignored "-Wextra"
#endif

#include <g2o/types/sim3/types_seven_dof_expmap.h>

#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma clang diagnostic pop
#endif

namespace SLAM_PIPELINE {

typedef std::pair<std::set<KeyFrame*>, int> ConsistentGroup;
typedef std::map<
    KeyFrame*, g2o::Sim3, std::less<KeyFrame*>,
    Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > >
    KeyFrameAndPose;

class LoopClosing {
 public:
  LoopClosing(Map* pMap, KeyFrameDatabase* pDB, FeatureMatcher* featureMatcher,
              const FeatureParameters& parameters);

  void SetLocalMapper(LocalMapping* pLocalMapper);

  // Main function
  void Run();

  void InsertKeyFrame(KeyFrame* pKF);

  void Reset();

  // This function will run in a separate thread
  void RunGlobalBundleAdjustment(unsigned long nLoopKF);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  bool CheckNewKeyFrames();

  bool DetectLoop();

  bool ComputeSim3();

  void CorrectLoop();

  Map* mpMap;

  KeyFrameDatabase* mpKeyFrameDB;

  LocalMapping* mpLocalMapper;

  std::list<KeyFrame*> mlpLoopKeyFrameQueue;

  // Loop detector parameters
  float mnCovisibilityConsistencyTh;

  // Loop detector variables
  KeyFrame* mpCurrentKF;
  KeyFrame* mpMatchedKF;
  std::vector<ConsistentGroup> mvConsistentGroups;
  std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
  std::vector<KeyFrame*> mvpCurrentConnectedKFs;
  MatchFramesResult mvpCurrentMatchedPoints;
  KeyPointMap mvpLoopMapPoints;
  cv::Mat mScw;
  g2o::Sim3 mg2oScw;

  long unsigned int mLastLoopKFid;

  // Fix scale in the stereo/RGB-D case
  bool mbFixScale;

  int mLoopDetectionMaxFrames{10};
  int mMinSim3Matches{20};
  int mMinTotalSim3Matches{40};

  bool mnFullBAIdx;

  FeatureMatcher* mFeatureMatcher;
};

}  // namespace SLAM_PIPELINE

#endif  // LOOPCLOSING_H
