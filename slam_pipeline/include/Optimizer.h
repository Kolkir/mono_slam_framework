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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include "slam_pipeline_export.h"

#include "FeatureMatcher.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"

namespace SLAM_PIPELINE {

class SLAM_PIPELINE_EXPORT Optimizer {
 public:
  void static BundleAdjustment(const std::vector<KeyFrame *> &vpKF,
                               const std::vector<MapPoint *> &vpMP,
                               int nIterations = 5, bool *pbStopFlag = NULL,
                               const unsigned long nLoopKF = 0,
                               const bool bRobust = true);
  void static GlobalBundleAdjustemnt(Map *pMap, int nIterations = 5,
                                     bool *pbStopFlag = NULL,
                                     const unsigned long nLoopKF = 0,
                                     const bool bRobust = true);
  void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap);
  int static PoseOptimization(Frame *pFrame);

  // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise
  // (mono)
  void static OptimizeEssentialGraph(
      Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
      const KeyFrameAndPose &NonCorrectedSim3,
      const KeyFrameAndPose &CorrectedSim3,
      const std::map<KeyFrame *, std::set<KeyFrame *> > &LoopConnections,
      const bool &bFixScale);

  // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
  static int OptimizeSim3(MatchFramesResult &vpMatches1, g2o::Sim3 &g2oS12,
                          const float th2, const bool bFixScale);
};

}  // namespace SLAM_PIPELINE

#endif  // OPTIMIZER_H
