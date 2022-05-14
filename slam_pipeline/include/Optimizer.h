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

#include <vector>

#include "slam_pipeline_export.h"
#include "types.h"

namespace SLAM_PIPELINE {

class Map;
struct MatchFramesResult;

class SLAM_PIPELINE_EXPORT Optimizer {
 public:
  void static BundleAdjustment(const std::vector<KeyFramePtr> &vpKF,
                               const std::vector<MapPointPtr> &vpMP,
                               int nIterations = 5, bool *pbStopFlag = NULL,
                               const unsigned long nLoopKF = 0,
                               const bool bRobust = true);

  void static GlobalBundleAdjustemnt(Map *pMap, int nIterations = 5,
                                     bool *pbStopFlag = NULL,
                                     const unsigned long nLoopKF = 0,
                                     const bool bRobust = true);

  void static LocalBundleAdjustment(KeyFramePtr pKF, bool *pbStopFlag);

  int static PoseOptimization(FramePtr pFrame);
};

}  // namespace SLAM_PIPELINE

#endif  // OPTIMIZER_H
