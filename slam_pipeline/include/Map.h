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

#ifndef MAP_H
#define MAP_H

#include <set>
#include <vector>

#include "slam_pipeline_export.h"
#include "types.h"

namespace SLAM_PIPELINE {

class SLAM_PIPELINE_EXPORT Map {
 public:
  Map();

  void AddKeyFrame(KeyFramePtr pKF);
  void AddMapPoint(MapPointPtr pMP);
  void EraseMapPoint(MapPointPtr pMP);
  void EraseMapPoint(MapPoint* pMP);
  void EraseKeyFrame(KeyFramePtr pKF);
  void InformNewBigChange();
  int GetLastBigChangeIdx();

  std::pair<size_t, size_t> GoodBadMapPointsInMap();

  std::vector<std::shared_ptr<KeyFrame>> GetAllKeyFrames();
  std::vector<std::shared_ptr<MapPoint>> GetAllMapPoints();

  size_t MapPointsInMap();
  size_t KeyFramesInMap();

  long unsigned int GetMaxKFid();

  void clear();

  std::vector<KeyFramePtr> mvpKeyFrameOrigins;

 protected:
  std::set<MapPointPtr> mspMapPoints;
  std::set<KeyFramePtr> mspKeyFrames;

  long unsigned int mnMaxKFid;

  // Index related to a big change in the map (loop closure, global BA)
  int mnBigChangeIdx;
};

}  // namespace SLAM_PIPELINE

#endif  // MAP_H
