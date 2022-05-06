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

#include "Map.h"

namespace SLAM_PIPELINE {

Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0) {}

void Map::AddKeyFrame(KeyFrame *pKF) {
  mspKeyFrames.insert(pKF);
  if (pKF->id() > mnMaxKFid) mnMaxKFid = pKF->id();
}

void Map::AddMapPoint(MapPoint *pMP) {
  mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP) {
  mspMapPoints.erase(pMP);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF) {
  mspKeyFrames.erase(pKF);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::InformNewBigChange() {
  mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx() {
  return mnBigChangeIdx;
}

std::pair<size_t, size_t> Map::GoodBadMapPointsInMap() {
  size_t nbad = 0;
  size_t ngood = 0;
  for (auto pMP : mspMapPoints) {
    if (pMP->isBad())
      ++nbad;
    else
      ++ngood;
  }
  return {ngood, nbad};
}

std::vector<KeyFrame *> Map::GetAllKeyFrames() {
  return std::vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
}

std::vector<MapPoint *> Map::GetAllMapPoints() {
  return std::vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
}

size_t Map::MapPointsInMap() {
  return mspMapPoints.size();
}

size_t Map::KeyFramesInMap() {
  return mspKeyFrames.size();
}

long unsigned int Map::GetMaxKFid() {
  return mnMaxKFid;
}

void Map::clear() {
  for (std::set<MapPoint *>::iterator sit = mspMapPoints.begin(),
                                      send = mspMapPoints.end();
       sit != send; sit++)
    delete *sit;

  for (std::set<KeyFrame *>::iterator sit = mspKeyFrames.begin(),
                                      send = mspKeyFrames.end();
       sit != send; sit++)
    delete *sit;

  mspMapPoints.clear();
  mspKeyFrames.clear();
  mnMaxKFid = 0;
  mvpKeyFrameOrigins.clear();
}

}  // namespace SLAM_PIPELINE
