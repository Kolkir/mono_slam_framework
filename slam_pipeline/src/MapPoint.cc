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

#include "MapPoint.h"

#include <mutex>

#include "FeatureMatcher.h"

namespace SLAM_PIPELINE {

long unsigned int MapPoint::nNextId = 0;
std::mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat& Pos, KeyFrame* pRefKF, Map* pMap)
    : mnFirstKFid(pRefKF->id()),
      mnFirstFrame(pRefKF->mnFrameId),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mpRefKF(pRefKF),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(static_cast<MapPoint*>(NULL)),
      mfDistance(0),
      mpMap(pMap) {
  Pos.copyTo(mWorldPos);
  mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

MapPoint::MapPoint(const cv::Mat& Pos, Map* pMap, Frame* pFrame)
    : mnFirstKFid(-1),
      mnFirstFrame(pFrame->id()),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mpRefKF(static_cast<KeyFrame*>(NULL)),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpReplaced(NULL),
      mpMap(pMap) {
  Pos.copyTo(mWorldPos);
  cv::Mat Ow = pFrame->GetCameraCenter();
  mNormalVector = mWorldPos - Ow;
  mNormalVector = mNormalVector / cv::norm(mNormalVector);

  cv::Mat PC = Pos - Ow;
  const float dist = static_cast<float>(cv::norm(PC));

  mfDistance = dist;

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
  mnId = nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat& Pos) {
  std::unique_lock<std::mutex> lock2(mGlobalMutex);
  std::unique_lock<std::mutex> lock(mMutexPos);
  Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos() {
  std::unique_lock<std::mutex> lock(mMutexPos);
  return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal() {
  std::unique_lock<std::mutex> lock(mMutexPos);
  return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, const cv::Point2i& keyPoint) {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  if (mObservations.count(pKF)) return;
  mObservations[pKF] = keyPoint;
  nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF) {
  bool bBad = false;
  {
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    if (mObservations.count(pKF)) {
      auto key = mObservations[pKF];
      nObs--;

      mObservations.erase(pKF);

      if (mpRefKF == pKF) mpRefKF = mObservations.begin()->first;

      // If only 2 observations or less, discard point
      if (nObs <= 2) bBad = true;
    }
  }

  if (bBad) SetBadFlag();
}

const std::map<KeyFrame*, cv::Point2i>& MapPoint::GetObservations() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return mObservations;
}

int MapPoint::Observations() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return nObs;
}

void MapPoint::SetBadFlag() {
  std::map<KeyFrame*, cv::Point2i> obs;
  {
    std::unique_lock<std::mutex> lock1(mMutexFeatures);
    std::unique_lock<std::mutex> lock2(mMutexPos);
    mbBad = true;
    obs = mObservations;
    mObservations.clear();
  }
  for (auto mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
    KeyFrame* pKF = mit->first;
    pKF->EraseMapPointMatch(mit->second);
  }

  mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced() {
  std::unique_lock<std::mutex> lock1(mMutexFeatures);
  std::unique_lock<std::mutex> lock2(mMutexPos);
  return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP) {
  if (pMP->mnId == this->mnId) return;

  int nvisible, nfound;
  std::map<KeyFrame*, cv::Point2i> obs;
  {
    std::unique_lock<std::mutex> lock1(mMutexFeatures);
    std::unique_lock<std::mutex> lock2(mMutexPos);
    obs = mObservations;
    mObservations.clear();
    mbBad = true;
    nvisible = mnVisible;
    nfound = mnFound;
    mpReplaced = pMP;
  }

  for (auto mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
    // Replace measurement in keyframe
    KeyFrame* pKF = mit->first;

    if (!pMP->IsInKeyFrame(pKF)) {
      pKF->ReplaceMapPointMatch(mit->second, pMP);
      pMP->AddObservation(pKF, mit->second);
    } else {
      pKF->EraseMapPointMatch(mit->second);
    }
  }
  pMP->IncreaseFound(nfound);
  pMP->IncreaseVisible(nvisible);

  mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  std::unique_lock<std::mutex> lock2(mMutexPos);
  return mbBad;
}

void MapPoint::IncreaseVisible(int n) {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  mnVisible += n;
}

void MapPoint::IncreaseFound(int n) {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  mnFound += n;
}

float MapPoint::GetFoundRatio() {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return static_cast<float>(mnFound) / mnVisible;
}

bool MapPoint::GetKeyPointInKeyFrame(KeyFrame* pKF, cv::Point2i& keyPoint) {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  if (mObservations.count(pKF)) {
    keyPoint = mObservations[pKF];
    return true;
  } else
    return false;
}

bool MapPoint::IsInKeyFrame(KeyFrame* pKF) {
  std::unique_lock<std::mutex> lock(mMutexFeatures);
  return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth() {
  std::map<KeyFrame*, cv::Point2i> observations;
  KeyFrame* pRefKF;
  cv::Mat Pos;
  {
    std::unique_lock<std::mutex> lock1(mMutexFeatures);
    std::unique_lock<std::mutex> lock2(mMutexPos);
    if (mbBad) return;
    observations = mObservations;
    pRefKF = mpRefKF;
    Pos = mWorldPos.clone();
  }

  if (observations.empty()) return;

  cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
  int n = 0;
  for (auto mit = observations.begin(), mend = observations.end(); mit != mend;
       mit++) {
    KeyFrame* pKF = mit->first;
    cv::Mat Owi = pKF->GetCameraCenter();
    cv::Mat normali = mWorldPos - Owi;
    normal = normal + normali / cv::norm(normali);
    n++;
  }

  cv::Mat PC = Pos - pRefKF->GetCameraCenter();
  const float dist = static_cast<float>(cv::norm(PC));

  {
    std::unique_lock<std::mutex> lock3(mMutexPos);
    mfDistance = dist;
    mNormalVector = normal / n;
  }
}

float MapPoint::GetDistanceInvariance() {
  std::unique_lock<std::mutex> lock(mMutexPos);
  return 1.2f * mfDistance;
}

}  // namespace SLAM_PIPELINE
