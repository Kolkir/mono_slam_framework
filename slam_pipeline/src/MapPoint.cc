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

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"

namespace SLAM_PIPELINE {

bool operator<(const KeyFramePtr& lhs, const KeyFrame* rhs) {
  return std::less<const KeyFrame*>()(lhs.get(), rhs);
}
bool operator<(const KeyFrame* lhs, const KeyFramePtr& rhs) {
  return std::less<const KeyFrame*>()(lhs, rhs.get());
}

long unsigned int MapPoint::nNextId = 0;

MapPoint::MapPoint(const cv::Mat& Pos, KeyFramePtr pRefKF, Map* pMap)
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
      mfDistance(0),
      mpMap(pMap) {
  Pos.copyTo(mWorldPos);
  mNormalVector = cv::Mat::zeros(3, 1, CV_32F);
  mnId = nNextId++;
}

MapPoint::MapPoint(const cv::Mat& Pos, Map* pMap, const Frame& frame)
    : mnFirstKFid(-1),
      mnFirstFrame(frame.id()),
      nObs(0),
      mnTrackReferenceForFrame(0),
      mnLastFrameSeen(0),
      mnBALocalForKF(0),
      mnFuseCandidateForKF(0),
      mnLoopPointForKF(0),
      mnCorrectedByKF(0),
      mnCorrectedReference(0),
      mnBAGlobalForKF(0),
      mnVisible(1),
      mnFound(1),
      mbBad(false),
      mpMap(pMap) {
  Pos.copyTo(mWorldPos);
  cv::Mat Ow = frame.GetCameraCenter();
  mNormalVector = mWorldPos - Ow;
  mNormalVector = mNormalVector / cv::norm(mNormalVector);

  cv::Mat PC = Pos - Ow;
  const float dist = static_cast<float>(cv::norm(PC));

  mfDistance = dist;

  mnId = nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat& Pos) { Pos.copyTo(mWorldPos); }

cv::Mat MapPoint::GetWorldPos() const { return mWorldPos.clone(); }

cv::Mat MapPoint::GetNormal() const { return mNormalVector.clone(); }

KeyFramePtr MapPoint::GetReferenceKeyFrame() { return mpRefKF; }

void MapPoint::AddObservation(KeyFramePtr pKF, const cv::Point2i& keyPoint) {
  if (mObservations.count(pKF)) return;
  mObservations[pKF] = keyPoint;
  nObs++;
}

void MapPoint::EraseObservation(KeyFramePtr pKF) {
  bool bBad = false;
  if (mObservations.count(pKF)) {
    auto key = mObservations[pKF];
    nObs--;

    mObservations.erase(pKF);

    if (mpRefKF == pKF) mpRefKF = mObservations.begin()->first;

    // If only 2 observations or less, discard point
    if (nObs <= 2) bBad = true;
  }

  if (bBad) SetBadFlag();
}

const MapPoint::ObservationsMap& MapPoint::GetObservations() {
  return mObservations;
}

int MapPoint::Observations() { return nObs; }

void MapPoint::SetBadFlag() {
  mbBad = true;
  auto obs = mObservations;
  mObservations.clear();
  for (auto mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
    KeyFramePtr pKF = mit->first;
    pKF->EraseMapPointMatch(mit->second);
  }

  mpMap->EraseMapPoint(this);
}

MapPointPtr MapPoint::GetReplaced() { return mpReplaced; }

void MapPoint::Replace(MapPointPtr pMP) {
  if (pMP->mnId == this->mnId) return;

  int nvisible, nfound;
  auto obs = mObservations;
  mObservations.clear();
  mbBad = true;
  nvisible = mnVisible;
  nfound = mnFound;
  mpReplaced = pMP;

  for (auto mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
    // Replace measurement in keyframe
    KeyFramePtr pKF = mit->first;

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

bool MapPoint::isBad() { return mbBad; }

void MapPoint::IncreaseVisible(int n) { mnVisible += n; }

void MapPoint::IncreaseFound(int n) { mnFound += n; }

float MapPoint::GetFoundRatio() {
  return static_cast<float>(mnFound) / mnVisible;
}

bool MapPoint::GetKeyPointInKeyFrame(const KeyFrame* pKF,
                                     cv::Point2i& keyPoint) const {
  auto i = mObservations.find(pKF);
  if (i != mObservations.end()) {
    keyPoint = i->second;
    return true;
  } else
    return false;
}

bool MapPoint::IsInKeyFrame(const KeyFrame* pKF) const {
  return mObservations.count(pKF);
}

bool MapPoint::IsInKeyFrame(KeyFramePtr pKF) const {
  return mObservations.count(pKF);
}

void MapPoint::UpdateNormalAndDepth() {
  if (mbBad) return;
  auto observations = mObservations;
  if (observations.empty()) return;
  auto pRefKF = mpRefKF;
  auto Pos = mWorldPos.clone();

  cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
  int n = 0;
  for (auto mit = observations.begin(), mend = observations.end(); mit != mend;
       mit++) {
    auto pKF = mit->first;
    cv::Mat Owi = pKF->GetCameraCenter();
    cv::Mat normali = mWorldPos - Owi;
    normal = normal + normali / cv::norm(normali);
    n++;
  }

  cv::Mat PC = Pos - pRefKF->GetCameraCenter();
  const float dist = static_cast<float>(cv::norm(PC));

  mfDistance = dist;
  mNormalVector = normal / n;
}

float MapPoint::GetDistanceInvariance() const { return 1.2f * mfDistance; }

}  // namespace SLAM_PIPELINE
