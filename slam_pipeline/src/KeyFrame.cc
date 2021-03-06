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

#include "KeyFrame.h"

#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "MapPoint.h"

namespace SLAM_PIPELINE {

long unsigned int KeyFrame::nNextId = 0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB)
    : FrameBase(F),
      mnFrameId(F.id()),
      mTimeStamp(F.mTimeStamp),
      mnTrackReferenceForFrame(0),
      mnFuseTargetForKF(0),
      mnBALocalForKF(0),
      mnBAFixedForKF(0),
      mnLoopQuery(0),
      mnRelocQuery(0),
      mnBAGlobalForKF(0),
      mpKeyFrameDB(pKFDB),
      mbFirstConnection(true),
      mpParent(NULL),
      mbNotErase(false),
      mbToBeErased(false),
      mbBad(false),
      mpMap(pMap),
      mRelocScore(0) {
  mnId = nNextId++;
}

long unsigned int KeyFrame::id() const { return mnId; }

void KeyFrame::AddConnection(KeyFramePtr pKF, const int &weight) {
  {
    if (!mConnectedKeyFrameWeights.count(pKF))
      mConnectedKeyFrameWeights[pKF] = weight;
    else if (mConnectedKeyFrameWeights[pKF] != weight)
      mConnectedKeyFrameWeights[pKF] = weight;
    else
      return;
  }

  UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles() {
  std::vector<std::pair<int, KeyFramePtr>> vPairs;
  vPairs.reserve(mConnectedKeyFrameWeights.size());
  for (std::map<KeyFramePtr, int>::iterator
           mit = mConnectedKeyFrameWeights.begin(),
           mend = mConnectedKeyFrameWeights.end();
       mit != mend; mit++)
    vPairs.push_back(std::make_pair(mit->second, mit->first));

  std::sort(vPairs.begin(), vPairs.end());
  std::list<KeyFramePtr> lKFs;
  std::list<int> lWs;
  for (size_t i = 0, iend = vPairs.size(); i < iend; i++) {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  mvpOrderedConnectedKeyFrames =
      std::vector<KeyFramePtr>(lKFs.begin(), lKFs.end());
  mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());
}

std::set<KeyFramePtr> KeyFrame::GetConnectedKeyFrames() const {
  std::set<KeyFramePtr> s;
  for (auto mit = mConnectedKeyFrameWeights.begin();
       mit != mConnectedKeyFrameWeights.end(); mit++)
    s.insert(mit->first);
  return s;
}

std::vector<KeyFramePtr> KeyFrame::GetVectorCovisibleKeyFrames() {
  return mvpOrderedConnectedKeyFrames;
}

std::vector<KeyFramePtr> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
  if ((int)mvpOrderedConnectedKeyFrames.size() < N)
    return mvpOrderedConnectedKeyFrames;
  else
    return std::vector<KeyFramePtr>(mvpOrderedConnectedKeyFrames.begin(),
                                    mvpOrderedConnectedKeyFrames.begin() + N);
}

std::vector<KeyFramePtr> KeyFrame::GetCovisiblesByWeight(const int &w) {
  if (mvpOrderedConnectedKeyFrames.empty()) return std::vector<KeyFramePtr>();

  std::vector<int>::iterator it =
      upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w,
                  KeyFrame::weightComp);
  if (it == mvOrderedWeights.end())
    return std::vector<KeyFramePtr>();
  else {
    auto n = it - mvOrderedWeights.begin();
    return std::vector<KeyFramePtr>(mvpOrderedConnectedKeyFrames.begin(),
                                    mvpOrderedConnectedKeyFrames.begin() + n);
  }
}

int KeyFrame::GetWeight(KeyFramePtr pKF) {
  if (mConnectedKeyFrameWeights.count(pKF))
    return mConnectedKeyFrameWeights[pKF];
  else
    return 0;
}

void KeyFrame::AddMapPoint(MapPointPtr pMP, const cv::Point2i &keyPoint) {
  mKeyPointMap.SetMapPoint(keyPoint, pMP);
}

void KeyFrame::EraseMapPointMatch(const cv::Point2i &keyPoint) {
  mKeyPointMap.SetMapPoint(keyPoint, nullptr);
}

void KeyFrame::EraseMapPointMatch(MapPointPtr pMP) {
  cv::Point2i keyPoint;
  if (pMP->GetKeyPointInKeyFrame(this, keyPoint)) {
    mKeyPointMap.SetMapPoint(keyPoint, nullptr);
  }
}

void KeyFrame::ReplaceMapPointMatch(const cv::Point2i &keyPoint,
                                    MapPointPtr pMP) {
  mKeyPointMap.SetMapPoint(keyPoint, pMP);
}

std::set<MapPointPtr> KeyFrame::GetMapPoints() {
  std::set<MapPointPtr> s;
  auto start = mKeyPointMap.Begin();
  auto end = mKeyPointMap.End();
  for (auto i = start; i != end; ++i) {
    if (!i->second.mapPoint) continue;
    MapPointPtr pMP = i->second.mapPoint;
    if (!pMP->isBad()) s.insert(pMP);
  }
  return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs) {
  int nPoints = 0;
  const bool bCheckObs = minObs > 0;
  auto start = mKeyPointMap.Begin();
  auto end = mKeyPointMap.End();
  for (auto i = start; i != end; ++i) {
    MapPointPtr pMP = i->second.mapPoint;
    if (pMP) {
      if (!pMP->isBad()) {
        if (bCheckObs) {
          if (pMP->Observations() >= minObs) nPoints++;
        } else
          nPoints++;
      }
    }
  }

  return nPoints;
}

KeyPointMap KeyFrame::GetMapPointMatches() { return mKeyPointMap; }

MapPointPtr KeyFrame::GetMapPoint(const cv::Point2i &keyPoint) {
  return mKeyPointMap.GetMapPoint(keyPoint);
}

void KeyFrame::UpdateConnections() {
  std::map<KeyFramePtr, int> KFcounter;

  KeyPointMap vpMP = mKeyPointMap;

  // For all map points in keyframe check in which other keyframes are they seen
  // Increase counter for those keyframes
  for (auto vit = vpMP.Begin(), vend = vpMP.End(); vit != vend; vit++) {
    MapPointPtr pMP = vit->second.mapPoint;

    if (!pMP) continue;

    if (pMP->isBad()) continue;

    auto observations = pMP->GetObservations();

    for (auto mit = observations.begin(), mend = observations.end();
         mit != mend; mit++) {
      if (mit->first->id() == mnId) continue;
      KFcounter[mit->first]++;
    }
  }

  // This should not happen
  if (KFcounter.empty()) return;

  // If the counter is greater than threshold add connection
  // In case no keyframe counter is over threshold add the one with maximum
  // counter
  int nmax = 0;
  KeyFramePtr pKFmax = NULL;
  int th = 15;

  std::vector<std::pair<int, KeyFramePtr>> vPairs;
  vPairs.reserve(KFcounter.size());
  for (std::map<KeyFramePtr, int>::iterator mit = KFcounter.begin(),
                                            mend = KFcounter.end();
       mit != mend; mit++) {
    if (mit->second > nmax) {
      nmax = mit->second;
      pKFmax = mit->first;
    }
    if (mit->second >= th) {
      vPairs.push_back(std::make_pair(mit->second, mit->first));
      mit->first->AddConnection(shared_from_this(), mit->second);
    }
  }

  if (vPairs.empty()) {
    vPairs.push_back(std::make_pair(nmax, pKFmax));
    pKFmax->AddConnection(shared_from_this(), nmax);
  }

  std::sort(vPairs.begin(), vPairs.end());
  std::list<KeyFramePtr> lKFs;
  std::list<int> lWs;
  for (size_t i = 0; i < vPairs.size(); i++) {
    lKFs.push_front(vPairs[i].second);
    lWs.push_front(vPairs[i].first);
  }

  mConnectedKeyFrameWeights = KFcounter;
  mvpOrderedConnectedKeyFrames =
      std::vector<KeyFramePtr>(lKFs.begin(), lKFs.end());
  mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());

  if (mbFirstConnection && mnId != 0) {
    mpParent = mvpOrderedConnectedKeyFrames.front();
    mpParent->AddChild(shared_from_this());
    mbFirstConnection = false;
  }
}

void KeyFrame::AddChild(KeyFramePtr pKF) { mspChildrens.insert(pKF); }

void KeyFrame::EraseChild(KeyFramePtr pKF) { mspChildrens.erase(pKF); }

void KeyFrame::ChangeParent(KeyFramePtr pKF) {
  mpParent = pKF;
  pKF->AddChild(shared_from_this());
}

std::set<KeyFramePtr> KeyFrame::GetChilds() { return mspChildrens; }

KeyFramePtr KeyFrame::GetParent() { return mpParent; }

bool KeyFrame::hasChild(KeyFramePtr pKF) { return mspChildrens.count(pKF); }

void KeyFrame::SetNotErase() { mbNotErase = true; }

void KeyFrame::SetErase() {
  if (mbToBeErased) {
    SetBadFlag();
  }
}

void KeyFrame::SetBadFlag() {
  if (mnId == 0)
    return;
  else if (mbNotErase) {
    mbToBeErased = true;
    return;
  }

  for (std::map<KeyFramePtr, int>::iterator
           mit = mConnectedKeyFrameWeights.begin(),
           mend = mConnectedKeyFrameWeights.end();
       mit != mend; mit++)
    mit->first->EraseConnection(shared_from_this());

  auto start = mKeyPointMap.Begin();
  auto end = mKeyPointMap.End();
  for (auto i = start; i != end; ++i) {
    i->second.mapPoint->EraseObservation(shared_from_this());
  }

  mConnectedKeyFrameWeights.clear();
  mvpOrderedConnectedKeyFrames.clear();

  // Update Spanning Tree
  std::set<KeyFramePtr> sParentCandidates;
  sParentCandidates.insert(mpParent);

  // Assign at each iteration one children with a parent (the pair with
  // highest covisibility weight) Include that children as new parent
  // candidate for the rest
  while (!mspChildrens.empty()) {
    bool bContinue = false;

    int max = -1;
    KeyFramePtr pC = nullptr;
    KeyFramePtr pP = nullptr;

    for (std::set<KeyFramePtr>::iterator sit = mspChildrens.begin(),
                                         send = mspChildrens.end();
         sit != send; sit++) {
      KeyFramePtr pKF = *sit;
      if (pKF->isBad()) continue;

      // Check if a parent candidate is connected to the keyframe
      std::vector<KeyFramePtr> vpConnected = pKF->GetVectorCovisibleKeyFrames();
      for (size_t i = 0, iend = vpConnected.size(); i < iend; i++) {
        for (std::set<KeyFramePtr>::iterator spcit = sParentCandidates.begin(),
                                             spcend = sParentCandidates.end();
             spcit != spcend; spcit++) {
          if (vpConnected[i]->id() == (*spcit)->id()) {
            int w = pKF->GetWeight(vpConnected[i]);
            if (w > max) {
              pC = pKF;
              pP = vpConnected[i];
              max = w;
              bContinue = true;
            }
          }
        }
      }
    }

    if (bContinue) {
      assert(pC);
      pC->ChangeParent(pP);
      sParentCandidates.insert(pC);
      mspChildrens.erase(pC);
    } else
      break;
  }

  // If a children has no covisibility links with any parent candidate, assign
  // to the original parent of this KF
  if (!mspChildrens.empty())
    for (std::set<KeyFramePtr>::iterator sit = mspChildrens.begin();
         sit != mspChildrens.end(); sit++) {
      (*sit)->ChangeParent(mpParent);
    }

  mpParent->EraseChild(shared_from_this());
  mTcp = mTcw * mpParent->GetPoseInverse();
  mbBad = true;

  mpMap->EraseKeyFrame(shared_from_this());
  mpKeyFrameDB->erase(shared_from_this());
}

bool KeyFrame::isBad() { return mbBad; }

void KeyFrame::EraseConnection(KeyFramePtr pKF) {
  bool bUpdate = false;
  if (mConnectedKeyFrameWeights.count(pKF)) {
    mConnectedKeyFrameWeights.erase(pKF);
    bUpdate = true;
  }

  if (bUpdate) UpdateBestCovisibles();
}

bool KeyFrame::IsInImage(const float &x, const float &y) const {
  return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
}

float KeyFrame::ComputeSceneMedianDepth(const int q) {
  KeyPointMap mapPoints;
  cv::Mat Tcw_;
  mapPoints = mKeyPointMap;
  Tcw_ = mTcw.clone();

  std::vector<double> vDepths;
  auto startMapPoint = mapPoints.Begin();
  auto endMapPoint = mapPoints.End();
  size_t N = std::distance(endMapPoint, startMapPoint);
  vDepths.reserve(N);
  cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
  Rcw2 = Rcw2.t();
  float zcw = Tcw_.at<float>(2, 3);
  for (auto i = startMapPoint; i != endMapPoint; ++i) {
    MapPointPtr pMP = i->second.mapPoint;
    cv::Mat x3Dw = pMP->GetWorldPos();
    double z = Rcw2.dot(x3Dw) + zcw;
    vDepths.push_back(z);
  }

  std::sort(vDepths.begin(), vDepths.end());

  return static_cast<float>(vDepths[(vDepths.size() - 1) / q]);
}

KeyFramePtr KeyFrameFactory::Create(Frame &F, Map *pMap,
                                    KeyFrameDatabase *pKFDB) const {
  return KeyFramePtr(new KeyFrame(F, pMap, pKFDB));
}
}  // namespace SLAM_PIPELINE
