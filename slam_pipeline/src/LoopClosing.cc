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

#include "LoopClosing.h"

#include "Converter.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "MapPoint.h"
#include "Optimizer.h"
#include "LocalMapping.h"

namespace SLAM_PIPELINE {

LoopClosing::LoopClosing(Map* pMap, KeyFrameDatabase* pDB,
                         FeatureMatcher* featureMatcher,
                         const SlamParameters& parameters)
    : mpMap(pMap),
      mpKeyFrameDB(pDB),
      mpMatchedKF(NULL),
      mLastLoopKFid(0),
      mnFullBAIdx(false),
      mFeatureMatcher(featureMatcher) {
  mLoopDetectionMaxFrames = parameters.loopDetectionMaxFrames;
  mMinNumMPMatches = parameters.minNumMPMatches;
}

void LoopClosing::SetLocalMapper(LocalMapping* pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void LoopClosing::Run() {
  // Check if there are keyframes in the queue
  if (CheckNewKeyFrames()) {
    // Detect loop candidates
    if (DetectLoop()) {
      // Perform pose graph optimization
      CorrectLoop();
    }
  }
}

void LoopClosing::InsertKeyFrame(KeyFramePtr pKF) {
  if (pKF->id() != 0) mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames() {
  return (!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::DetectLoop() {
  mpCurrentKF = mlpLoopKeyFrameQueue.front();
  mlpLoopKeyFrameQueue.pop_front();
  // Avoid that a keyframe can be erased while it is being process by this
  // thread
  mpCurrentKF->SetNotErase();

  // If the map contains less than 10 KF or less than 10 KF have passed from
  // last loop detection
  if (mpCurrentKF->id() < mLastLoopKFid + mLoopDetectionMaxFrames) {
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
  }

  // Query the database imposing the minimum num matches
  KeyFramePtr candidateKF =
      mpKeyFrameDB->DetectLoopCandidate(*mpCurrentKF, mMinNumMPMatches);

  // If there are no loop candidates, just add new keyframe and return false
  if (!candidateKF) {
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
  }

  mpMatchedKF = candidateKF;

  mpCurrentKF->SetErase();
  return true;
}

void LoopClosing::CorrectLoop() {
  std::cout << "Loop detected!" << std::endl;

  mnFullBAIdx = true;

  // Ensure current keyframe is updated
  mpCurrentKF->UpdateConnections();

  RunGlobalBundleAdjustment(mpCurrentKF->id());

  // Loop closed. Release Local Mapping.
  mpLocalMapper->Release();

  mLastLoopKFid = mpCurrentKF->id();
}

void LoopClosing::Reset() {
  mlpLoopKeyFrameQueue.clear();
  mLastLoopKFid = 0;
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF) {
  std::cout << "Starting Global Bundle Adjustment" << std::endl;

  bool idx = mnFullBAIdx;
  bool bStopGBA = false;
  Optimizer::GlobalBundleAdjustemnt(mpMap, 10, &bStopGBA, nLoopKF, false);

  // Update all MapPoints and KeyFrames
  // Local Mapping was active during BA, that means that there might be new
  // keyframes not included in the Global BA and they are not consistent with
  // the updated map. We need to propagate the correction through the spanning
  // tree
  if (idx != mnFullBAIdx) return;

  std::cout << "Global Bundle Adjustment finished" << std::endl;
  std::cout << "Updating map ..." << std::endl;

  // Correct keyframes starting at map first keyframe
  std::list<KeyFramePtr> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),
                                     mpMap->mvpKeyFrameOrigins.end());

  while (!lpKFtoCheck.empty()) {
    KeyFramePtr pKF = lpKFtoCheck.front();
    const std::set<KeyFramePtr> sChilds = pKF->GetChilds();
    cv::Mat Twc = pKF->GetPoseInverse();
    for (std::set<KeyFramePtr>::const_iterator sit = sChilds.begin();
         sit != sChilds.end(); sit++) {
      KeyFramePtr pChild = *sit;
      if (pChild->mnBAGlobalForKF != nLoopKF) {
        cv::Mat Tchildc = pChild->GetPose() * Twc;
        pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;  //*Tcorc*pKF->mTcwGBA;
        pChild->mnBAGlobalForKF = nLoopKF;
      }
      lpKFtoCheck.push_back(pChild);
    }

    pKF->mTcwBefGBA = pKF->GetPose();
    pKF->SetPose(pKF->mTcwGBA);
    lpKFtoCheck.pop_front();
  }

  // Correct MapPoints
  const std::vector<MapPointPtr> vpMPs = mpMap->GetAllMapPoints();

  for (size_t i = 0; i < vpMPs.size(); i++) {
    MapPointPtr pMP = vpMPs[i];

    if (pMP->isBad()) continue;

    if (pMP->mnBAGlobalForKF == nLoopKF) {
      // If optimized by Global BA, just update
      pMP->SetWorldPos(pMP->mPosGBA);
    } else {
      // Update according to the correction of its reference keyframe
      KeyFramePtr pRefKF = pMP->GetReferenceKeyFrame();

      if (pRefKF->mnBAGlobalForKF != nLoopKF) continue;

      // Map to non-corrected camera
      cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0, 3).colRange(0, 3);
      cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0, 3).col(3);
      cv::Mat Xc = Rcw * pMP->GetWorldPos() + tcw;

      // Backproject using corrected camera
      cv::Mat Twc = pRefKF->GetPoseInverse();
      cv::Mat Rwc = Twc.rowRange(0, 3).colRange(0, 3);
      cv::Mat twc = Twc.rowRange(0, 3).col(3);

      pMP->SetWorldPos(Rwc * Xc + twc);
    }
  }

  mpMap->InformNewBigChange();

  mpLocalMapper->Release();

  std::cout << "Map updated!" << std::endl;
}

}  // namespace SLAM_PIPELINE
