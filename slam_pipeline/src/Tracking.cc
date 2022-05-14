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

#include "Tracking.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Converter.h"
#include "FeatureMatcher.h"
#include "Initializer.h"
#include "KeyFrame.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapDrawer.h"
#include "MapPoint.h"
#include "Optimizer.h"
#include "PnPsolver.h"
#include "KeyFrameDatabase.h"

using namespace std;

namespace SLAM_PIPELINE {

Tracking::Tracking(MapDrawer* pMapDrawer, Map* pMap, KeyFrameDatabase* pKFDB,
                   const SlamParameters& parameters,
                   FeatureMatcher* featureMatcher, FrameFactory* frameFactory,
                   KeyFrameFactory* keyFrameFactory)
    : mState(NO_IMAGES_YET),
      mpKeyFrameDB(pKFDB),
      mpInitializer(static_cast<Initializer*>(NULL)),
      mpMapDrawer(pMapDrawer),
      mpMap(pMap),
      mnLastRelocFrameId(0),
      mnMinimumKeyFrames(5),
      mFrameFactory(frameFactory),
      mKeyFrameFactory(keyFrameFactory),
      mFeatureMatcher(featureMatcher) {
  mImgWidth = static_cast<int>(parameters.cx * 2);
  mImgHeight = static_cast<int>(parameters.cy * 2);
  mCurrentMatchImage = cv::Mat(cv::Size(mImgWidth * 2, mImgHeight), CV_8UC3);

  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = parameters.fx;
  K.at<float>(1, 1) = parameters.fy;
  K.at<float>(0, 2) = parameters.cx;
  K.at<float>(1, 2) = parameters.cy;
  K.copyTo(mK);

  // Max/Min Frames to insert keyframes and to check relocalization
  mMinFrames = parameters.minFrames;
  mMaxFrames = parameters.maxFrames;

  mMinLocalMatchCount = parameters.minLocalMatchCount;
  mMinIniMatchCount = parameters.minIniMatchCount;

  mnMinimumKeyFrames = parameters.minimumKeyFrames;

  cout << endl << "Camera Parameters: " << endl;
  cout << "- fx: " << parameters.fx << endl;
  cout << "- fy: " << parameters.fy << endl;
  cout << "- cx: " << parameters.cx << endl;
  cout << "- cy: " << parameters.cy << endl;

  mMinParallax = static_cast<float>(parameters.minimumParallax);
}

void Tracking::SetLocalMapper(LocalMapping* pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing* pLoopClosing) {
  mpLoopClosing = pLoopClosing;
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat& mImGray,
                                     const double& timestamp) {
  mCurrentFrame = mFrameFactory->Create(mImGray, timestamp, mK);
  Track();
  return mCurrentFrame->mTcw.clone();
}

void Tracking::Track() {
  if (mState == NO_IMAGES_YET) {
    mState = NOT_INITIALIZED;
  }

  mLastProcessedState = mState;

  if (mState == NOT_INITIALIZED) {
    if (mpMap->MapPointsInMap() == 0) {
      MonocularInitialization();

      mpMapDrawer->Update();
    }

    if (mState != OK) return;
  } else {
    // System is initialized. Track Frame.
    bool bOK = false;

    // Initial camera pose estimation using motion model or relocalization (if
    // tracking is lost)

    if (mState == OK) {
      // Local Mapping might have changed some MapPoints tracked in last frame
      CheckReplacedInLastFrame();

      if (mVelocity.empty() || mCurrentFrame->id() < mnLastRelocFrameId + 2) {
        bOK = TrackReferenceKeyFrame();
      } else {
        bOK = TrackWithMotionModel();
        if (!bOK) {
          bOK = TrackReferenceKeyFrame();
        }
      }
    } else {
      bOK = Relocalization();
    }

    mCurrentFrame->mpReferenceKF = mpReferenceKF;

    if (bOK) {
      // If we have an initial estimation of the camera pose and matching. Track
      // the local map.
      bOK = TrackLocalMap();
    }
    if (bOK) {
      mState = OK;
    } else {
      mState = LOST;
      std::cout << "Tracking lost ..." << std::endl;
    }

    // If tracking were good, check if we insert a keyframe
    if (bOK) {
      // Update motion model
      if (!mLastFrame->mTcw.empty()) {
        cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
        mLastFrame->GetRotationInverse().copyTo(
            LastTwc.rowRange(0, 3).colRange(0, 3));
        mLastFrame->GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
        mVelocity = mCurrentFrame->mTcw * LastTwc;
      } else {
        mVelocity = cv::Mat();
      }

      // Check if we need to insert a new keyframe
      if (NeedNewKeyFrame()) {
        CreateNewKeyFrame();
      }
    }

    // Reset if the camera get lost soon after initialization
    if (mState == LOST) {
      if (mpMap->KeyFramesInMap() <= mnMinimumKeyFrames) {
        cout << "Track lost soon after initialisation, reseting..." << endl;
        Reset();
        return;
      }
    }

    // Update drawer
    if (mState == OK) {
      mpMapDrawer->Update();
      auto pos = mCurrentFrame->GetCameraCenter();
      cv::Mat dir = cv::Mat::zeros(3, 1, CV_32F);
      dir.at<float>(2) = 1;
      dir = mCurrentFrame->GetRotationInverse() * dir;
      dir = dir / cv::norm(dir);
      mpMapDrawer->SetPosDir(pos.at<float>(0), pos.at<float>(1),
                             pos.at<float>(2), dir.at<float>(0),
                             dir.at<float>(1), dir.at<float>(2));
    }

    if (!mCurrentFrame->mpReferenceKF)
      mCurrentFrame->mpReferenceKF = mpReferenceKF;

    mLastFrame = FramePtr(mFrameFactory->Clone(*mCurrentFrame));
  }

  // Store frame pose information to retrieve the complete camera trajectory
  // afterwards.
  if (!mCurrentFrame->mTcw.empty()) {
    cv::Mat Tcr =
        mCurrentFrame->mTcw * mCurrentFrame->mpReferenceKF->GetPoseInverse();
    mlRelativeFramePoses.push_back(Tcr);
    mlpReferences.push_back(mpReferenceKF);
    mlFrameTimes.push_back(mCurrentFrame->mTimeStamp);
    mlbLost.push_back(mState == LOST);
  } else {
    // This can happen if tracking is lost
    mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
    mlpReferences.push_back(mlpReferences.back());
    mlFrameTimes.push_back(mlFrameTimes.back());
    mlbLost.push_back(mState == LOST);
  }
}

void Tracking::MonocularInitialization() {
  if (!mpInitializer) {
    // Set Reference Frame
    if (mCurrentFrame) {
      mInitialFrame = mFrameFactory->Clone(*mCurrentFrame);
      mLastFrame = mFrameFactory->Clone(*mCurrentFrame);
      mpInitializer = std::make_unique<Initializer>(mCurrentFrame->mK);
      return;
    }
  }
  if (mpInitializer) {
    // Find correspondences
    mIniMatchResult =
        mFeatureMatcher->MatchFrames(*mInitialFrame, *mCurrentFrame);

    CreateCurrentMatchImage(mIniMatchResult);

    if (!mInitializationAllowed) return;

    // Try to initialize
    // Check if there are enough correspondences
    if (mIniMatchResult.GetNumMatches() < mMinIniMatchCount) {
      std::cout << "Not enough matches to start initialization ..."
                << std::endl;
      mpInitializer.reset();
      return;
    }

    cv::Mat Rcw;                  // Current Camera Rotation
    cv::Mat tcw;                  // Current Camera Translation
    vector<bool> vbTriangulated;  // Triangulated Correspondences (mvIniMatches)

    if (mpInitializer->Initialize(mIniMatchResult, Rcw, tcw, mvIniP3D,
                                  vbTriangulated, mMinIniMatchCount,
                                  mMinParallax)) {
      mvIniMatches.resize(mIniMatchResult.GetNumMatches());
      for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
        if (vbTriangulated[i]) {
          mvIniMatches[i] = 1;
        } else {
          mvIniMatches[i] = -1;
        }
      }

      // Set Frame Poses
      mInitialFrame->SetPose(cv::Mat::eye(4, 4, CV_32F));
      cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
      Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
      tcw.copyTo(Tcw.rowRange(0, 3).col(3));
      mCurrentFrame->SetPose(Tcw);

      CreateInitialMapMonocular();
    } else {
      std::cout << "Initialization failed!" << std::endl;
    }
  }
}

void Tracking::CreateInitialMapMonocular() {
  mpMap->clear();
  // Create KeyFrames
  auto pKFini = mKeyFrameFactory->Create(*mInitialFrame, mpMap, mpKeyFrameDB);
  auto pKFcur = mKeyFrameFactory->Create(*mCurrentFrame, mpMap, mpKeyFrameDB);

  // Insert KFs in the map
  mpMap->AddKeyFrame(pKFini);
  mpMap->AddKeyFrame(pKFcur);

  // Create MapPoints and asscoiate to keyframes
  for (size_t i = 0; i < mvIniMatches.size(); i++) {
    if (mvIniMatches[i] < 0) continue;

    // Create MapPoint.
    cv::Mat worldPos(mvIniP3D[i]);

    auto pMP = std::make_shared<MapPoint>(worldPos, pKFcur, mpMap);

    pKFini->AddMapPoint(pMP, mIniMatchResult.keyPoints1[i]);
    pKFcur->AddMapPoint(pMP, mIniMatchResult.keyPoints2[i]);

    pMP->AddObservation(pKFini, mIniMatchResult.keyPoints1[i]);
    pMP->AddObservation(pKFcur, mIniMatchResult.keyPoints2[i]);

    pMP->UpdateNormalAndDepth();

    // Fill Current Frame structure
    mCurrentFrame->mKeyPointMap.SetMapPoint(mIniMatchResult.keyPoints2[i], pMP);

    // Add to Map
    mpMap->AddMapPoint(pMP);
  }

  // Update Connections
  pKFini->UpdateConnections();
  pKFcur->UpdateConnections();

  // Bundle Adjustment
  cout << "New Map created with " << mpMap->MapPointsInMap() << " points"
       << endl;

  Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

  // Set median depth to 1
  float medianDepth = pKFini->ComputeSceneMedianDepth(2);
  float invMedianDepth = 1.0f / medianDepth;
  float minDepth = 0.0;
  cout << "Scene depth " << medianDepth << endl;
  if (medianDepth < minDepth ||
      pKFcur->TrackedMapPoints(1) < mMinIniMatchCount) {
    cout << "Wrong initialization, reseting..." << endl;
    Reset();
    return;
  }

  // Scale initial baseline
  cv::Mat Tc2w = pKFcur->GetPose();
  Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
  pKFcur->SetPose(Tc2w);

  // Scale points
  auto& vpAllMapPoints = pKFini->mKeyPointMap;
  for (auto iMP = vpAllMapPoints.Begin(), end = vpAllMapPoints.End();
       iMP != end; ++iMP) {
    MapPointPtr pMP = iMP->second.mapPoint;
    pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
  }

  mpLocalMapper->InsertKeyFrame(pKFini);
  mpLocalMapper->InsertKeyFrame(pKFcur);

  mCurrentFrame->SetPose(pKFcur->GetPose());
  mnLastKeyFrameId = mCurrentFrame->id();
  mpLastKeyFrame = pKFcur;

  mvpLocalKeyFrames.push_back(pKFcur);
  mvpLocalKeyFrames.push_back(pKFini);
  mpReferenceKF = pKFcur;
  mCurrentFrame->mpReferenceKF = pKFcur;

  mLastFrame = mFrameFactory->Clone(*mCurrentFrame);

  mpMap->mvpKeyFrameOrigins.push_back(pKFini);

  mState = OK;
}

void Tracking::CheckReplacedInLastFrame() {
  for (auto i = mLastFrame->mKeyPointMap.Begin(),
            end = mLastFrame->mKeyPointMap.End();
       i != end; ++i) {
    MapPointPtr pMP = i->second.mapPoint;

    if (pMP) {
      MapPointPtr pRep = pMP->GetReplaced();
      if (pRep) {
        i->second.mapPoint = pRep;
      }
    }
  }
}

bool Tracking::TrackReferenceKeyFrame() {
  // We perform first feature matching with the reference keyframe
  auto matchResult =
      mFeatureMatcher->MatchFrames(*mCurrentFrame, *mpReferenceKF);

  CreateCurrentMatchImage(matchResult);

  auto nmatches = matchResult.GetNumMatches();

  if (nmatches < mMinLocalMatchCount) {
    return false;
  }

  for (size_t i = 0; i < matchResult.keyPoints2.size(); ++i) {
    auto pMP = matchResult.GetMapPoint2(i);
    if (pMP) {
      mCurrentFrame->mKeyPointMap.SetMapPoint(matchResult.keyPoints1[i], pMP);
    }
  }

  mCurrentFrame->SetPose(mLastFrame->mTcw);

  Optimizer::PoseOptimization(mCurrentFrame);

  // Discard outliers
  std::vector<int> pointsToRemove;
  int nmatchesMap = 0;
  for (auto i = mCurrentFrame->mKeyPointMap.Begin(),
            end = mCurrentFrame->mKeyPointMap.End();
       i != end; ++i) {
    if (i->second.outlier) {
      MapPointPtr pMP = i->second.mapPoint;
      pointsToRemove.push_back(i->first);
      pMP->mnLastFrameSeen = mCurrentFrame->id();
      nmatches--;
    } else if (i->second.mapPoint->Observations() > 0) {
      nmatchesMap++;
    }
  }
  for (auto index : pointsToRemove) {
    mCurrentFrame->mKeyPointMap.SetMapPoint(index, nullptr);
  }

  return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame() {
  // Update pose according to reference keyframe
  KeyFramePtr pRef = mLastFrame->mpReferenceKF;
  cv::Mat Tlr = mlRelativeFramePoses.back();

  mLastFrame->SetPose(Tlr * pRef->GetPose());
}

bool Tracking::TrackWithMotionModel() {
  // Update last frame pose according to its reference keyframe
  // Create "visual odometry" points if in Localization Mode
  UpdateLastFrame();

  mCurrentFrame->SetPose(mVelocity * mLastFrame->mTcw);

  mCurrentFrame->mKeyPointMap.Clear();

  // Match points seen in previous frame
  auto matchResult = mFeatureMatcher->MatchFrames(*mCurrentFrame, *mLastFrame);

  CreateCurrentMatchImage(matchResult);

  auto nmatches = matchResult.GetNumMatches();

  if (nmatches < mMinLocalMatchCount) {
    return false;
  }

  for (size_t i = 0; i < matchResult.keyPoints2.size(); ++i) {
    auto pMP = matchResult.GetMapPoint2(i);
    if (pMP) {
      mCurrentFrame->mKeyPointMap.SetMapPoint(matchResult.keyPoints1[i],
                                              matchResult.GetMapPoint2(i));
    }
  }

  // Optimize frame pose with all matches
  Optimizer::PoseOptimization(mCurrentFrame);

  // Discard outliers
  std::vector<int> pointsToRemove;
  int nmatchesMap = 0;
  for (auto i = mCurrentFrame->mKeyPointMap.Begin(),
            end = mCurrentFrame->mKeyPointMap.End();
       i != end; ++i) {
    if (i->second.outlier) {
      MapPointPtr pMP = i->second.mapPoint;
      pointsToRemove.push_back(i->first);
      pMP->mnLastFrameSeen = mCurrentFrame->id();
      nmatches--;
    } else if (i->second.mapPoint->Observations() > 0) {
      nmatchesMap++;
    }
  }
  for (auto index : pointsToRemove) {
    mCurrentFrame->mKeyPointMap.SetMapPoint(index, nullptr);
  }

  return nmatchesMap >= 10;
}

bool Tracking::TrackLocalMap() {
  // We have an estimation of the camera pose and some map points tracked in the
  // frame. We retrieve the local map and try to find matches to points in the
  // local map.

  UpdateLocalKeyFrames();
  SearchLocalPoints();

  // Optimize Pose
  Optimizer::PoseOptimization(mCurrentFrame);
  mnMatchesInliers = 0;

  // Update MapPoints Statistics
  for (auto i = mCurrentFrame->mKeyPointMap.Begin(),
            end = mCurrentFrame->mKeyPointMap.End();
       i != end; ++i) {
    if (!i->second.outlier) {
      i->second.mapPoint->IncreaseFound();
      if (i->second.mapPoint->Observations() > 0) mnMatchesInliers++;
    }
  }

  auto trackCoeff = (mnMatchesInliers + 0.0) / mMinLocalMatchCount;
  std::cout << "Tracking coefficient - " << trackCoeff
            << ", if < 1.0 then tracking will be lost." << std::endl;

  // Decide if the tracking was succesful
  if (mnMatchesInliers < mMinLocalMatchCount)
    return false;
  else
    return true;
}

bool Tracking::NeedNewKeyFrame() {
  const int nKFs = static_cast<int>(mpMap->KeyFramesInMap());

  // Do not insert keyframes if not enough frames have passed from last
  // relocalisation
  if (mCurrentFrame->id() < mnLastRelocFrameId + mMaxFrames &&
      nKFs > mMaxFrames)
    return false;

  // Tracked MapPoints in the reference keyframe
  int nMinObs = 3;
  if (nKFs <= 2) nMinObs = 2;
  int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

  // Thresholds
  float thRefRatio = 0.9f;

  // Condition 1a: More than "MaxFrames" have passed from last keyframe
  // insertion
  const bool c1a = mCurrentFrame->id() >= mnLastKeyFrameId + mMaxFrames;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  const bool c1b = (mCurrentFrame->id() >= mnLastKeyFrameId + mMinFrames);

  // Few tracked points compared to reference keyframe. Lots of
  // visual odometry compared to map matches.
  const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio) &&
                   mnMatchesInliers > mMinLocalMatchCount);

  auto newKeyFrameCoeff = (nRefMatches * thRefRatio + 0.0) / mnMatchesInliers;
  std::cout << "New KeyFrame coeff - " << newKeyFrameCoeff
            << ", shoule be > 1 to create new KF" << std::endl;

  if ((c1a || c1b) && c2) {
    return true;
  } else
    return false;
}

void Tracking::CreateNewKeyFrame() {
  std::cout << "New KF created" << std::endl;

  KeyFramePtr pKF =
      mKeyFrameFactory->Create(*mCurrentFrame, mpMap, mpKeyFrameDB);

  mpReferenceKF = pKF;
  mCurrentFrame->mpReferenceKF = pKF;

  mpLocalMapper->InsertKeyFrame(pKF);

  mnLastKeyFrameId = mCurrentFrame->id();
  mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
  // Do not search map points already matched
  std::vector<int> pointsToRemove;
  for (auto vit = mCurrentFrame->mKeyPointMap.Begin(),
            vend = mCurrentFrame->mKeyPointMap.End();
       vit != vend; ++vit) {
    MapPointPtr pMP = vit->second.mapPoint;
    if (pMP) {
      if (pMP->isBad()) {
        pointsToRemove.push_back(vit->first);
      } else {
        pMP->IncreaseVisible();
        pMP->mnLastFrameSeen = mCurrentFrame->id();
      }
    }
  }
  for (auto index : pointsToRemove) {
    // erase
    mCurrentFrame->mKeyPointMap.SetMapPoint(index, nullptr);
  }

  // Project local frame points in frame and check its visibility
  for (vector<KeyFramePtr>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                           itEndKF = mvpLocalKeyFrames.end();
       itKF != itEndKF; itKF++) {
    KeyFramePtr pKF = *itKF;
    auto vpMPs = pKF->GetMapPointMatches();

    // Project points in frame and check its visibility
    int nToMatch = 0;
    for (auto itMP = vpMPs.Begin(), itEndMP = vpMPs.End(); itMP != itEndMP;
         ++itMP) {
      MapPointPtr pMP = itMP->second.mapPoint;
      if (!pMP) continue;
      if (pMP->mnTrackReferenceForFrame == mCurrentFrame->id()) continue;
      if (!pMP->isBad()) {
        pMP->mnTrackReferenceForFrame = mCurrentFrame->id();
        if (pMP->mnLastFrameSeen != mCurrentFrame->id()) {
          // Project
          if (mCurrentFrame->isInFrustum(*pMP, 0.5)) {
            pMP->IncreaseVisible();
            nToMatch++;
          }
        }
      }
    }

    if (nToMatch > 0) {
      auto matchResult = mFeatureMatcher->MatchFrames(*mCurrentFrame, *pKF);
      for (size_t i = 0; i < matchResult.keyPoints1.size(); ++i) {
        auto pMP1 = matchResult.GetMapPoint1(i);
        auto pMP2 = matchResult.GetMapPoint2(i);
        // add new found points
        if (!pMP1 && pMP2) {
          mCurrentFrame->mKeyPointMap.SetMapPoint(matchResult.keyPoints1[i],
                                                  pMP2);
        }
      }
    }
  }
}

void Tracking::UpdateLocalKeyFrames() {
  // Each map point vote for the keyframes in which it has been observed
  std::vector<int> pointsToRemove;
  map<KeyFramePtr, int> keyframeCounter;
  for (auto i = mCurrentFrame->mKeyPointMap.Begin(),
            end = mCurrentFrame->mKeyPointMap.End();
       i != end; ++i) {
    MapPointPtr pMP = i->second.mapPoint;
    if (!pMP->isBad()) {
      const auto observations = pMP->GetObservations();
      for (auto it = observations.begin(), itend = observations.end();
           it != itend; it++)
        keyframeCounter[it->first]++;
    } else {
      pointsToRemove.push_back(i->first);
    }
  }
  for (auto index : pointsToRemove) {
    // erase
    mCurrentFrame->mKeyPointMap.SetMapPoint(index, nullptr);
  }

  if (keyframeCounter.empty()) return;

  int max = 0;
  KeyFramePtr pKFmax = static_cast<KeyFramePtr>(NULL);

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also
  // check which keyframe shares most points
  for (map<KeyFramePtr, int>::const_iterator it = keyframeCounter.begin(),
                                             itEnd = keyframeCounter.end();
       it != itEnd; it++) {
    KeyFramePtr pKF = it->first;

    if (pKF->isBad()) continue;

    if (it->second > max) {
      max = it->second;
      pKFmax = pKF;
    }

    mvpLocalKeyFrames.push_back(it->first);
    pKF->mnTrackReferenceForFrame = mCurrentFrame->id();
  }

  // Include also some not-already-included keyframes that are neighbors to
  // already-included keyframes
  for (vector<KeyFramePtr>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                           itEndKF = mvpLocalKeyFrames.end();
       itKF != itEndKF; itKF++) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size() > 80) break;

    KeyFramePtr pKF = *itKF;

    const vector<KeyFramePtr> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

    for (vector<KeyFramePtr>::const_iterator itNeighKF = vNeighs.begin(),
                                             itEndNeighKF = vNeighs.end();
         itNeighKF != itEndNeighKF; itNeighKF++) {
      KeyFramePtr pNeighKF = *itNeighKF;
      if (!pNeighKF->isBad()) {
        if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame->id()) {
          mvpLocalKeyFrames.push_back(pNeighKF);
          pNeighKF->mnTrackReferenceForFrame = mCurrentFrame->id();
          break;
        }
      }
    }

    const set<KeyFramePtr> spChilds = pKF->GetChilds();
    for (set<KeyFramePtr>::const_iterator sit = spChilds.begin(),
                                          send = spChilds.end();
         sit != send; sit++) {
      KeyFramePtr pChildKF = *sit;
      if (!pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame->id()) {
          mvpLocalKeyFrames.push_back(pChildKF);
          pChildKF->mnTrackReferenceForFrame = mCurrentFrame->id();
          break;
        }
      }
    }

    KeyFramePtr pParent = pKF->GetParent();
    if (pParent) {
      if (pParent->mnTrackReferenceForFrame != mCurrentFrame->id()) {
        mvpLocalKeyFrames.push_back(pParent);
        pParent->mnTrackReferenceForFrame = mCurrentFrame->id();
        break;
      }
    }
  }

  if (pKFmax) {
    mpReferenceKF = pKFmax;
    mCurrentFrame->mpReferenceKF = mpReferenceKF;
  }
}

bool Tracking::Relocalization() {
  // Relocalization is performed when tracking is lost
  // Track Lost: Query KeyFrame Database for keyframe candidates for
  // relocalisation
  vector<KeyFramePtr> vpCandidateKFs =
      mpKeyFrameDB->DetectRelocalizationCandidates(*mCurrentFrame);

  if (vpCandidateKFs.empty()) return false;

  const auto nKFs = vpCandidateKFs.size();

  // We perform first an matching with each candidate
  // If enough matches are found we setup a PnP solver

  vector<std::unique_ptr<PnPsolver>> vpPnPsolvers;
  vpPnPsolvers.resize(nKFs);

  vector<MatchFramesResult> vvpMapPointMatches;
  vvpMapPointMatches.resize(nKFs);

  vector<bool> vbDiscarded;
  vbDiscarded.resize(nKFs);

  int nCandidates = 0;

  for (int i = 0; i < nKFs; i++) {
    KeyFramePtr pKF = vpCandidateKFs[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else {
      vvpMapPointMatches[i] =
          mFeatureMatcher->MatchFrames(*mCurrentFrame, *pKF);
      auto nmatches = vvpMapPointMatches[i].GetNumMatches();
      if (nmatches < mMinLocalMatchCount) {
        vbDiscarded[i] = true;
        continue;
      } else {
        auto pSolver = std::make_unique<PnPsolver>(vvpMapPointMatches[i]);
        pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991f);
        vpPnPsolvers[i] = std::move(pSolver);
        nCandidates++;
      }
    }
  }

  // Alternatively perform some iterations of P4P RANSAC
  // Until we found a camera pose supported by enough inliers
  bool bMatch = false;

  while (nCandidates > 0 && !bMatch) {
    for (int i = 0; i < nKFs; i++) {
      if (vbDiscarded[i]) continue;

      // Perform 5 Ransac Iterations
      vector<bool> vbInliers;
      int nInliers;
      bool bNoMore;

      CreateCurrentMatchImage(vvpMapPointMatches[i]);

      PnPsolver* pSolver = vpPnPsolvers[i].get();
      cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

      // If Ransac reachs max. iterations discard keyframe
      if (bNoMore) {
        vbDiscarded[i] = true;
        nCandidates--;
      }

      // If a Camera Pose is computed, optimize
      if (!Tcw.empty()) {
        Tcw.copyTo(mCurrentFrame->mTcw);

        set<MapPointPtr> sFound;

        const auto np = vbInliers.size();

        for (size_t j = 0; j < np; j++) {
          if (vbInliers[j]) {
            mCurrentFrame->mKeyPointMap.SetMapPoint(
                vvpMapPointMatches[i].keyPoints1[j],
                vvpMapPointMatches[i].GetMapPoint2(j));
            sFound.insert(vvpMapPointMatches[i].GetMapPoint2(j));
          } else {
            mCurrentFrame->mKeyPointMap.SetMapPoint(
                vvpMapPointMatches[i].keyPoints1[j], nullptr);
          }
        }

        int nGood = Optimizer::PoseOptimization(mCurrentFrame);

        if (nGood < 10) {
          continue;
        }

        std::vector<int> pointsToRemove;
        for (auto io = mCurrentFrame->mKeyPointMap.Begin(),
                  endo = mCurrentFrame->mKeyPointMap.End();
             io != endo; ++io) {
          if (io->second.outlier) {
            pointsToRemove.push_back(io->first);
          }
        }
        for (auto index : pointsToRemove) {
          mCurrentFrame->mKeyPointMap.SetMapPoint(index, nullptr);
        }

        // If the pose is supported by enough inliers stop ransacs and continue
        if (nGood >= mMinLocalMatchCount) {
          bMatch = true;
          break;
        }
      }
    }
  }

  if (!bMatch) {
    // this prevents a segfault later
    // (https://github.com/raulmur/ORB_SLAM2/pull/381#issuecomment-337312336)
    mCurrentFrame->mTcw = cv::Mat::zeros(0, 0, CV_32F);
    return false;
  } else {
    std::cout << "Relocalization successful" << std::endl;
    mnLastRelocFrameId = mCurrentFrame->id();
    return true;
  }
}

void Tracking::Reset() {
  cout << "System Reseting" << endl;

  // Reset Local Mapping
  cout << "Reseting Local Mapper...";
  mpLocalMapper->Reset();
  cout << " done" << endl;

  // Reset Loop Closing
  cout << "Reseting Loop Closing...";
  mpLoopClosing->Reset();
  cout << " done" << endl;

  // Clear Database
  cout << "Reseting Database...";
  mpKeyFrameDB->clear();
  cout << " done" << endl;

  // Clear Map (this erase MapPoints and KeyFrames)
  mpMap->clear();

  mState = NO_IMAGES_YET;

  mpInitializer.reset();

  mlRelativeFramePoses.clear();
  mlpReferences.clear();
  mlFrameTimes.clear();
  mlbLost.clear();
}

cv::Mat Tracking::GetCurrentMatchImage() const { return mCurrentMatchImage; }

void Tracking::CreateCurrentMatchImage(const MatchFramesResult& matchResult) {
  cv::Mat out_img0 = mCurrentMatchImage(cv::Rect(0, 0, mImgWidth, mImgHeight));
  cv::Mat out_img1 =
      mCurrentMatchImage(cv::Rect(mImgWidth, 0, mImgWidth, mImgHeight));
  cv::cvtColor(matchResult.pF1->imGray, out_img0, cv::COLOR_GRAY2RGB);
  cv::cvtColor(matchResult.pF2->imGray, out_img1, cv::COLOR_GRAY2RGB);

  cv::Scalar colorNewMatch(0, 255, 0, 255);
  cv::Scalar colorWithMapPoint(255, 0, 0, 255);
  int radius = 3;
  auto numMatches = matchResult.GetNumMatches();
  for (int i = 0; i < numMatches; ++i) {
    auto pMP1 = matchResult.GetMapPoint1(i);
    auto pMP2 = matchResult.GetMapPoint2(i);
    if (!pMP1 && !pMP2) {
      cv::circle(
          out_img0,
          cv::Point(matchResult.keyPoints1[i].x, matchResult.keyPoints1[i].y),
          radius, colorNewMatch, cv::FILLED);

      cv::circle(
          out_img1,
          cv::Point(matchResult.keyPoints2[i].x, matchResult.keyPoints2[i].y),
          radius, colorNewMatch, cv::FILLED);
    }
  }
  for (int i = 0; i < numMatches; ++i) {
    auto pMP1 = matchResult.GetMapPoint1(i);
    auto pMP2 = matchResult.GetMapPoint2(i);
    if (pMP1 || pMP2) {
      cv::circle(
          out_img0,
          cv::Point(matchResult.keyPoints1[i].x, matchResult.keyPoints1[i].y),
          radius, colorWithMapPoint, cv::FILLED);

      cv::circle(
          out_img1,
          cv::Point(matchResult.keyPoints2[i].x, matchResult.keyPoints2[i].y),
          radius, colorWithMapPoint, cv::FILLED);
    }
  }
}

void Tracking::ToggleInitializationAllowed() { mInitializationAllowed = true; }

}  // namespace SLAM_PIPELINE
