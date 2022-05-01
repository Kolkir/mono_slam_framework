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
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Converter.h"
#include "FeatureMatcher.h"
#include "Initializer.h"
#include "LoopClosing.h"
#include "Map.h"
#include "Optimizer.h"
#include "PnPsolver.h"

using namespace std;

namespace SLAM_PIPELINE {

Tracking::Tracking(System* pSys, MapDrawer* pMapDrawer, Map* pMap,
                   KeyFrameDatabase* pKFDB, const FeatureParameters& parameters,
                   FeatureMatcher* featureMatcher, FrameFactory* frameFactory,
                   KeyFrameFactory* keyFrameFactory)
    : mState(NO_IMAGES_YET),
      mpKeyFrameDB(pKFDB),
      mpInitializer(static_cast<Initializer*>(NULL)),
      mpSystem(pSys),
      mpMapDrawer(pMapDrawer),
      mpMap(pMap),
      mnLastRelocFrameId(0),
      mnMinimumKeyFrames(5),
      mFrameFactory(frameFactory),
      mKeyFrameFactory(keyFrameFactory),
      mFeatureMatcher(featureMatcher) {
  mImgWidth = static_cast<int>(parameters.cx * 2);
  mImgHeight = static_cast<int>(parameters.cy * 2);
  mIniMatchImage = cv::Mat(cv::Size(mImgWidth * 2, mImgHeight), CV_8UC3);

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
}

void Tracking::SetLocalMapper(LocalMapping* pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing* pLoopClosing) {
  mpLoopClosing = pLoopClosing;
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat& mImGray,
                                     const double& timestamp) {
  mCurrentFrame.reset(mFrameFactory->Create(mImGray, timestamp, mK));
  Track();
  return mCurrentFrame->mTcw.clone();
}

void Tracking::Track() {
  if (mState == NO_IMAGES_YET) {
    mState = NOT_INITIALIZED;
  }

  mLastProcessedState = mState;

  // Get Map Mutex -> Map cannot be changed
  unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

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

    // Local Mapping is activated.

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
    if (bOK)
      mState = OK;
    else
      mState = LOST;

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
        mpSystem->Reset();
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

    mLastFrame.reset(mFrameFactory->Clone(mCurrentFrame.get()));
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
      mInitialFrame.reset(mFrameFactory->Clone(mCurrentFrame.get()));
      mLastFrame.reset(mFrameFactory->Clone(mCurrentFrame.get()));
      mpInitializer = std::make_unique<Initializer>(mCurrentFrame->mK);
      return;
    }
  }
  if (mpInitializer) {
    // Find correspondences
    mIniMatchResult =
        mFeatureMatcher->MatchFrames(mInitialFrame.get(), mCurrentFrame.get());

    CreateIniMatchImage();

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
                                  vbTriangulated, mMinIniMatchCount)) {
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
  KeyFrame* pKFini =
      mKeyFrameFactory->Create(*mInitialFrame, mpMap, mpKeyFrameDB);
  KeyFrame* pKFcur =
      mKeyFrameFactory->Create(*mCurrentFrame, mpMap, mpKeyFrameDB);

  // Insert KFs in the map
  mpMap->AddKeyFrame(pKFini);
  mpMap->AddKeyFrame(pKFcur);

  // Create MapPoints and asscoiate to keyframes
  for (size_t i = 0; i < mvIniMatches.size(); i++) {
    if (mvIniMatches[i] < 0) continue;

    // Create MapPoint.
    cv::Mat worldPos(mvIniP3D[i]);

    MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);

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
    MapPoint* pMP = iMP->second.mapPoint;
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

  mLastFrame.reset(mFrameFactory->Clone(mCurrentFrame.get()));

  mpMap->mvpKeyFrameOrigins.push_back(pKFini);

  mState = OK;
}

void Tracking::CheckReplacedInLastFrame() {
  for (auto i = mLastFrame->mKeyPointMap.Begin(),
            end = mLastFrame->mKeyPointMap.End();
       i != end; ++i) {
    MapPoint* pMP = i->second.mapPoint;

    if (pMP) {
      MapPoint* pRep = pMP->GetReplaced();
      if (pRep) {
        i->second.mapPoint = pRep;
      }
    }
  }
}

bool Tracking::TrackReferenceKeyFrame() {
  // We perform first feature matching with the reference keyframe
  auto matchResult =
      mFeatureMatcher->MatchFrames(mCurrentFrame.get(), mpReferenceKF);

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

  Optimizer::PoseOptimization(mCurrentFrame.get());

  // Discard outliers
  std::vector<int> pointsToRemove;
  int nmatchesMap = 0;
  for (auto i = mCurrentFrame->mKeyPointMap.Begin(),
            end = mCurrentFrame->mKeyPointMap.End();
       i != end; ++i) {
    if (i->second.outlier) {
      MapPoint* pMP = i->second.mapPoint;
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
  KeyFrame* pRef = mLastFrame->mpReferenceKF;
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
  auto matchResult =
      mFeatureMatcher->MatchFrames(mCurrentFrame.get(), mLastFrame.get());

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
  Optimizer::PoseOptimization(mCurrentFrame.get());

  // Discard outliers
  std::vector<int> pointsToRemove;
  int nmatchesMap = 0;
  for (auto i = mCurrentFrame->mKeyPointMap.Begin(),
            end = mCurrentFrame->mKeyPointMap.End();
       i != end; ++i) {
    if (i->second.outlier) {
      MapPoint* pMP = i->second.mapPoint;
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
  Optimizer::PoseOptimization(mCurrentFrame.get());
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

  // Decide if the tracking was succesful
  if (mnMatchesInliers < mMinLocalMatchCount)
    return false;
  else
    return true;
}

bool Tracking::NeedNewKeyFrame() {
  // If Local Mapping is freezed by a Loop Closure do not insert keyframes
  if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    return false;

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

  // Local Mapping accept keyframes?
  bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

  // Thresholds
  float thRefRatio = 0.9f;

  // Condition 1a: More than "MaxFrames" have passed from last keyframe
  // insertion
  const bool c1a = mCurrentFrame->id() >= mnLastKeyFrameId + mMaxFrames;
  // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
  const bool c1b = (mCurrentFrame->id() >= mnLastKeyFrameId + mMinFrames &&
                    bLocalMappingIdle);

  // Few tracked points compared to reference keyframe. Lots of
  // visual odometry compared to map matches.
  const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio) &&
                   mnMatchesInliers > mMinLocalMatchCount);

  if ((c1a || c1b) && c2) {
    // If the mapping accepts keyframes, insert keyframe.
    // Otherwise send a signal to interrupt BA
    if (bLocalMappingIdle) {
      return true;
    } else {
      mpLocalMapper->InterruptBA();
      return false;
    }
  } else
    return false;
}

void Tracking::CreateNewKeyFrame() {
  if (!mpLocalMapper->SetNotStop(true)) return;

  std::cout << "New KF created" << std::endl;

  KeyFrame* pKF = mKeyFrameFactory->Create(*mCurrentFrame, mpMap, mpKeyFrameDB);

  mpReferenceKF = pKF;
  mCurrentFrame->mpReferenceKF = pKF;

  mpLocalMapper->InsertKeyFrame(pKF);

  mpLocalMapper->SetNotStop(false);

  mnLastKeyFrameId = mCurrentFrame->id();
  mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
  // Do not search map points already matched
  std::vector<int> pointsToRemove;
  for (auto vit = mCurrentFrame->mKeyPointMap.Begin(),
            vend = mCurrentFrame->mKeyPointMap.End();
       vit != vend; ++vit) {
    MapPoint* pMP = vit->second.mapPoint;
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
  for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                         itEndKF = mvpLocalKeyFrames.end();
       itKF != itEndKF; itKF++) {
    KeyFrame* pKF = *itKF;
    auto vpMPs = pKF->GetMapPointMatches();

    // Project points in frame and check its visibility
    int nToMatch = 0;
    for (auto itMP = vpMPs.Begin(), itEndMP = vpMPs.End(); itMP != itEndMP;
         ++itMP) {
      MapPoint* pMP = itMP->second.mapPoint;
      if (!pMP) continue;
      if (pMP->mnTrackReferenceForFrame == mCurrentFrame->id()) continue;
      if (!pMP->isBad()) {
        pMP->mnTrackReferenceForFrame = mCurrentFrame->id();
        if (pMP->mnLastFrameSeen != mCurrentFrame->id()) {
          // Project (this fills MapPoint variables for matching)
          if (mCurrentFrame->isInFrustum(pMP, 0.5)) {
            pMP->IncreaseVisible();
            nToMatch++;
          }
        }
      }
    }

    if (nToMatch > 0) {
      auto matchResult = mFeatureMatcher->MatchFrames(mCurrentFrame.get(), pKF);
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
  map<KeyFrame*, int> keyframeCounter;
  for (auto i = mCurrentFrame->mKeyPointMap.Begin(),
            end = mCurrentFrame->mKeyPointMap.End();
       i != end; ++i) {
    MapPoint* pMP = i->second.mapPoint;
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
  KeyFrame* pKFmax = static_cast<KeyFrame*>(NULL);

  mvpLocalKeyFrames.clear();
  mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

  // All keyframes that observe a map point are included in the local map. Also
  // check which keyframe shares most points
  for (map<KeyFrame*, int>::const_iterator it = keyframeCounter.begin(),
                                           itEnd = keyframeCounter.end();
       it != itEnd; it++) {
    KeyFrame* pKF = it->first;

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
  for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(),
                                         itEndKF = mvpLocalKeyFrames.end();
       itKF != itEndKF; itKF++) {
    // Limit the number of keyframes
    if (mvpLocalKeyFrames.size() > 80) break;

    KeyFrame* pKF = *itKF;

    const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

    for (vector<KeyFrame*>::const_iterator itNeighKF = vNeighs.begin(),
                                           itEndNeighKF = vNeighs.end();
         itNeighKF != itEndNeighKF; itNeighKF++) {
      KeyFrame* pNeighKF = *itNeighKF;
      if (!pNeighKF->isBad()) {
        if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame->id()) {
          mvpLocalKeyFrames.push_back(pNeighKF);
          pNeighKF->mnTrackReferenceForFrame = mCurrentFrame->id();
          break;
        }
      }
    }

    const set<KeyFrame*> spChilds = pKF->GetChilds();
    for (set<KeyFrame*>::const_iterator sit = spChilds.begin(),
                                        send = spChilds.end();
         sit != send; sit++) {
      KeyFrame* pChildKF = *sit;
      if (!pChildKF->isBad()) {
        if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame->id()) {
          mvpLocalKeyFrames.push_back(pChildKF);
          pChildKF->mnTrackReferenceForFrame = mCurrentFrame->id();
          break;
        }
      }
    }

    KeyFrame* pParent = pKF->GetParent();
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
  vector<KeyFrame*> vpCandidateKFs =
      mpKeyFrameDB->DetectRelocalizationCandidates(mCurrentFrame.get());

  if (vpCandidateKFs.empty()) return false;

  const auto nKFs = vpCandidateKFs.size();

  // We perform first an matching with each candidate
  // If enough matches are found we setup a PnP solver

  vector<PnPsolver*> vpPnPsolvers;
  vpPnPsolvers.resize(nKFs);

  vector<MatchFramesResult> vvpMapPointMatches;
  vvpMapPointMatches.resize(nKFs);

  vector<bool> vbDiscarded;
  vbDiscarded.resize(nKFs);

  int nCandidates = 0;

  for (int i = 0; i < nKFs; i++) {
    KeyFrame* pKF = vpCandidateKFs[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else {
      vvpMapPointMatches[i] =
          mFeatureMatcher->MatchFrames(mCurrentFrame.get(), pKF);
      auto nmatches = vvpMapPointMatches[i].GetNumMatches();
      if (nmatches < mMinLocalMatchCount) {
        vbDiscarded[i] = true;
        continue;
      } else {
        PnPsolver* pSolver = new PnPsolver(vvpMapPointMatches[i]);
        pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991f);
        vpPnPsolvers[i] = pSolver;
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

      PnPsolver* pSolver = vpPnPsolvers[i];
      cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

      // If Ransac reachs max. iterations discard keyframe
      if (bNoMore) {
        vbDiscarded[i] = true;
        nCandidates--;
      }

      // If a Camera Pose is computed, optimize
      if (!Tcw.empty()) {
        Tcw.copyTo(mCurrentFrame->mTcw);

        set<MapPoint*> sFound;

        const auto np = vbInliers.size();

        for (size_t j = 0; j < np; j++) {
          if (vbInliers[j]) {
            mCurrentFrame->mKeyPointMap.SetMapPoint(
                vvpMapPointMatches[i].keyPoints1[j],
                vvpMapPointMatches[i].GetMapPoint1(j));
            sFound.insert(vvpMapPointMatches[i].GetMapPoint1(j));
          } else {
            mCurrentFrame->mKeyPointMap.SetMapPoint(
                vvpMapPointMatches[i].keyPoints1[j], nullptr);
          }
        }

        int nGood = Optimizer::PoseOptimization(mCurrentFrame.get());

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
    mnLastRelocFrameId = mCurrentFrame->id();
    return true;
  }
}

void Tracking::Reset() {
  cout << "System Reseting" << endl;

  // Reset Local Mapping
  cout << "Reseting Local Mapper...";
  mpLocalMapper->RequestReset();
  cout << " done" << endl;

  // Reset Loop Closing
  cout << "Reseting Loop Closing...";
  // mpLoopClosing->RequestReset();
  cout << " done" << endl;

  // Clear BoW Database
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

cv::Mat Tracking::GetIniMatchImage() const { return mIniMatchImage; }

void Tracking::CreateIniMatchImage() {
  cv::Mat out_img0 = mIniMatchImage(cv::Rect(0, 0, mImgWidth, mImgHeight));
  cv::Mat out_img1 =
      mIniMatchImage(cv::Rect(mImgWidth, 0, mImgWidth, mImgHeight));
  cv::cvtColor(mInitialFrame->imGray, out_img0, cv::COLOR_GRAY2RGB);
  cv::cvtColor(mCurrentFrame->imGray, out_img1, cv::COLOR_GRAY2RGB);

  cv::Scalar color(0, 255, 0, 255);
  int radius = 3;
  auto numMatches = mIniMatchResult.GetNumMatches();
  for (int i = 0; i < numMatches; ++i) {
    cv::circle(out_img0,
               cv::Point(mIniMatchResult.keyPoints1[i].x,
                         mIniMatchResult.keyPoints1[i].y),
               radius, color, cv::FILLED);
    cv::circle(out_img1,
               cv::Point(mIniMatchResult.keyPoints2[i].x,
                         mIniMatchResult.keyPoints2[i].y),
               radius, color, cv::FILLED);
  }
}

void Tracking::ToggleInitializationAllowed() { mInitializationAllowed = true; }

}  // namespace SLAM_PIPELINE
