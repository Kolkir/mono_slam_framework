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

#include "LocalMapping.h"

#include <mutex>

#include "FeatureMatcher.h"
#include "KeyPointMap.h"
#include "LoopClosing.h"
#include "Optimizer.h"

namespace SLAM_PIPELINE {

LocalMapping::LocalMapping(Map *pMap, FeatureMatcher *featureMatcher)
    : mbResetRequested(false),
      mbFinishRequested(false),
      mbFinished(true),
      mpMap(pMap),
      mbAbortBA(false),
      mbStopped(false),
      mbStopRequested(false),
      mbNotStop(false),
      mbAcceptKeyFrames(true),
      mFeatureMatcher(featureMatcher) {}

void LocalMapping::SetLoopCloser(LoopClosing *pLoopCloser) {
  mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker) { mpTracker = pTracker; }

void LocalMapping::Run() {
  mbFinished = false;

  while (1) {
    // Tracking will see that Local Mapping is busy
    SetAcceptKeyFrames(false);

    // Check if there are keyframes in the queue
    if (CheckNewKeyFrames()) {
      ProcessNewKeyFrame();

      // Check recent MapPoints
      MapPointCulling();

      // Triangulate new MapPoints
      CreateNewMapPoints();

      if (!CheckNewKeyFrames()) {
        // Find more matches in neighbor keyframes and fuse point duplications
        SearchInNeighbors();
      }

      mbAbortBA = false;

      if (!CheckNewKeyFrames() && !stopRequested()) {
        // Local BA
        if (mpMap->KeyFramesInMap() > 2)
          Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA,
                                           mpMap);

        // Check redundant local Keyframes
        KeyFrameCulling();
      }

      // mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
    } else if (Stop()) {
      // Safe area to stop
      while (isStopped() && !CheckFinish()) {
        std::this_thread::sleep_for(std::chrono::microseconds(3000));
      }
      if (CheckFinish()) break;
    }

    ResetIfRequested();

    // Tracking will see that Local Mapping is busy
    SetAcceptKeyFrames(true);

    if (CheckFinish()) break;

    std::this_thread::sleep_for(std::chrono::microseconds(3000));
  }

  SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF) {
  std::unique_lock<std::mutex> lock(mMutexNewKFs);
  mlNewKeyFrames.push_back(pKF);
  mbAbortBA = true;
}

bool LocalMapping::CheckNewKeyFrames() {
  std::unique_lock<std::mutex> lock(mMutexNewKFs);
  return (!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame() {
  {
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    mpCurrentKeyFrame = mlNewKeyFrames.front();
    mlNewKeyFrames.pop_front();
  }

  // Associate MapPoints to the new keyframe and update normal
  auto vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
  auto start = vpMapPointMatches.Begin();
  auto end = vpMapPointMatches.End();
  for (auto i = start; i != end; ++i) {
    MapPoint *pMP = i->second.mapPoint;
    if (pMP) {
      if (!pMP->isBad()) {
        if (!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
          pMP->AddObservation(mpCurrentKeyFrame,
                              vpMapPointMatches.KeyPointFromIndex(i->first));
          pMP->UpdateNormalAndDepth();
        } else  // this can only happen for new stereo points inserted by the
                // Tracking
        {
          // TODO: investigate
          // assert(false);  // due to only monocular mode
          mlpRecentAddedMapPoints.push_back(pMP);
        }
      }
    }
  }

  // Update links in the Covisibility Graph
  mpCurrentKeyFrame->UpdateConnections();

  // Insert Keyframe in Map
  mpMap->AddKeyFrame(mpCurrentKeyFrame);
}

void LocalMapping::MapPointCulling() {
  // Check Recent Added MapPoints
  std::list<MapPoint *>::iterator lit = mlpRecentAddedMapPoints.begin();
  const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

  int nThObs = 2;
  const int cnThObs = nThObs;

  while (lit != mlpRecentAddedMapPoints.end()) {
    MapPoint *pMP = *lit;
    if (pMP->isBad()) {
      lit = mlpRecentAddedMapPoints.erase(lit);
    } else if (pMP->GetFoundRatio() < 0.25f) {
      pMP->SetBadFlag();
      lit = mlpRecentAddedMapPoints.erase(lit);
    } else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 2 &&
               pMP->Observations() <= cnThObs) {
      pMP->SetBadFlag();
      lit = mlpRecentAddedMapPoints.erase(lit);
    } else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 3)
      lit = mlpRecentAddedMapPoints.erase(lit);
    else
      lit++;
  }
}

void LocalMapping::CreateNewMapPoints() {
  // Retrieve neighbor keyframes in covisibility graph
  int nn = 20;
  const std::vector<KeyFrame *> vpNeighKFs =
      mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

  cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
  cv::Mat Rwc1 = Rcw1.t();
  cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
  cv::Mat Tcw1(3, 4, CV_32F);
  Rcw1.copyTo(Tcw1.colRange(0, 3));
  tcw1.copyTo(Tcw1.col(3));
  cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

  const float &fx1 = mpCurrentKeyFrame->fx();
  const float &fy1 = mpCurrentKeyFrame->fy();
  const float &cx1 = mpCurrentKeyFrame->cx();
  const float &cy1 = mpCurrentKeyFrame->cy();
  const auto invfx1 = 1.0f / fx1;
  const auto invfy1 = 1.0f / fy1;

  int nnew = 0;
  int ncandidates = 0;

  // Search matches and triangulate
  for (size_t i = 0; i < vpNeighKFs.size(); i++) {
    if (i > 0 && CheckNewKeyFrames()) return;

    KeyFrame *pKF2 = vpNeighKFs[i];

    // Check first that baseline is not too short
    cv::Mat Ow2 = pKF2->GetCameraCenter();
    cv::Mat vBaseline = Ow2 - Ow1;
    const float baseline = cv::norm(vBaseline);

    const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
    const float ratioBaselineDepth = baseline / medianDepthKF2;

    if (ratioBaselineDepth < 0.01) continue;

    auto matchResult = mFeatureMatcher->MatchFrames(mpCurrentKeyFrame, pKF2);

    cv::Mat Rcw2 = pKF2->GetRotation();
    cv::Mat Rwc2 = Rcw2.t();
    cv::Mat tcw2 = pKF2->GetTranslation();
    cv::Mat Tcw2(3, 4, CV_32F);
    Rcw2.copyTo(Tcw2.colRange(0, 3));
    tcw2.copyTo(Tcw2.col(3));

    const float &fx2 = pKF2->fx();
    const float &fy2 = pKF2->fy();
    const float &cx2 = pKF2->cx();
    const float &cy2 = pKF2->cy();
    const auto invfx2 = 1.0f / fx2;
    const auto invfy2 = 1.0f / fy2;

    // Triangulate each match
    const int nmatches = matchResult.GetNumMatches();
    ncandidates += nmatches;
    for (int ikp = 0; ikp < nmatches; ikp++) {
      cv::Point2f kp1 = matchResult.keyPoints1[ikp];
      cv::Point2f kp2 = matchResult.keyPoints2[ikp];

      // Check parallax between rays
      cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.x - cx1) * invfx1,
                     (kp1.y - cy1) * invfy1, 1.0);
      cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.x - cx2) * invfx2,
                     (kp2.y - cy2) * invfy2, 1.0);

      cv::Mat ray1 = Rwc1 * xn1;
      cv::Mat ray2 = Rwc2 * xn2;
      const float cosParallaxRays =
          ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));

      cv::Mat x3D;
      if (cosParallaxRays > 0 && cosParallaxRays < 0.9998) {
        // Linear Triangulation Method
        cv::Mat A(4, 4, CV_32F);
        A.row(0) = xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0);
        A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
        A.row(2) = xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0);
        A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);

        cv::Mat w, u, vt;
        cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        x3D = vt.row(3).t();

        if (x3D.at<float>(3) == 0) continue;

        // Euclidean coordinates
        x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);

      } else {
        continue;  // very low parallax
      }

      cv::Mat x3Dt = x3D.t();

      // Check triangulation in front of cameras
      float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
      if (z1 <= 0) continue;

      float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
      if (z2 <= 0) continue;

      // Check reprojection error in first keyframe
      const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
      const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
      const float invz1 = 1.0 / z1;

      float u1 = fx1 * x1 * invz1 + cx1;
      float v1 = fy1 * y1 * invz1 + cy1;
      float errX1 = u1 - kp1.x;
      float errY1 = v1 - kp1.y;
      if ((errX1 * errX1 + errY1 * errY1) > 5.991) {
        continue;
      }

      // Check reprojection error in second keyframe
      const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
      const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
      const float invz2 = 1.0 / z2;

      float u2 = fx2 * x2 * invz2 + cx2;
      float v2 = fy2 * y2 * invz2 + cy2;
      float errX2 = u2 - kp2.x;
      float errY2 = v2 - kp2.y;
      if ((errX2 * errX2 + errY2 * errY2) > 5.991) continue;

      // Triangulation is succesfull
      MapPoint *pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpMap);

      pMP->AddObservation(mpCurrentKeyFrame, matchResult.keyPoints1[ikp]);
      pMP->AddObservation(pKF2, matchResult.keyPoints2[ikp]);

      mpCurrentKeyFrame->AddMapPoint(pMP, matchResult.keyPoints1[ikp]);
      pKF2->AddMapPoint(pMP, matchResult.keyPoints2[ikp]);

      pMP->UpdateNormalAndDepth();

      mpMap->AddMapPoint(pMP);
      mlpRecentAddedMapPoints.push_back(pMP);

      nnew++;
    }

    if (nnew > 0) {
      std::cout << "New MPs created " << nnew << std::endl;
    } else {
      std::cout << "Failed to create new MPs, candidates " << ncandidates
                << std::endl;
    }
  }
}

void LocalMapping::SearchInNeighbors() {
  // Retrieve neighbor keyframes
  int nn = 20;
  const std::vector<KeyFrame *> vpNeighKFs =
      mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
  std::vector<KeyFrame *> vpTargetKFs;
  for (std::vector<KeyFrame *>::const_iterator vit = vpNeighKFs.begin(),
                                               vend = vpNeighKFs.end();
       vit != vend; vit++) {
    KeyFrame *pKFi = *vit;
    if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
      continue;
    vpTargetKFs.push_back(pKFi);
    pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

    // Extend to some second neighbors
    const std::vector<KeyFrame *> vpSecondNeighKFs =
        pKFi->GetBestCovisibilityKeyFrames(5);
    for (std::vector<KeyFrame *>::const_iterator
             vit2 = vpSecondNeighKFs.begin(),
             vend2 = vpSecondNeighKFs.end();
         vit2 != vend2; vit2++) {
      KeyFrame *pKFi2 = *vit2;
      if (pKFi2->isBad() ||
          pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId ||
          pKFi2->mnId == mpCurrentKeyFrame->mnId)
        continue;
      vpTargetKFs.push_back(pKFi2);
    }
  }

  // Fuse matches from current KF in target KFs
  for (std::vector<KeyFrame *>::iterator vit = vpTargetKFs.begin(),
                                         vend = vpTargetKFs.end();
       vit != vend; vit++) {
    KeyFrame *pKFi = *vit;

    auto matchResult = mFeatureMatcher->MatchFrames(mpCurrentKeyFrame, pKFi);
    auto nmatches = matchResult.GetNumMatches();
    for (int i = 0; i < nmatches; ++i) {
      MapPoint *pMP1 = matchResult.GetMapPoint1(i);
      MapPoint *pMP2 = matchResult.GetMapPoint2(i);
      if (pMP1 && pMP2) {
        if (!pMP1->isBad() && !pMP2->isBad()) {
          if (pMP2->Observations() > pMP1->Observations())
            pMP1->Replace(pMP2);
          else
            pMP2->Replace(pMP1);
          pMP1->UpdateNormalAndDepth();
          pMP2->UpdateNormalAndDepth();
        }
      } else if (!pMP1 && pMP2) {
        pMP2->AddObservation(mpCurrentKeyFrame, matchResult.keyPoints1[i]);
        mpCurrentKeyFrame->mKeyPointMap.SetMapPoint(matchResult.keyPoints1[i],
                                                    pMP2);
        if (!pMP2->isBad()) {
          pMP2->UpdateNormalAndDepth();
        }
      } else if (pMP1 && !pMP2) {
        pMP1->AddObservation(pKFi, matchResult.keyPoints2[i]);
        pKFi->mKeyPointMap.SetMapPoint(matchResult.keyPoints2[i], pMP1);
        if (!pMP1->isBad()) {
          pMP1->UpdateNormalAndDepth();
        }
      }
    }
  }

  // Update connections in covisibility graph
  mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2) {
  cv::Mat R1w = pKF1->GetRotation();
  cv::Mat t1w = pKF1->GetTranslation();
  cv::Mat R2w = pKF2->GetRotation();
  cv::Mat t2w = pKF2->GetTranslation();

  cv::Mat R12 = R1w * R2w.t();
  cv::Mat t12 = -R1w * R2w.t() * t2w + t1w;

  cv::Mat t12x = SkewSymmetricMatrix(t12);

  const cv::Mat &K1 = pKF1->mK;
  const cv::Mat &K2 = pKF2->mK;

  return K1.t().inv() * t12x * R12 * K2.inv();
}

void LocalMapping::RequestStop() {
  std::unique_lock<std::mutex> lock(mMutexStop);
  mbStopRequested = true;
  std::unique_lock<std::mutex> lock2(mMutexNewKFs);
  mbAbortBA = true;
}

bool LocalMapping::Stop() {
  std::unique_lock<std::mutex> lock(mMutexStop);
  if (mbStopRequested && !mbNotStop) {
    mbStopped = true;
    std::cout << "Local Mapping STOP" << std::endl;
    return true;
  }

  return false;
}

bool LocalMapping::isStopped() {
  std::unique_lock<std::mutex> lock(mMutexStop);
  return mbStopped;
}

bool LocalMapping::stopRequested() {
  std::unique_lock<std::mutex> lock(mMutexStop);
  return mbStopRequested;
}

void LocalMapping::Release() {
  std::unique_lock<std::mutex> lock(mMutexStop);
  std::unique_lock<std::mutex> lock2(mMutexFinish);
  if (mbFinished) return;
  mbStopped = false;
  mbStopRequested = false;
  for (std::list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(),
                                       lend = mlNewKeyFrames.end();
       lit != lend; lit++)
    delete *lit;
  mlNewKeyFrames.clear();

  std::cout << "Local Mapping RELEASE" << std::endl;
}

bool LocalMapping::AcceptKeyFrames() {
  std::unique_lock<std::mutex> lock(mMutexAccept);
  return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag) {
  std::unique_lock<std::mutex> lock(mMutexAccept);
  mbAcceptKeyFrames = flag;
}

bool LocalMapping::SetNotStop(bool flag) {
  std::unique_lock<std::mutex> lock(mMutexStop);

  if (flag && mbStopped) return false;

  mbNotStop = flag;

  return true;
}

void LocalMapping::InterruptBA() { mbAbortBA = true; }

void LocalMapping::KeyFrameCulling() {
  // Check redundant keyframes (only local keyframes)
  // A keyframe is considered redundant if the 90% of the MapPoints it sees, are
  // seen in at least other 3 keyframes (in the same or finer scale) We only
  // consider close stereo points
  std::vector<KeyFrame *> vpLocalKeyFrames =
      mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

  for (std::vector<KeyFrame *>::iterator vit = vpLocalKeyFrames.begin(),
                                         vend = vpLocalKeyFrames.end();
       vit != vend; vit++) {
    KeyFrame *pKF = *vit;
    if (pKF->mnId == 0) continue;
    auto vpMapPoints = pKF->GetMapPointMatches();

    const int thObs = 3;
    int nRedundantObservations = 0;
    int nMPs = 0;
    for (auto i = vpMapPoints.Begin(), iend = vpMapPoints.End(); i != iend;
         ++i) {
      MapPoint *pMP = i->second.mapPoint;
      if (pMP) {
        if (!pMP->isBad()) {
          nMPs++;
          if (pMP->Observations() > thObs) {
            auto observations = pMP->GetObservations();
            int nObs = 0;
            for (auto mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
              KeyFrame *pKFi = mit->first;
              if (pKFi == pKF) {
                continue;
              }
              nObs++;
              if (nObs >= thObs) {
                break;
              }
            }
            if (nObs >= thObs) {
              nRedundantObservations++;
            }
          }
        }
      }
    }

    if (nRedundantObservations > 0.9 * nMPs) pKF->SetBadFlag();
  }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v) {
  return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1),
          v.at<float>(2), 0, -v.at<float>(0), -v.at<float>(1), v.at<float>(0),
          0);
}

void LocalMapping::RequestReset() {
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbResetRequested = true;
  }

  while (1) {
    {
      std::unique_lock<std::mutex> lock2(mMutexReset);
      if (!mbResetRequested) break;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(3000));
  }
}

void LocalMapping::ResetIfRequested() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  if (mbResetRequested) {
    mlNewKeyFrames.clear();
    mlpRecentAddedMapPoints.clear();
    mbResetRequested = false;
  }
}

void LocalMapping::RequestFinish() {
  std::unique_lock<std::mutex> lock(mMutexFinish);
  mbFinishRequested = true;
}

bool LocalMapping::CheckFinish() {
  std::unique_lock<std::mutex> lock(mMutexFinish);
  return mbFinishRequested;
}

void LocalMapping::SetFinish() {
  std::unique_lock<std::mutex> lock(mMutexFinish);
  mbFinished = true;
  std::unique_lock<std::mutex> lock2(mMutexStop);
  mbStopped = true;
}

bool LocalMapping::isFinished() {
  std::unique_lock<std::mutex> lock(mMutexFinish);
  return mbFinished;
}

}  // namespace SLAM_PIPELINE
