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

#include "FeatureMatcher.h"
#include "KeyFrame.h"
#include "KeyPointMap.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"
#include "Optimizer.h"

namespace SLAM_PIPELINE {

LocalMapping::LocalMapping(Map *pMap, FeatureMatcher *featureMatcher,
                           const SlamParameters &parameters)
    : mpMap(pMap), mFeatureMatcher(featureMatcher) {
  mMinParallax = parameters.minimumParallax;
}

void LocalMapping::SetLoopCloser(LoopClosing *pLoopCloser) {
  mpLoopCloser = pLoopCloser;
}

void LocalMapping::Run() {
  // Check if there are keyframes in the queue
  if (CheckNewKeyFrames()) {
    ProcessNewKeyFrame();

    // Check recent MapPoints
    MapPointCulling();

    // Triangulate new MapPoints
    CreateNewMapPoints();

    // Find more matches in neighbor keyframes and fuse point duplications
    SearchInNeighbors();

    // Local BA
    if (mpMap->KeyFramesInMap() > 2) {
      bool bAbortBA = false;
      Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &bAbortBA);
    }

    std::cout << "Local BA done" << std::endl;

    // Check redundant local Keyframes
    KeyFrameCulling();

    mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
  }
}

void LocalMapping::InsertKeyFrame(KeyFramePtr pKF) {
  mlNewKeyFrames.push_back(pKF);
}

bool LocalMapping::CheckNewKeyFrames() { return (!mlNewKeyFrames.empty()); }

void LocalMapping::ProcessNewKeyFrame() {
  mpCurrentKeyFrame = mlNewKeyFrames.front();
  mlNewKeyFrames.pop_front();

  // Associate MapPoints to the new keyframe and update normal
  auto vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
  auto start = vpMapPointMatches.Begin();
  auto end = vpMapPointMatches.End();
  for (auto i = start; i != end; ++i) {
    MapPointPtr pMP = i->second.mapPoint;
    if (pMP) {
      if (!pMP->isBad()) {
        if (!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
          pMP->AddObservation(mpCurrentKeyFrame,
                              vpMapPointMatches.KeyPointFromIndex(i->first));
          pMP->UpdateNormalAndDepth();
        } else  // this can only happen for new points inserted by the
                // initialization
        {
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
  auto lit = mlpRecentAddedMapPoints.begin();
  const auto nCurrentKFid = mpCurrentKeyFrame->id();

  int nThObs = 2;
  const int cnThObs = nThObs;

  while (lit != mlpRecentAddedMapPoints.end()) {
    MapPointPtr pMP = *lit;
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
  const std::vector<KeyFramePtr> vpNeighKFs =
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

  size_t nnew = 0;
  size_t ncandidates = 0;

  // Search matches and triangulate
  for (size_t i = 0; i < vpNeighKFs.size(); i++) {
    if (i > 0 && CheckNewKeyFrames()) return;

    KeyFramePtr pKF2 = vpNeighKFs[i];

    // Check first that baseline is not too short
    cv::Mat Ow2 = pKF2->GetCameraCenter();
    cv::Mat vBaseline = Ow2 - Ow1;
    const auto baseline = cv::norm(vBaseline);

    const auto medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
    const auto ratioBaselineDepth = baseline / medianDepthKF2;

    if (ratioBaselineDepth < 0.01) continue;

    auto matchResult = mFeatureMatcher->MatchFrames(*mpCurrentKeyFrame, *pKF2);

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
    const auto nmatches = matchResult.GetNumMatches();
    ncandidates += nmatches;
    for (size_t ikp = 0; ikp < nmatches; ikp++) {
      cv::Point2f kp1 = matchResult.keyPoints1[ikp];
      cv::Point2f kp2 = matchResult.keyPoints2[ikp];

      // Check parallax between rays
      cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.x - cx1) * invfx1,
                     (kp1.y - cy1) * invfy1, 1.0);
      cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.x - cx2) * invfx2,
                     (kp2.y - cy2) * invfy2, 1.0);

      cv::Mat ray1 = Rwc1 * xn1;
      cv::Mat ray2 = Rwc2 * xn2;
      const auto cosParallaxRays =
          ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));

      cv::Mat x3D;
      if (cosParallaxRays > 0 && cosParallaxRays < mMinParallax) {
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
      auto z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
      if (z1 <= 0) continue;

      auto z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
      if (z2 <= 0) continue;

      // Check reprojection error in first keyframe
      const auto x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
      const auto y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
      const auto invz1 = 1.0 / z1;

      auto u1 = fx1 * x1 * invz1 + cx1;
      auto v1 = fy1 * y1 * invz1 + cy1;
      auto errX1 = u1 - kp1.x;
      auto errY1 = v1 - kp1.y;
      if ((errX1 * errX1 + errY1 * errY1) > 5.991) {
        continue;
      }

      // Check reprojection error in second keyframe
      const auto x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
      const auto y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
      const auto invz2 = 1.0 / z2;

      auto u2 = fx2 * x2 * invz2 + cx2;
      auto v2 = fy2 * y2 * invz2 + cy2;
      auto errX2 = u2 - kp2.x;
      auto errY2 = v2 - kp2.y;
      if ((errX2 * errX2 + errY2 * errY2) > 5.991) continue;

      // Triangulation is succesfull
      auto pMP = std::make_shared<MapPoint>(x3D, mpCurrentKeyFrame, mpMap);

      pMP->AddObservation(mpCurrentKeyFrame, matchResult.keyPoints1[ikp]);
      pMP->AddObservation(pKF2, matchResult.keyPoints2[ikp]);

      mpCurrentKeyFrame->AddMapPoint(pMP, matchResult.keyPoints1[ikp]);
      pKF2->AddMapPoint(pMP, matchResult.keyPoints2[ikp]);

      pMP->UpdateNormalAndDepth();

      mpMap->AddMapPoint(pMP);
      mlpRecentAddedMapPoints.push_back(pMP);

      nnew++;
    }
  }
  if (nnew > 0) {
    std::cout << "New MPs created " << nnew << std::endl;
  } else {
    std::cout << "Failed to create new MPs, candidates " << ncandidates
              << std::endl;
  }
  // show map statistics
  auto kfNum = mpMap->KeyFramesInMap();
  std::cout << "KF num " << kfNum << std::endl;
  auto mapPointsNum = mpMap->MapPointsInMap();
  std::cout << "MP num " << mapPointsNum << std::endl;
}

void LocalMapping::SearchInNeighbors() {
  // Retrieve neighbor keyframes
  int nn = 20;
  const std::vector<KeyFramePtr> vpNeighKFs =
      mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
  std::vector<KeyFramePtr> vpTargetKFs;
  for (auto vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend;
       vit++) {
    KeyFramePtr pKFi = *vit;
    if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->id())
      continue;
    vpTargetKFs.push_back(pKFi);
    pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->id();

    // Extend to some second neighbors
    const std::vector<KeyFramePtr> vpSecondNeighKFs =
        pKFi->GetBestCovisibilityKeyFrames(5);
    for (auto vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end();
         vit2 != vend2; vit2++) {
      KeyFramePtr pKFi2 = *vit2;
      if (pKFi2->isBad() ||
          pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->id() ||
          pKFi2->id() == mpCurrentKeyFrame->id())
        continue;
      vpTargetKFs.push_back(pKFi2);
    }
  }

  // Fuse matches from current KF in target KFs
  for (auto vit = vpTargetKFs.begin(), vend = vpTargetKFs.end(); vit != vend;
       vit++) {
    KeyFramePtr pKFi = *vit;

    auto matchResult = mFeatureMatcher->MatchFrames(*mpCurrentKeyFrame, *pKFi);
    auto nmatches = matchResult.GetNumMatches();
    for (int i = 0; i < nmatches; ++i) {
      MapPointPtr pMP1 = matchResult.GetMapPoint1(i);
      MapPointPtr pMP2 = matchResult.GetMapPoint2(i);
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

void LocalMapping::Release() {
  mlNewKeyFrames.clear();
  std::cout << "Local Mapping RELEASE" << std::endl;
}

void LocalMapping::KeyFrameCulling() {
  // Check redundant keyframes (only local keyframes)
  // A keyframe is considered redundant if the 90% of the MapPoints it sees, are
  // seen in at least other 3 keyframes (in the same or finer scale) We only
  // consider close stereo points
  std::vector<KeyFramePtr> vpLocalKeyFrames =
      mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();
  int nbadframes = 0;
  for (auto vit = vpLocalKeyFrames.begin(), vend = vpLocalKeyFrames.end();
       vit != vend; vit++) {
    KeyFramePtr pKF = *vit;
    if (pKF->id() == 0) continue;
    auto vpMapPoints = pKF->GetMapPointMatches();

    const int thObs = 3;
    int nRedundantObservations = 0;
    int nMPs = 0;
    for (auto i = vpMapPoints.Begin(), iend = vpMapPoints.End(); i != iend;
         ++i) {
      MapPointPtr pMP = i->second.mapPoint;
      if (pMP) {
        if (!pMP->isBad()) {
          nMPs++;
          if (pMP->Observations() > thObs) {
            auto observations = pMP->GetObservations();
            int nObs = 0;
            for (auto mit = observations.begin(), mend = observations.end();
                 mit != mend; mit++) {
              KeyFramePtr pKFi = mit->first;
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

    if (nRedundantObservations > 0.9 * nMPs) {
      pKF->SetBadFlag();
      ++nbadframes;
    }
  }
  std::cout << "Bad KF " << nbadframes << std::endl;
}

void LocalMapping::Reset() {
  mlNewKeyFrames.clear();
  mlpRecentAddedMapPoints.clear();
}

}  // namespace SLAM_PIPELINE
