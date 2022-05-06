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
#include "Optimizer.h"
#include "Sim3Solver.h"

namespace SLAM_PIPELINE {

LoopClosing::LoopClosing(Map* pMap, KeyFrameDatabase* pDB,
                         FeatureMatcher* featureMatcher,
                         const FeatureParameters& parameters)
    : mpMap(pMap),
      mpKeyFrameDB(pDB),
      mpMatchedKF(NULL),
      mLastLoopKFid(0),
      mnFullBAIdx(false),
      mFeatureMatcher(featureMatcher) {
  mbFixScale = parameters.bFixScale;
  mnCovisibilityConsistencyTh = parameters.nCovisibilityConsistencyTh;
  mLoopDetectionMaxFrames = parameters.loopDetectionMaxFrames;
  mMinSim3Matches = parameters.minSim3Matches;
  mMinTotalSim3Matches = parameters.minTotalSim3Matches;
}

void LoopClosing::SetLocalMapper(LocalMapping* pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void LoopClosing::Run() {
  // Check if there are keyframes in the queue
  if (CheckNewKeyFrames()) {
    // Detect loop candidates and check covisibility consistency
    if (DetectLoop()) {
      // Compute similarity transformation [sR|t]
      // In the stereo/RGBD case s=1
      if (ComputeSim3()) {
        // Perform loop fusion and pose graph optimization
        CorrectLoop();
      }
    }
  }
}

void LoopClosing::InsertKeyFrame(KeyFrame* pKF) {
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

  // Compute reference similarity score
  // This is the lowest num matches to a connected keyframe in the covisibility graph
  // We will impose loop candidates to have a higher similarity than this
  const std::vector<KeyFrame*> vpConnectedKeyFrames =
      mpCurrentKF->GetVectorCovisibleKeyFrames();
  size_t minNumCovisibleMatches = 1;
  bool newSearch = true;
  for (size_t i = 0; i < vpConnectedKeyFrames.size(); i++) {
    KeyFrame* pKF = vpConnectedKeyFrames[i];
    if (pKF->isBad()) continue;
    auto matchResult = mFeatureMatcher->MatchFrames(mpCurrentKF, pKF);
    auto numMatches = matchResult.GetNumMatches();
    if (newSearch) {
      minNumCovisibleMatches = numMatches;
      newSearch = false;
    }
    if (numMatches < minNumCovisibleMatches)
      minNumCovisibleMatches = numMatches;
  }

  // Query the database imposing the minimum num matches
  std::vector<KeyFrame*> vpCandidateKFs =
      mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minNumCovisibleMatches);

  // If there are no loop candidates, just add new keyframe and return false
  if (vpCandidateKFs.empty()) {
    mpKeyFrameDB->add(mpCurrentKF);
    mvConsistentGroups.clear();
    mpCurrentKF->SetErase();
    return false;
  }

  // For each loop candidate check consistency with previous loop candidates
  // Each candidate expands a covisibility group (keyframes connected to the
  // loop candidate in the covisibility graph) A group is consistent with a
  // previous group if they share at least a keyframe We must detect a
  // consistent loop in several consecutive keyframes to accept it
  mvpEnoughConsistentCandidates.clear();

  std::vector<ConsistentGroup> vCurrentConsistentGroups;
  std::vector<bool> vbConsistentGroup(mvConsistentGroups.size(), false);
  for (size_t i = 0, iend = vpCandidateKFs.size(); i < iend; i++) {
    KeyFrame* pCandidateKF = vpCandidateKFs[i];

    std::set<KeyFrame*> spCandidateGroup =
        pCandidateKF->GetConnectedKeyFrames();
    spCandidateGroup.insert(pCandidateKF);

    bool bEnoughConsistent = false;
    bool bConsistentForSomeGroup = false;
    for (size_t iG = 0, iendG = mvConsistentGroups.size(); iG < iendG; iG++) {
      std::set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

      bool bConsistent = false;
      for (std::set<KeyFrame*>::iterator sit = spCandidateGroup.begin(),
                                         send = spCandidateGroup.end();
           sit != send; sit++) {
        if (sPreviousGroup.count(*sit)) {
          bConsistent = true;
          bConsistentForSomeGroup = true;
          break;
        }
      }

      if (bConsistent) {
        int nPreviousConsistency = mvConsistentGroups[iG].second;
        int nCurrentConsistency = nPreviousConsistency + 1;
        if (!vbConsistentGroup[iG]) {
          ConsistentGroup cg = make_pair(spCandidateGroup, nCurrentConsistency);
          vCurrentConsistentGroups.push_back(cg);
          vbConsistentGroup[iG] =
              true;  // this avoid to include the same group more than once
        }
        if (nCurrentConsistency >= mnCovisibilityConsistencyTh &&
            !bEnoughConsistent) {
          mvpEnoughConsistentCandidates.push_back(pCandidateKF);
          bEnoughConsistent =
              true;  // this avoid to insert the same candidate more than once
        }
      }
    }

    // If the group is not consistent with any previous group insert with
    // consistency counter set to zero
    if (!bConsistentForSomeGroup) {
      ConsistentGroup cg = make_pair(spCandidateGroup, 0);
      vCurrentConsistentGroups.push_back(cg);
    }
  }

  // Update Covisibility Consistent Groups
  mvConsistentGroups = vCurrentConsistentGroups;

  // Add Current Keyframe to database
  mpKeyFrameDB->add(mpCurrentKF);

  if (mvpEnoughConsistentCandidates.empty()) {
    mpCurrentKF->SetErase();
    return false;
  } else {
    return true;
  }

  mpCurrentKF->SetErase();
  return false;
}

bool LoopClosing::ComputeSim3() {
  // For each consistent loop candidate we try to compute a Sim3

  const auto nInitialCandidates = mvpEnoughConsistentCandidates.size();

  // We compute first matches for each candidate
  // If enough matches are found, we setup a Sim3Solver

  std::vector<Sim3Solver*> vpSim3Solvers;
  vpSim3Solvers.resize(nInitialCandidates);

  std::vector<MatchFramesResult> vvpMapPointMatches;
  vvpMapPointMatches.resize(nInitialCandidates);

  std::vector<bool> vbDiscarded;
  vbDiscarded.resize(nInitialCandidates);

  int nCandidates = 0;  // candidates with enough matches

  for (int i = 0; i < nInitialCandidates; i++) {
    KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

    // avoid that local mapping erase it while it is being processed in this
    // thread
    pKF->SetNotErase();

    if (pKF->isBad()) {
      vbDiscarded[i] = true;
      continue;
    }

    vvpMapPointMatches[i] = mFeatureMatcher->MatchFrames(mpCurrentKF, pKF);
    auto nmatches = vvpMapPointMatches[i].GetNumMatches();

    if (nmatches < mMinSim3Matches) {
      vbDiscarded[i] = true;
      continue;
    } else {
      Sim3Solver* pSolver =
          new Sim3Solver(mpCurrentKF, pKF, vvpMapPointMatches[i], mbFixScale);
      pSolver->SetRansacParameters(0.99, 20, 300);
      vpSim3Solvers[i] = pSolver;
    }

    nCandidates++;
  }

  bool bMatch = false;

  // Perform alternatively RANSAC iterations for each candidate
  // until one is succesful or all fail
  while (nCandidates > 0 && !bMatch) {
    for (int i = 0; i < nInitialCandidates; i++) {
      if (vbDiscarded[i]) continue;

      // Perform 5 Ransac Iterations
      std::vector<bool> vbInliers;
      bool bNoMore = false;
      cv::Mat Scm;

      Sim3Solver* pSolver = vpSim3Solvers[i];
      {
        int nInliers;
        Scm = pSolver->iterate(5, bNoMore, vbInliers, nInliers);
      }

      // If Ransac reachs max. iterations discard keyframe
      if (bNoMore) {
        vbDiscarded[i] = true;
        nCandidates--;
      }

      // If RANSAC returns a Sim3, perform a guided matching and optimize with
      // all correspondences
      if (!Scm.empty()) {
        auto vpMapPointMatches = vvpMapPointMatches[i];
        for (size_t j = 0, jend = vbInliers.size(); j < jend; j++) {
          if (vbInliers[j]) {
            vpMapPointMatches.DeleteMatch(j);
          }
        }

        cv::Mat R = pSolver->GetEstimatedRotation();
        cv::Mat t = pSolver->GetEstimatedTranslation();
        const float s = pSolver->GetEstimatedScale();
        g2o::Sim3 gScm(Converter::toMatrix3d(R), Converter::toVector3d(t), s);
        const int nInliers =
            Optimizer::OptimizeSim3(vpMapPointMatches, gScm, 10, mbFixScale);

        // If optimization is succesful stop ransacs and continue
        if (nInliers >= mMinSim3Matches) {
          bMatch = true;
          mpMatchedKF = mvpEnoughConsistentCandidates[i];
          g2o::Sim3 gSmw(Converter::toMatrix3d(mpMatchedKF->GetRotation()),
                         Converter::toVector3d(mpMatchedKF->GetTranslation()),
                         1.0);
          mg2oScw = gScm * gSmw;
          mScw = Converter::toCvMat(mg2oScw);

          mvpCurrentMatchedPoints = vpMapPointMatches;
          break;
        }
      }
    }
  }

  if (!bMatch) {
    for (int i = 0; i < nInitialCandidates; i++)
      mvpEnoughConsistentCandidates[i]->SetErase();
    mpCurrentKF->SetErase();
    return false;
  }

  // If enough matches accept Loop
  auto nTotalMatches = mvpCurrentMatchedPoints.GetNumMatches();

  if (nTotalMatches >= mMinTotalSim3Matches) {
    for (int i = 0; i < nInitialCandidates; i++)
      if (mvpEnoughConsistentCandidates[i] != mpMatchedKF)
        mvpEnoughConsistentCandidates[i]->SetErase();
    return true;
  } else {
    for (int i = 0; i < nInitialCandidates; i++)
      mvpEnoughConsistentCandidates[i]->SetErase();
    mpCurrentKF->SetErase();
    return false;
  }
}

void LoopClosing::CorrectLoop() {
  std::cout << "Loop detected!" << std::endl;

  mnFullBAIdx = true;

  // Ensure current keyframe is updated
  mpCurrentKF->UpdateConnections();

  // Retrive keyframes connected to the current keyframe and compute corrected
  // Sim3 pose by propagation
  mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
  mvpCurrentConnectedKFs.push_back(mpCurrentKF);

  KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
  CorrectedSim3[mpCurrentKF] = mg2oScw;
  cv::Mat Twc = mpCurrentKF->GetPoseInverse();

  for (std::vector<KeyFrame*>::iterator vit = mvpCurrentConnectedKFs.begin(),
                                        vend = mvpCurrentConnectedKFs.end();
       vit != vend; vit++) {
    KeyFrame* pKFi = *vit;

    cv::Mat Tiw = pKFi->GetPose();

    if (pKFi != mpCurrentKF) {
      cv::Mat Tic = Tiw * Twc;
      cv::Mat Ric = Tic.rowRange(0, 3).colRange(0, 3);
      cv::Mat tic = Tic.rowRange(0, 3).col(3);
      g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric), Converter::toVector3d(tic),
                       1.0);
      g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oScw;
      // Pose corrected with the Sim3 of the loop closure
      CorrectedSim3[pKFi] = g2oCorrectedSiw;
    }

    cv::Mat Riw = Tiw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tiw = Tiw.rowRange(0, 3).col(3);
    g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw), Converter::toVector3d(tiw),
                     1.0);
    // Pose without correction
    NonCorrectedSim3[pKFi] = g2oSiw;

    // Correct all MapPoints obsrved by current keyframe and neighbors, so that
    // they align with the other side of the loop
    for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(),
                                   mend = CorrectedSim3.end();
         mit != mend; mit++) {
      KeyFrame* pKFi = mit->first;
      g2o::Sim3 g2oCorrectedSiw = mit->second;
      g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

      g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

      auto vpMPsi = pKFi->GetMapPointMatches();
      for (auto iMP = vpMPsi.Begin(), endMPi = vpMPsi.End(); iMP != endMPi;
           ++iMP) {
        MapPoint* pMPi = iMP->second.mapPoint;
        if (!pMPi) continue;
        if (pMPi->isBad()) continue;
        if (pMPi->mnCorrectedByKF == mpCurrentKF->id()) continue;

        // Project with non-corrected pose and project back with corrected pose
        cv::Mat P3Dw = pMPi->GetWorldPos();
        Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
            g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMPi->SetWorldPos(cvCorrectedP3Dw);
        pMPi->mnCorrectedByKF = mpCurrentKF->id();
        pMPi->mnCorrectedReference = pKFi->id();
        pMPi->UpdateNormalAndDepth();
      }

      // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3
      // (scale translation)
      Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
      Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
      double s = g2oCorrectedSiw.scale();

      eigt *= (1. / s);  //[R t/s;0 1]

      cv::Mat correctedTiw = Converter::toCvSE3(eigR, eigt);

      pKFi->SetPose(correctedTiw);

      // Make sure connections are updated
      pKFi->UpdateConnections();
    }
  }

  // Add loop edge
  mpMatchedKF->AddLoopEdge(mpCurrentKF);
  mpCurrentKF->AddLoopEdge(mpMatchedKF);

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
  std::list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),
                                   mpMap->mvpKeyFrameOrigins.end());

  while (!lpKFtoCheck.empty()) {
    KeyFrame* pKF = lpKFtoCheck.front();
    const std::set<KeyFrame*> sChilds = pKF->GetChilds();
    cv::Mat Twc = pKF->GetPoseInverse();
    for (std::set<KeyFrame*>::const_iterator sit = sChilds.begin();
         sit != sChilds.end(); sit++) {
      KeyFrame* pChild = *sit;
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
  const std::vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

  for (size_t i = 0; i < vpMPs.size(); i++) {
    MapPoint* pMP = vpMPs[i];

    if (pMP->isBad()) continue;

    if (pMP->mnBAGlobalForKF == nLoopKF) {
      // If optimized by Global BA, just update
      pMP->SetWorldPos(pMP->mPosGBA);
    } else {
      // Update according to the correction of its reference keyframe
      KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

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
