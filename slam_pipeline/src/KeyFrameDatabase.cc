#include "KeyFrameDatabase.h"

#include "FeatureMatcher.h"
#include "Frame.h"
#include "KeyFrame.h"

namespace SLAM_PIPELINE {

KeyFrameMatchDatabase::KeyFrameMatchDatabase(FeatureMatcher* featureMatcher)
    : mFeatureMatcher(featureMatcher) {}

void KeyFrameMatchDatabase::add(KeyFramePtr pKF) { mFrames.push_back(pKF); }

void KeyFrameMatchDatabase::erase(KeyFramePtr pKF) {
  auto i = std::find(mFrames.begin(), mFrames.end(), pKF);
  if (i != mFrames.end()) {
    mFrames.erase(i);
  }
}

void KeyFrameMatchDatabase::clear() { mFrames.clear(); }

KeyFramePtr KeyFrameMatchDatabase::DetectLoopCandidate(KeyFrame& pKF,
                                                       size_t minNumMPMatches) {
  std::set<KeyFramePtr> spConnectedKeyFrames = pKF.GetConnectedKeyFrames();
  KeyFramePtr loopCandidate;
  size_t maxNumMP = 0;

  // Search all keyframes that share features with current keyframes
  // Discard keyframes connected to the query keyframe
  for (auto pKFi : mFrames) {
    auto matchResult = mFeatureMatcher->MatchFrames(pKF, *pKFi);
    auto numMatches = matchResult.GetNumMatches();
    if (numMatches != 0 && pKFi->mnLoopQuery != pKF.id()) {
      if (!spConnectedKeyFrames.count(pKFi)) {
        size_t numMP = 0;
        for (size_t i = 0; i < numMatches; ++i) {
          MapPointPtr pMP1 = matchResult.GetMapPoint1(i);
          MapPointPtr pMP2 = matchResult.GetMapPoint2(i);
          if (pMP1 && pMP2) {
            ++numMP;
          }
        }
        if (numMP > minNumMPMatches && numMP > maxNumMP) {
          loopCandidate = pKFi;
          maxNumMP = numMP;
        }
      }
    }
  }

  return loopCandidate;
}

std::vector<KeyFramePtr> KeyFrameMatchDatabase::DetectRelocalizationCandidates(
    FrameBase& pF) {
  std::vector<std::pair<KeyFramePtr, size_t>> frameMatchCounts;
  frameMatchCounts.reserve(mFrames.size() / 3);

  size_t maxNumMatches = 0;
  // calculate number of feature matches for every key frame and frame
  // also search for max number of matches
  for (auto pKFi : mFrames) {
    auto matchResult = mFeatureMatcher->MatchFrames(pF, *pKFi);
    auto numMatches = static_cast<int>(matchResult.GetNumMatches());
    pKFi->mnRelocQuery = pF.id();
    pKFi->mRelocScore = static_cast<float>(numMatches);
    frameMatchCounts.emplace_back(pKFi, numMatches);
    if (numMatches > maxNumMatches) maxNumMatches = numMatches;
  }
  auto minNumMatches = static_cast<size_t>(maxNumMatches * 0.8f);

  // accumulate num matches by covisibility
  float bestAccNumMatches = 0;
  std::vector<std::pair<KeyFramePtr, float>> accNumMatchFrames;
  for (auto& kfItem : frameMatchCounts) {
    // consider key frames only with enough matches
    if (kfItem.second >= minNumMatches) {
      KeyFramePtr pKFi = kfItem.first;
      auto vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

      float bestNumMatches = static_cast<float>(kfItem.second);
      float accNumMatches = bestNumMatches;
      KeyFramePtr pBestKF = pKFi;
      for (auto pKF2 : vpNeighs) {
        if (pKF2->mnRelocQuery != pF.id()) {
          continue;
        }

        accNumMatches += pKF2->mRelocScore;
        if (pKF2->mRelocScore > bestNumMatches) {
          pBestKF = pKF2;
          bestNumMatches = pKF2->mRelocScore;
        }
      }
      accNumMatchFrames.emplace_back(pBestKF, accNumMatches);
      if (accNumMatches > bestAccNumMatches) bestAccNumMatches = accNumMatches;
    }
  }

  // Return all those keyframes with a score higher than 0.75*bestAccNumMatches
  float minNumMatchesToRetain = 0.75f * bestAccNumMatches;
  std::set<KeyFramePtr> spAlreadyAddedKF;
  std::vector<KeyFramePtr> vpRelocCandidates;
  vpRelocCandidates.reserve(accNumMatchFrames.size());
  for (auto& kfItem : accNumMatchFrames) {
    float numMatches = static_cast<float>(kfItem.second);
    if (numMatches > minNumMatchesToRetain) {
      KeyFramePtr pKFi = kfItem.first;
      if (!spAlreadyAddedKF.count(pKFi)) {
        vpRelocCandidates.push_back(pKFi);
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }
  return vpRelocCandidates;
}

}  // namespace SLAM_PIPELINE
