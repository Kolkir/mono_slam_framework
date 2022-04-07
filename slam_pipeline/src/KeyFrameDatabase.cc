#include "KeyFrameDatabase.h"

#include <mutex>

#include "FeatureMatcher.h"
#include "Frame.h"
#include "KeyFrame.h"

namespace SLAM_PIPELINE {

KeyFrameMatchDatabase::KeyFrameMatchDatabase(FeatureMatcher* featureMatcher)
    : mFeatureMatcher(featureMatcher) {}

void KeyFrameMatchDatabase::add(KeyFrame* pKF) {
  std::unique_lock<std::mutex> lock(mGuard);
  mFrames.push_back(pKF);
}

void KeyFrameMatchDatabase::erase(KeyFrame* pKF) {
  std::unique_lock<std::mutex> lock(mGuard);
  auto i = std::find(mFrames.begin(), mFrames.end(), pKF);
  if (i != mFrames.end()) {
    mFrames.erase(i);
  }
}

void KeyFrameMatchDatabase::clear() { mFrames.clear(); }

std::vector<KeyFrame*> KeyFrameMatchDatabase::DetectLoopCandidates(
    KeyFrame* pKF, float minScore) {
  return {};
}

std::vector<KeyFrame*> KeyFrameMatchDatabase::DetectRelocalizationCandidates(
    Frame* F) {
  std::vector<std::pair<KeyFrame*, size_t>> frameMatchCounts;
  frameMatchCounts.reserve(mFrames.size() / 3);
  int maxNumMatches = 0;
  {
    std::unique_lock<std::mutex> lock(mGuard);
    // calculate number of feature matches for every key frame and frame
    // also search for max number of matches
    for (auto pKFi : mFrames) {
      auto matchResult = mFeatureMatcher->MatchFrames(F, pKFi);
      auto numMatches = static_cast<int>(matchResult.GetNumMatches());
      pKFi->mnRelocQuery = F->mnId;
      pKFi->mRelocScore = static_cast<float>(numMatches);
      frameMatchCounts.emplace_back(pKFi, numMatches);
      if (numMatches > maxNumMatches) maxNumMatches = numMatches;
    }
  }
  int minNumMatches = static_cast<int>(maxNumMatches * 0.8f);

  // accumulate num matches by covisibility
  float bestAccNumMatches = 0;
  std::vector<std::pair<KeyFrame*, float>> accNumMatchFrames;
  for (auto& kfItem : frameMatchCounts) {
    // consider key frames only with enough matches
    if (kfItem.second >= minNumMatches) {
      KeyFrame* pKFi = kfItem.first;
      auto vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

      float bestNumMatches = static_cast<float>(kfItem.second);
      float accNumMatches = bestNumMatches;
      KeyFrame* pBestKF = pKFi;
      for (auto pKF2 : vpNeighs) {
        if (pKF2->mnRelocQuery != F->mnId) {
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
  std::set<KeyFrame*> spAlreadyAddedKF;
  std::vector<KeyFrame*> vpRelocCandidates;
  vpRelocCandidates.reserve(accNumMatchFrames.size());
  for (auto& kfItem : accNumMatchFrames) {
    float numMatches = static_cast<float>(kfItem.second);
    if (numMatches > minNumMatchesToRetain) {
      KeyFrame* pKFi = kfItem.first;
      if (!spAlreadyAddedKF.count(pKFi)) {
        vpRelocCandidates.push_back(pKFi);
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }
  return vpRelocCandidates;
}

}  // namespace SLAM_PIPELINE
