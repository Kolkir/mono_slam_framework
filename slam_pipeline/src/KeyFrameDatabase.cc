#include "KeyFrameDatabase.h"

#include "FeatureMatcher.h"
#include "Frame.h"
#include "KeyFrame.h"

namespace SLAM_PIPELINE {

KeyFrameMatchDatabase::KeyFrameMatchDatabase(FeatureMatcher* featureMatcher)
    : mFeatureMatcher(featureMatcher) {}

void KeyFrameMatchDatabase::add(KeyFrame* pKF) { mFrames.push_back(pKF); }

void KeyFrameMatchDatabase::erase(KeyFrame* pKF) {
  auto i = std::find(mFrames.begin(), mFrames.end(), pKF);
  if (i != mFrames.end()) {
    mFrames.erase(i);
  }
}

void KeyFrameMatchDatabase::clear() { mFrames.clear(); }

std::vector<KeyFrame*> KeyFrameMatchDatabase::DetectLoopCandidates(
    KeyFrame* pKF, size_t minNumCovisibleMatches) {
  std::set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
  std::vector<KeyFrame*> lKFsSharingFeatures;

  // Search all keyframes that share features with current keyframes
  // Discard keyframes connected to the query keyframe
  for (auto pKFi : mFrames) {
    auto matchResult = mFeatureMatcher->MatchFrames(pKF, pKFi);
    auto numMatches = matchResult.GetNumMatches();
    if (numMatches != 0 && pKFi->mnLoopQuery != pKF->id()) {
      if (!spConnectedKeyFrames.count(pKFi)) {
        pKFi->mnLoopQuery = pKF->id();
        pKFi->mnLoopMatches = numMatches;
        lKFsSharingFeatures.push_back(pKFi);
      }
    }
  }

  if (lKFsSharingFeatures.empty()) return std::vector<KeyFrame*>();

  std::list<KeyFrame*> lMatch;

  size_t maxCommonFeatures = 0;
  for (auto kf : lKFsSharingFeatures) {
    if (kf->mnLoopMatches > maxCommonFeatures)
      maxCommonFeatures = kf->mnLoopMatches;
  }

  auto minCommonFeatures = maxCommonFeatures * 0.8f;

  // Retain the matches whose feature matches num is higher than
  // minCommonFeatures
  for (auto pKFi : lKFsSharingFeatures) {
    if (pKFi->mnLoopMatches > minCommonFeatures) {
      lMatch.push_back(pKFi);
    }
  }

  if (lMatch.empty()) return std::vector<KeyFrame*>();

  std::list<std::pair<size_t, KeyFrame*>> lAccMatch;
  auto bestAccMatches = minNumCovisibleMatches;

  // Lets now accumulate matches by covisibility
  for (auto pKFi : lMatch) {
    auto vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    auto bestMatches = pKFi->mnLoopMatches;
    auto accMatches = pKFi->mnLoopMatches;
    auto pBestKF = pKFi;
    for (auto pKF2 : vpNeighs) {
      if (pKF2->mnLoopQuery == pKF->id() &&
          pKF2->mnLoopMatches > minNumCovisibleMatches) {
        accMatches += pKF2->mnLoopMatches;
        if (pKF2->mnLoopMatches > bestMatches) {
          pBestKF = pKF2;
          bestMatches = pKF2->mnLoopMatches;
        }
      }
    }

    lAccMatch.push_back(std::make_pair(accMatches, pBestKF));
    if (accMatches > bestAccMatches) bestAccMatches = accMatches;
  }

  // Return all those keyframes with a matches higher than 0.75 * bestMatches
  float minMatchesToRetain = 0.75f * bestAccMatches;

  std::set<KeyFrame*> spAlreadyAddedKF;
  std::vector<KeyFrame*> vpLoopCandidates;
  vpLoopCandidates.reserve(lAccMatch.size());

  for (auto it = lAccMatch.begin(), itend = lAccMatch.end(); it != itend;
       it++) {
    if (it->first > minMatchesToRetain) {
      KeyFrame* pKFi = it->second;
      if (!spAlreadyAddedKF.count(pKFi)) {
        vpLoopCandidates.push_back(pKFi);
        spAlreadyAddedKF.insert(pKFi);
      }
    }
  }

  return vpLoopCandidates;
}

std::vector<KeyFrame*> KeyFrameMatchDatabase::DetectRelocalizationCandidates(
    FrameBase* pF) {
  std::vector<std::pair<KeyFrame*, size_t>> frameMatchCounts;
  frameMatchCounts.reserve(mFrames.size() / 3);

  size_t maxNumMatches = 0;
  // calculate number of feature matches for every key frame and frame
  // also search for max number of matches
  for (auto pKFi : mFrames) {
    auto matchResult = mFeatureMatcher->MatchFrames(pF, pKFi);
    auto numMatches = static_cast<int>(matchResult.GetNumMatches());
    pKFi->mnRelocQuery = pF->id();
    pKFi->mRelocScore = static_cast<float>(numMatches);
    frameMatchCounts.emplace_back(pKFi, numMatches);
    if (numMatches > maxNumMatches) maxNumMatches = numMatches;
  }
  auto minNumMatches = static_cast<size_t>(maxNumMatches * 0.8f);

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
        if (pKF2->mnRelocQuery != pF->id()) {
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
