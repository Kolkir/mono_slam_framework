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
    KeyFrame* pKF, size_t minNumMatches) {
  std::set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
  std::list<KeyFrame*> lKFsSharingFeatures;

  // Search all keyframes that share features with current keyframes
  // Discard keyframes connected to the query keyframe
  for (auto pKFi : mFrames) {
    auto matchResult = mFeatureMatcher->MatchFrames(pKF, pKFi);
    auto numMatches = static_cast<int>(matchResult.GetNumMatches());
    if (numMatches != 0 && pKFi->mnLoopQuery != pKF->id()) {
      if (!spConnectedKeyFrames.count(pKFi)) {
        pKFi->mnLoopQuery = pKF->id();
        pKFi->mLoopScore = static_cast<float>(numMatches);
        lKFsSharingFeatures.push_back(pKFi);
      }
    }
  }

  if (lKFsSharingFeatures.empty()) return std::vector<KeyFrame*>();

  std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

  float maxCommonFeatures = 0;
  for (auto kf : lKFsSharingFeatures) {
    if (kf->mLoopScore > maxCommonFeatures) maxCommonFeatures = kf->mLoopScore;
  }

  auto minCommonFeatures = maxCommonFeatures * 0.8f;

  // Retain the matches whose feature matches num is higher than  minNumMatches
  for (auto pKFi : lKFsSharingFeatures) {
    if (pKFi->mLoopScore > minNumMatches) {
      lScoreAndMatch.push_back(std::make_pair(pKFi->mLoopScore, pKFi));
    }
  }

  if (lScoreAndMatch.empty()) return std::vector<KeyFrame*>();

  std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
  float bestAccScore = static_cast<float>(minNumMatches);

  // Lets now accumulate score by covisibility
  for (std::list<std::pair<float, KeyFrame*>>::iterator
           it = lScoreAndMatch.begin(),
           itend = lScoreAndMatch.end();
       it != itend; it++) {
    KeyFrame* pKFi = it->second;
    std::vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

    float bestScore = it->first;
    float accScore = it->first;
    KeyFrame* pBestKF = pKFi;
    for (std::vector<KeyFrame*>::iterator vit = vpNeighs.begin(),
                                          vend = vpNeighs.end();
         vit != vend; vit++) {
      KeyFrame* pKF2 = *vit;
      if (pKF2->mnLoopQuery == pKF->id() &&
          pKF2->mLoopScore > minCommonFeatures) {
        accScore += pKF2->mLoopScore;
        if (pKF2->mLoopScore > bestScore) {
          pBestKF = pKF2;
          bestScore = pKF2->mLoopScore;
        }
      }
    }

    lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
    if (accScore > bestAccScore) bestAccScore = accScore;
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  float minScoreToRetain = 0.75f * bestAccScore;

  std::set<KeyFrame*> spAlreadyAddedKF;
  std::vector<KeyFrame*> vpLoopCandidates;
  vpLoopCandidates.reserve(lAccScoreAndMatch.size());

  for (std::list<std::pair<float, KeyFrame*>>::iterator
           it = lAccScoreAndMatch.begin(),
           itend = lAccScoreAndMatch.end();
       it != itend; it++) {
    if (it->first > minScoreToRetain) {
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
  {
    std::unique_lock<std::mutex> lock(mGuard);
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
