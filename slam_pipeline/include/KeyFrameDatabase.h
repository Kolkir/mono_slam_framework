#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>

#include "slam_pipeline_export.h"

namespace SLAM_PIPELINE {

class FeatureMatcher;
class KeyFrame;
class FrameBase;

class SLAM_PIPELINE_EXPORT KeyFrameDatabase {
 public:
  virtual ~KeyFrameDatabase() {}

  virtual void add(KeyFrame* pKF) = 0;

  virtual void erase(KeyFrame* pKF) = 0;

  virtual void clear() = 0;

  virtual std::vector<KeyFrame*> DetectLoopCandidates(
      KeyFrame* pKF, size_t minNumCovisibleMatches) = 0;

  virtual std::vector<KeyFrame*> DetectRelocalizationCandidates(
      FrameBase* pF) = 0;
};

class SLAM_PIPELINE_EXPORT KeyFrameMatchDatabase : public KeyFrameDatabase {
 public:
  KeyFrameMatchDatabase(FeatureMatcher* mFeatureMatcher);
  ~KeyFrameMatchDatabase() override {}

  void add(KeyFrame* pKF) override;

  void erase(KeyFrame* pKF) override;

  void clear() override;

  std::vector<KeyFrame*> DetectLoopCandidates(
      KeyFrame* pKF, size_t minNumCovisibleMatches) override;

  std::vector<KeyFrame*> DetectRelocalizationCandidates(FrameBase* pF) override;

 private:
  FeatureMatcher* mFeatureMatcher{nullptr};
  std::vector<KeyFrame*> mFrames;
};

}  // namespace SLAM_PIPELINE

#endif
