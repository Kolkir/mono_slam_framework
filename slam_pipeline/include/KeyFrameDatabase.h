#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>

#include "slam_pipeline_export.h"
#include "types.h"

namespace SLAM_PIPELINE {

class FeatureMatcher;
class FrameBase;

class SLAM_PIPELINE_EXPORT KeyFrameDatabase {
 public:
  virtual ~KeyFrameDatabase() {}

  virtual void add(KeyFramePtr pKF) = 0;

  virtual void erase(KeyFramePtr pKF) = 0;

  virtual void clear() = 0;

  virtual std::vector<KeyFramePtr> DetectLoopCandidates(
      KeyFrame& pKF, size_t minNumCovisibleMatches) = 0;

  virtual std::vector<KeyFramePtr> DetectRelocalizationCandidates(
      FrameBase& pF) = 0;
};

class SLAM_PIPELINE_EXPORT KeyFrameMatchDatabase : public KeyFrameDatabase {
 public:
  KeyFrameMatchDatabase(FeatureMatcher* mFeatureMatcher);
  ~KeyFrameMatchDatabase() override {}

  void add(KeyFramePtr pKF) override;

  void erase(KeyFramePtr pKF) override;

  void clear() override;

  std::vector<KeyFramePtr> DetectLoopCandidates(
      KeyFrame& pKF, size_t minNumCovisibleMatches) override;

  std::vector<KeyFramePtr> DetectRelocalizationCandidates(
      FrameBase& pF) override;

 private:
  FeatureMatcher* mFeatureMatcher{nullptr};
  std::vector<KeyFramePtr> mFrames;
};

}  // namespace SLAM_PIPELINE

#endif
