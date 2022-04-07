#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include "slam_pipeline_export.h"

#include <mutex>
#include <vector>

namespace SLAM_PIPELINE {

class FeatureMatcher;
class KeyFrame;
class Frame;

class SLAM_PIPELINE_EXPORT KeyFrameDatabase {
 public:
  virtual ~KeyFrameDatabase() {}

  virtual void add(KeyFrame* pKF) = 0;

  virtual void erase(KeyFrame* pKF) = 0;

  virtual void clear() = 0;

  // Loop Detection
  virtual std::vector<KeyFrame*> DetectLoopCandidates(KeyFrame* pKF,
                                                      float minScore) = 0;

  // Relocalization
  virtual std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F) = 0;
};

class SLAM_PIPELINE_EXPORT KeyFrameMatchDatabase : public KeyFrameDatabase {
 public:
  KeyFrameMatchDatabase(FeatureMatcher* mFeatureMatcher);
  ~KeyFrameMatchDatabase() override {}

  void add(KeyFrame* pKF) override;

  void erase(KeyFrame* pKF) override;

  void clear() override;

  // Loop Detection
  std::vector<KeyFrame*> DetectLoopCandidates(KeyFrame* pKF,
                                              float minScore) override;

  // Relocalization
  std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F) override;

 private:
  std::mutex mGuard;
  FeatureMatcher* mFeatureMatcher{nullptr};
  std::vector<KeyFrame*> mFrames;
};

}  // namespace SLAM_PIPELINE

#endif
