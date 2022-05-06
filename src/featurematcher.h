#pragma once

#include <opencv2/features2d.hpp>

#include "slam_pipeline/include/FeatureMatcher.h"

class FeatureMatcher : public SLAM_PIPELINE::FeatureMatcher {
 public:
  FeatureMatcher(float threshold = 0.8f);
  FeatureMatcher(const FeatureMatcher&) = delete;
  FeatureMatcher& operator=(const FeatureMatcher&) = delete;
  ~FeatureMatcher() override;

  SLAM_PIPELINE::MatchFramesResult MatchFrames(
      SLAM_PIPELINE::FrameBase* pF1, SLAM_PIPELINE::FrameBase* pF2) override;

  void SetThreshold(float value);

 private:
  cv::Ptr<cv::Feature2D> feature_extractor_;
  cv::Ptr<cv::DescriptorMatcher> feature_matcher_;
  float threshold_{0};
};