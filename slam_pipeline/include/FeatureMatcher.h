#ifndef FEATUREMATCHER_H
#define FEATUREMATCHER_H

#include "slam_pipeline_export.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

namespace SLAM_PIPELINE {

struct SLAM_PIPELINE_EXPORT MatchFramesResult {
  FrameBase *pF1{nullptr};
  FrameBase *pF2{nullptr};
  std::vector<cv::Point2i> keyPoints1;
  std::vector<cv::Point2i> keyPoints2;

  size_t GetNumMatches() const { return keyPoints1.size(); }

  MapPoint *GetMapPoint1(size_t idx) const {
    return pF1->mKeyPointMap.GetMapPoint(keyPoints1[idx]);
  }

  MapPoint *GetMapPoint2(size_t idx) const {
    return pF2->mKeyPointMap.GetMapPoint(keyPoints2[idx]);
  }

  void DeleteMatch(size_t idx) {
    if (idx >= 0 && idx < GetNumMatches()) {
      auto i1 = keyPoints1.begin() + idx;
      keyPoints1.erase(i1);
      auto i2 = keyPoints2.begin() + idx;
      keyPoints1.erase(i2);
    }
  }
};

class SLAM_PIPELINE_EXPORT FeatureMatcher {
 public:
  virtual ~FeatureMatcher() {}

  // Searches key point matches in two frame images
  virtual MatchFramesResult MatchFrames(FrameBase *pF1, FrameBase *pF2) = 0;
};

}  // namespace SLAM_PIPELINE

#endif  // FEATUREMATCHER_H
