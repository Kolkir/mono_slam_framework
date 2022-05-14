#pragma once

#include "slam_pipeline_export.h"

namespace SLAM_PIPELINE {

struct SLAM_PIPELINE_EXPORT SlamParameters {
  // camera parameters
  float fx{0}, fy{0}, cx{0}, cy{0};

  //  Max/Min Frames to insert keyframes and to check relocalization
  int maxFrames{10};
  int minFrames{0};

  // Min required feature matches number for initialization
  int minIniMatchCount{25};

  // Min required feature matches number for local mapping (check lost)
  int minLocalMatchCount{15};

  // Min required key frames to perform relocalization
  int minimumKeyFrames{5};

  // Loop closing parameters

  // If the map contains less than num KF or less than num KF have passed from
  // last loop detection then populate key frame database
  int loopDetectionMaxFrames{5};

  int minNumMPMatches{15};  // mininum number of MP matches in the not connected frames

  // The minimum possible parallax value to perform triangulation:
  // how much map points will be created
  double minimumParallax = 1.1;
};

}  // namespace SLAM_PIPELINE
