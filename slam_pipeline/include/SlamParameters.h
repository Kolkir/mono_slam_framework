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
  float nCovisibilityConsistencyTh{3};

  // If the map contains less than num KF or less than num KF have passed from
  // last loop detection then populate key frame database
  int loopDetectionMaxFrames{5};

  double minSim3ReprojectionError = 20.0f; // original is 9.210f
  int minSim3Matches{6};        // map points matches for key points
  int minTotalSim3Matches{12};  // key points matches

  // The minimum possible parallax value to perform triangulation:
  // how much map points will be created
  double minimumParallax = 1.1;
};

}  // namespace SLAM_PIPELINE
