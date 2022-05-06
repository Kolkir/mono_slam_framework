/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza) For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <opencv2/core/core.hpp>
#include <string>
#include <memory>

#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "Map.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "slam_pipeline_export.h"

namespace SLAM_PIPELINE {
class LoopClosing;

class SLAM_PIPELINE_EXPORT System {
 public:
  System(FeatureParameters &parameters, FeatureMatcher *featureMatcher,
         KeyFrameDatabase *keyFrameDatabase, FrameFactory *frameFactory,
         KeyFrameFactory *keyFrameFactory);

  // Process the given monocular frame
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  void TrackMonocular(const cv::Mat &im, const double &timestamp);

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();

  // Reset the system (clear map)
  void Reset();

  void StartGUI();
  void StopGUI();

  // Save keyframe poses in the TUM RGB-D dataset format.
  // This method works for all sensor input.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveKeyFrameTrajectoryTUM(const std::string &filename);

  void SetMinimumKeyFrames(int min_num_kf);

  cv::Mat GetCurrentPosition();

  std::vector<MapPoint *> GetAllMapPoints();

  cv::Mat GetCurrentMatchImage() const;

  void ToggleInitializationAllowed();

 private:
  FeatureMatcher *mFeatureMatcher;
  // KeyFrame database for place recognition (relocalization and loop
  // detection).
  KeyFrameDatabase *mpKeyFrameDatabase;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  std::unique_ptr<Map> mpMap;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints
  // and performs relocalization if tracking fails.
  std::unique_ptr<Tracking> mpTracker;

  // Local Mapper. It manages the local map and performs local bundle
  // adjustment.
  std::unique_ptr<LocalMapping> mpLocalMapper;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop
  // it performs a pose graph optimization and full bundle adjustment (in a new
  // thread) afterwards.
  std::shared_ptr<LoopClosing> mpLoopCloser;

  std::unique_ptr<MapDrawer> mpMapDrawer;

  // Current position
  cv::Mat current_position_;
};

}  // namespace SLAM_PIPELINE

#endif  // SYSTEM_H
