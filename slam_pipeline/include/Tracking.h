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

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/imgproc/types_c.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "FeatureMatcher.h"
#include "Initializer.h"
#include "SlamParameters.h"
#include "slam_pipeline_export.h"
#include "types.h"

namespace SLAM_PIPELINE {

class MapDrawer;
class Map;
class KeyFrameDatabase;
class FrameFactory;
class KeyFrameFactory;
class LocalMapping;
class LoopClosing;

class SLAM_PIPELINE_EXPORT Tracking {
 public:
  Tracking(MapDrawer* pMapDrawer, Map* pMap, KeyFrameDatabase* pKFDB,
           const SlamParameters& parameters, FeatureMatcher* featureMatcher,
           FrameFactory* frameFactory, KeyFrameFactory* keyFrameFactory);

  Tracking(const Tracking&) = delete;
  Tracking operator=(const Tracking&) = delete;

  // Preprocess the input and call Track().
  cv::Mat GrabImageMonocular(const cv::Mat& mImGray, const double& timestamp);

  void SetLocalMapper(LocalMapping* pLocalMapper);
  void SetLoopClosing(LoopClosing* pLoopClosing);
  void SetMinimumKeyFrames(int min_num_kf) { mnMinimumKeyFrames = min_num_kf; }

  void Reset();

  cv::Mat GetCurrentMatchImage() const;

  void ToggleInitializationAllowed();

 public:
  // Tracking states
  enum eTrackingState {
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    LOST = 3
  };

  eTrackingState mState;
  eTrackingState mLastProcessedState;

  // Current Frame
  std::shared_ptr<Frame> mCurrentFrame;

  // Min required feature matches number for local maping (check lost)
  int mMinLocalMatchCount{-1};

  // Initialization Variables (Monocular)
  int mMinIniMatchCount{-1};
  MatchFramesResult mIniMatchResult;
  std::vector<int> mvIniMatches;
  std::vector<cv::Point3f> mvIniP3D;
  FramePtr mInitialFrame;

  // Lists used to recover the full camera trajectory at the end of the
  // execution. Basically we store the reference keyframe for each frame and its
  // relative transformation
  std::list<cv::Mat> mlRelativeFramePoses;
  std::list<KeyFramePtr> mlpReferences;
  std::list<double> mlFrameTimes;
  std::list<bool> mlbLost;

 protected:
  // Main tracking function. It is independent of the input sensor.
  void Track();

  // Map initialization for monocular
  void MonocularInitialization();
  void CreateInitialMapMonocular();

  void CheckReplacedInLastFrame();
  bool TrackReferenceKeyFrame();
  void UpdateLastFrame();
  bool TrackWithMotionModel();

  bool Relocalization();

  void UpdateLocalKeyFrames();

  bool TrackLocalMap();
  void SearchLocalPoints();

  bool NeedNewKeyFrame();
  void CreateNewKeyFrame();

  void CreateCurrentMatchImage(const MatchFramesResult& matchResult);

  // Numer of Keyframes a map has to have to not get a reset in the event of
  // lost tracking.
  int mnMinimumKeyFrames;

  // Other Thread Pointers
  LocalMapping* mpLocalMapper;
  LoopClosing* mpLoopClosing;

  KeyFrameDatabase* mpKeyFrameDB;

  // Initalization (only for monocular)
  std::unique_ptr<Initializer> mpInitializer;

  // Local Map
  KeyFramePtr mpReferenceKF;
  std::vector<KeyFramePtr> mvpLocalKeyFrames;

  // Drawers
  MapDrawer* mpMapDrawer;

  // Map
  Map* mpMap;

  // Camera matrix
  cv::Mat mK;

  // New KeyFrame rules (according to fps)
  int mMinFrames{-1};
  int mMaxFrames{-1};

  // Current matches in frame
  int mnMatchesInliers;

  // Last Frame, KeyFrame and Relocalisation Info
  KeyFramePtr mpLastKeyFrame;
  FramePtr mLastFrame;
  unsigned int mnLastKeyFrameId;
  unsigned int mnLastRelocFrameId;

  // Motion Model
  cv::Mat mVelocity;

  FrameFactory* mFrameFactory{nullptr};
  KeyFrameFactory* mKeyFrameFactory{nullptr};
  FeatureMatcher* mFeatureMatcher;
  cv::Mat mCurrentMatchImage;
  int mImgWidth{0};
  int mImgHeight{0};
  bool mInitializationAllowed{false};
  float mMinParallax;
};

}  // namespace SLAM_PIPELINE

#endif  // TRACKING_H
