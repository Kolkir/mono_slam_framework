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

#include "System.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

#include "Converter.h"
#include "LoopClosing.h"
#include "MapDrawer.h"

namespace SLAM_PIPELINE {

System::System(FeatureParameters &parameters, FeatureMatcher *featureMatcher,
               KeyFrameDatabase *keyFrameDatabase, FrameFactory *frameFactory,
               KeyFrameFactory *keyFrameFactory)
    : mbReset(false),
      mFeatureMatcher(featureMatcher),
      mpKeyFrameDatabase(keyFrameDatabase) {
  // Create the Map
  mpMap = new Map();

  // Create Drawers. These are used by the Viewer
  mpMapDrawer = new MapDrawer(mpMap);

  // Initialize the Tracking thread
  //(it will live in the main thread of execution, the one that called this
  // constructor)
  mpTracker =
      new Tracking(this, mpMapDrawer, mpMap, mpKeyFrameDatabase, parameters,
                   mFeatureMatcher, frameFactory, keyFrameFactory);

  // Initialize the Local Mapping thread and launch
  mpLocalMapper = new LocalMapping(mpMap, mFeatureMatcher);
  mptLocalMapping =
      new std::thread(&SLAM_PIPELINE::LocalMapping::Run, mpLocalMapper);

  // Initialize the Loop Closing thread and launch
  mpLoopCloser =
      new LoopClosing(mpMap, mpKeyFrameDatabase, mFeatureMatcher, parameters);
  mptLoopClosing =
      new std::thread(&SLAM_PIPELINE::LoopClosing::Run, mpLoopCloser);

  // Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetLoopCloser(mpLoopCloser);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

void System::TrackMonocular(const cv::Mat &im, const double &timestamp) {
  // Check reset
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset = false;
    }
  }

  cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);

  std::unique_lock<std::mutex> lock2(mMutexState);

  current_position_ = Tcw;
}

bool System::MapChanged() {
  static int n = 0;
  int curn = mpMap->GetLastBigChangeIdx();
  if (n < curn) {
    n = curn;
    return true;
  } else
    return false;
}

bool System::isRunningGBA() {
  return false;
  // mpLoopCloser->isRunningGBA();
}

void System::Reset() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  mbReset = true;
}

void System::Shutdown() {
  mpLocalMapper->RequestFinish();
  // mpLoopCloser->RequestFinish();

  // Wait until all thread have effectively stopped
  while (!mpLocalMapper->isFinished() /*|| !mpLoopCloser->isFinished() ||
         mpLoopCloser->isRunningGBA()*/) {
    std::this_thread::sleep_for(std::chrono::microseconds(5000));
  }
}

void System::SaveKeyFrameTrajectoryTUM(const std::string &filename) {
  std::cout << std::endl
            << "Saving keyframe trajectory to " << filename << " ..."
            << std::endl;

  std::vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  // cv::Mat Two = vpKFs[0]->GetPoseInverse();

  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame *pKF = vpKFs[i];

    // pKF->SetPose(pKF->GetPose()*Two);

    if (pKF->isBad()) continue;

    cv::Mat R = pKF->GetRotation().t();
    std::vector<float> q = Converter::toQuaternion(R);
    cv::Mat t = pKF->GetCameraCenter();
    f << std::setprecision(6) << pKF->mTimeStamp << std::setprecision(7) << " "
      << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " "
      << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;
  }

  f.close();
  std::cout << std::endl << "trajectory saved!" << std::endl;
}

void System::StartGUI() { mpMapDrawer->Start(); }
void System::StopGUI() { mpMapDrawer->Stop(); }

void System::SetMinimumKeyFrames(int min_num_kf) {
  mpTracker->SetMinimumKeyFrames(min_num_kf);
}

cv::Mat System::GetCurrentPosition() { return current_position_; }

std::vector<MapPoint *> System::GetAllMapPoints() {
  return mpMap->GetAllMapPoints();
}

cv::Mat System::GetIniMatchImage() const {
  return mpTracker->GetIniMatchImage();
}

void System::ToggleInitializationAllowed() {
  mpTracker->ToggleInitializationAllowed();
}

}  // namespace SLAM_PIPELINE
