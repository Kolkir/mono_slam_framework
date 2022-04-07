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
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include <opencv2/opencv.hpp>

#include "FeatureMatcher.h"
#include "Frame.h"
#include "slam_pipeline_export.h"

namespace SLAM_PIPELINE {

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM.
class SLAM_PIPELINE_EXPORT Initializer {
  typedef std::pair<int, int> Match;

 public:
  Initializer(const cv::Mat &K, float sigma = 1.0, int iterations = 200);

  // Computes in parallel a fundamental matrix and a homography
  // Selects a model and tries to recover the motion and the structure from
  // motion
  bool Initialize(const MatchFramesResult &matchResult, cv::Mat &R21,
                  cv::Mat &t21, std::vector<cv::Point3f> &vP3D,
                  std::vector<bool> &vbTriangulated);

  bool InitializeOpenCV(const MatchFramesResult &matchResult, cv::Mat &R21,
                        cv::Mat &t21, std::vector<cv::Point3f> &vP3D,
                        std::vector<bool> &vbTriangulated);

 private:
  void FindHomography(std::vector<bool> &vbMatchesInliers, float &score,
                      cv::Mat &H21);
  void FindFundamental(std::vector<bool> &vbInliers, float &score,
                       cv::Mat &F21);

  cv::Mat ComputeH21(const std::vector<cv::Point2f> &vP1,
                     const std::vector<cv::Point2f> &vP2);
  cv::Mat ComputeF21(const std::vector<cv::Point2f> &vP1,
                     const std::vector<cv::Point2f> &vP2);

  float CheckHomography(const cv::Mat &H21, const cv::Mat &H12,
                        std::vector<bool> &vbMatchesInliers, float sigma);

  float CheckFundamental(const cv::Mat &F21,
                         std::vector<bool> &vbMatchesInliers, float sigma);

  bool ReconstructF(std::vector<bool> &vbMatchesInliers, cv::Mat &F21,
                    cv::Mat &K, cv::Mat &R21, cv::Mat &t21,
                    std::vector<cv::Point3f> &vP3D,
                    std::vector<bool> &vbTriangulated, float minParallax,
                    int minTriangulated);

  bool ReconstructH(std::vector<bool> &vbMatchesInliers, cv::Mat &H21,
                    cv::Mat &K, cv::Mat &R21, cv::Mat &t21,
                    std::vector<cv::Point3f> &vP3D,
                    std::vector<bool> &vbTriangulated, float minParallax,
                    int minTriangulated);

  void Triangulate(const cv::Point2f &kp1, const cv::Point2f &kp2,
                   const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

  void Normalize(const std::vector<cv::Point2f> &vKeys,
                 std::vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

  int CheckRT(const cv::Mat &R, const cv::Mat &t,
              const std::vector<cv::Point2f> &vKeys1,
              const std::vector<cv::Point2f> &vKeys2,
              const std::vector<Match> &vMatches12,
              std::vector<bool> &vbInliers, const cv::Mat &K,
              std::vector<cv::Point3f> &vP3D, float th2,
              std::vector<bool> &vbGood, float &parallax);

  void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

  // Keypoints from Reference Frame (Frame 1)
  std::vector<cv::Point2f> mvKeys1;

  // Keypoints from Current Frame (Frame 2)
  std::vector<cv::Point2f> mvKeys2;

  // Current Matches from Reference to Current
  std::vector<Match> mvMatches12;

  // Calibration
  cv::Mat mK;

  // Standard Deviation and Variance
  float mSigma, mSigma2;

  // Ransac max iterations
  int mMaxIterations;

  // Ransac sets
  std::vector<std::vector<size_t> > mvSets;
};

}  // namespace SLAM_PIPELINE

#endif  // INITIALIZER_H
