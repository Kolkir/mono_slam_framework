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

#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/core/core_c.h>

#include <opencv2/opencv.hpp>
#include <vector>

#include "slam_pipeline_export.h"
#include "types.h"

namespace SLAM_PIPELINE {

struct MatchFramesResult;

class SLAM_PIPELINE_EXPORT Sim3Solver {
 public:
  Sim3Solver(const MatchFramesResult &matchResult,
             double minSim3ReprojectionError);

  void SetRansacParameters(double probability = 0.99, int minInliers = 6,
                           int maxIterations = 300);

  cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

  cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers,
                  int &nInliers);

  cv::Mat GetEstimatedRotation();
  cv::Mat GetEstimatedTranslation();
  float GetEstimatedScale();

 protected:
  void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

  void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

  void CheckInliers();

  void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D,
               cv::Mat Tcw, cv::Mat K);
  void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc,
                         std::vector<cv::Mat> &vP2D, cv::Mat K);

 protected:
  // KeyFrames and matches
  std::vector<cv::Mat> mvX3Dc1;
  std::vector<cv::Mat> mvX3Dc2;

  int mNumMapPointMatches;
  int mNumKeyPointMatches;

  // Current Estimation
  cv::Mat mR12i;
  cv::Mat mt12i;
  float ms12i;
  cv::Mat mT12i;
  cv::Mat mT21i;
  std::vector<bool> mvbInliersi;
  int mnInliersi;

  // Current Ransac State
  int mnIterations;
  std::vector<bool> mvbBestInliers;
  int mnBestInliers;
  cv::Mat mBestT12;
  cv::Mat mBestRotation;
  cv::Mat mBestTranslation;
  float mBestScale;

  // Indices for random selection
  std::vector<size_t> mvAllIndices;

  // Projections
  std::vector<cv::Mat> mvP1im1;
  std::vector<cv::Mat> mvP2im2;

  // RANSAC probability
  double mRansacProb;

  double mMinSim3ReprojectionError;

  // RANSAC min inliers
  int mRansacMinInliers;

  // RANSAC max iterations
  int mRansacMaxIts;

  // Calibration
  cv::Mat mK1;
  cv::Mat mK2;
};

}  // namespace SLAM_PIPELINE

#endif  // SIM3SOLVER_H
