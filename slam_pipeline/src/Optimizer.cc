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

#include "Optimizer.h"

#ifdef _MSC_VER
#pragma warning(push, 0)
#else
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wall"
#pragma clang diagnostic ignored "-Wextra"
#endif

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <Eigen/StdVector>

#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma clang diagnostic pop
#endif

#include <mutex>

#include "Converter.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"

namespace SLAM_PIPELINE {

#ifdef _MSC_VER
#pragma optimize("", off)
#endif

void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations,
                                       bool* pbStopFlag,
                                       const unsigned long nLoopKF,
                                       const bool bRobust) {
  auto vpKFs = pMap->GetAllKeyFrames();
  auto vpMP = pMap->GetAllMapPoints();
  BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::BundleAdjustment(const std::vector<KeyFramePtr>& vpKFs,
                                 const std::vector<MapPointPtr>& vpMP,
                                 int nIterations, bool* pbStopFlag,
                                 const unsigned long nLoopKF,
                                 const bool bRobust) {
  std::vector<bool> vbNotIncludedMP;
  vbNotIncludedMP.resize(vpMP.size());

  g2o::SparseOptimizer optimizer;
  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();

  auto solver_ptr =
      std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));

  auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  long unsigned int maxKFid = 0;

  // Set KeyFrame vertices
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFramePtr pKF = vpKFs[i];
    if (pKF->isBad()) continue;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
    vSE3->setId(pKF->id());
    vSE3->setFixed(pKF->id() == 0);
    optimizer.addVertex(vSE3);
    if (pKF->id() > maxKFid) maxKFid = pKF->id();
  }

  const double thHuber2D = sqrt(5.99);

  // Set MapPoint vertices
  for (size_t i = 0; i < vpMP.size(); i++) {
    MapPointPtr pMP = vpMP[i];
    if (pMP->isBad()) continue;
    g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
    const int id = pMP->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    const auto observations = pMP->GetObservations();

    int nEdges = 0;
    // SET EDGES
    for (auto mit = observations.begin(); mit != observations.end(); mit++) {
      KeyFramePtr pKF = mit->first;
      if (pKF->isBad() || pKF->id() > maxKFid) continue;

      nEdges++;

      const cv::Point2i& kpUn = mit->second;

      g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

      auto v0 = optimizer.vertex(id);
      e->setVertex(0, v0);
      auto v1 = optimizer.vertex(pKF->id());
      e->setVertex(1, v1);

      Eigen::Matrix<double, 2, 1> obs;
      obs << kpUn.x, kpUn.y;
      e->setMeasurement(obs);

      e->setInformation(Eigen::Matrix2d::Identity());

      if (bRobust) {
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber2D);
      }

      e->setFx(pKF->fx());
      e->setFy(pKF->fy());
      e->setCx(pKF->cx());
      e->setCy(pKF->cy());

      optimizer.addEdge(e);
    }

    if (nEdges == 0) {
      optimizer.removeVertex(vPoint);
      vbNotIncludedMP[i] = true;
    } else {
      vbNotIncludedMP[i] = false;
    }
  }

  // Optimize!
  optimizer.initializeOptimization();

  // Squared error before output optimization
  // std::cout << "Initial chi2(before opt) = " << FIXED(optimizer.chi2()) <<
  // endl; optimizer.setVerbose(true);

  optimizer.optimize(nIterations);

  // Squared error before output optimization
  // std::cout << "Initial chi2(after opt) = " << FIXED(optimizer.chi2()) <<
  // endl;

  // Recover optimized data

  // Keyframes
  for (size_t i = 0; i < vpKFs.size(); i++) {
    KeyFramePtr pKF = vpKFs[i];
    if (pKF->isBad()) continue;
    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->id()));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    if (nLoopKF == 0) {
      pKF->SetPose(Converter::toCvMat(SE3quat));
    } else {
      pKF->mTcwGBA.create(4, 4, CV_32F);
      Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
      pKF->mnBAGlobalForKF = nLoopKF;
    }
  }

  // Points
  for (size_t i = 0; i < vpMP.size(); i++) {
    if (vbNotIncludedMP[i]) continue;

    MapPointPtr pMP = vpMP[i];

    if (pMP->isBad()) continue;
    g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
        optimizer.vertex(pMP->mnId + maxKFid + 1));

    if (nLoopKF == 0) {
      pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
      pMP->UpdateNormalAndDepth();
    } else {
      pMP->mPosGBA.create(3, 1, CV_32F);
      Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
      pMP->mnBAGlobalForKF = nLoopKF;
    }
  }
}

int Optimizer::PoseOptimization(FramePtr pFrame) {
  g2o::SparseOptimizer optimizer;

  auto linearSolver = std::make_unique<
      g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();

  auto solver_ptr =
      std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));

  auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);

  int nInitialCorrespondences = 0;

  // Set Frame vertex
  g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
  vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  // Set MapPoint vertices
  const auto N = pFrame->mKeyPointMap.GetSize();

  std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
  std::vector<size_t> vnIndexEdgeMono;
  vpEdgesMono.reserve(N);
  vnIndexEdgeMono.reserve(N);

  const auto deltaMono = sqrt(5.991);
  for (auto i = pFrame->mKeyPointMap.Begin(), end = pFrame->mKeyPointMap.End();
       i != end; ++i) {
    MapPointPtr pMP = i->second.mapPoint;
    if (pMP) {
      // Monocular observation
      nInitialCorrespondences++;
      i->second.outlier = false;

      Eigen::Matrix<double, 2, 1> obs;
      const cv::Point2i& kpUn =
          pFrame->mKeyPointMap.KeyPointFromIndex(i->first);
      obs << kpUn.x, kpUn.y;

      g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

      e->setVertex(
          0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
      e->setMeasurement(obs);
      e->setInformation(Eigen::Matrix2d::Identity());

      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      e->setRobustKernel(rk);
      rk->setDelta(deltaMono);

      e->setFx(pFrame->fx());
      e->setFy(pFrame->fy());
      e->setCx(pFrame->cx());
      e->setCy(pFrame->cy());
      cv::Mat Xw = pMP->GetWorldPos();
      e->setXw(g2o::Vector3(Xw.at<float>(0), Xw.at<float>(1), Xw.at<float>(2)));

      optimizer.addEdge(e);

      vpEdgesMono.push_back(e);
      vnIndexEdgeMono.push_back(i->first);
    }
  }

  if (nInitialCorrespondences < 3) return 0;

  // We perform 4 optimizations, after each optimization we classify observation
  // as inlier/outlier At the next optimization, outliers are not included, but
  // at the end they can be classified as inliers again.
  const float chi2Mono[4] = {5.991f, 5.991f, 5.991f, 5.991f};
  const int its[4] = {10, 10, 10, 10};

  int nBad = 0;
  for (size_t it = 0; it < 4; it++) {
    vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

      const auto idx = static_cast<int>(vnIndexEdgeMono[i]);

      if (pFrame->mKeyPointMap.IsOutlier(idx)) {
        e->computeError();
      }

      const auto chi2 = e->chi2();

      if (chi2 > chi2Mono[it]) {
        pFrame->mKeyPointMap.SetOutlier(idx, true);
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mKeyPointMap.SetOutlier(idx, false);
        e->setLevel(0);
      }

      if (it == 2) e->setRobustKernel(0);
    }

    if (optimizer.edges().size() < 10) break;
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexSE3Expmap* vSE3_recov =
      static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  cv::Mat pose = Converter::toCvMat(SE3quat_recov);
  pFrame->SetPose(pose);

  return nInitialCorrespondences - nBad;
}

void Optimizer::LocalBundleAdjustment(KeyFramePtr pKF, bool* pbStopFlag) {
  // Local KeyFrames: First Breath Search from Current Keyframe
  std::list<KeyFramePtr> lLocalKeyFrames;

  lLocalKeyFrames.push_back(pKF);
  pKF->mnBALocalForKF = pKF->id();

  const std::vector<KeyFramePtr> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
  for (size_t i = 0, iend = vNeighKFs.size(); i < iend; i++) {
    KeyFramePtr pKFi = vNeighKFs[i];
    pKFi->mnBALocalForKF = pKF->id();
    if (!pKFi->isBad()) lLocalKeyFrames.push_back(pKFi);
  }

  // Local MapPoints seen in Local KeyFrames
  std::list<MapPointPtr> lLocalMapPoints;
  for (std::list<KeyFramePtr>::iterator lit = lLocalKeyFrames.begin(),
                                        lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    auto vpMPs = (*lit)->GetMapPointMatches();
    for (auto vit = vpMPs.Begin(), vend = vpMPs.End(); vit != vend; vit++) {
      MapPointPtr pMP = vit->second.mapPoint;
      if (pMP)
        if (!pMP->isBad())
          if (pMP->mnBALocalForKF != pKF->id()) {
            lLocalMapPoints.push_back(pMP);
            pMP->mnBALocalForKF = pKF->id();
          }
    }
  }

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local
  // Keyframes
  std::list<KeyFramePtr> lFixedCameras;
  for (std::list<MapPointPtr>::iterator lit = lLocalMapPoints.begin(),
                                        lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    auto observations = (*lit)->GetObservations();
    for (auto mit = observations.begin(), mend = observations.end();
         mit != mend; mit++) {
      KeyFramePtr pKFi = mit->first;

      if (pKFi->mnBALocalForKF != pKF->id() &&
          pKFi->mnBAFixedForKF != pKF->id()) {
        pKFi->mnBAFixedForKF = pKF->id();
        if (!pKFi->isBad()) lFixedCameras.push_back(pKFi);
      }
    }
  }

  // Setup optimizer
  g2o::SparseOptimizer optimizer;

  auto linearSolver = std::make_unique<
      g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();

  auto solver_ptr =
      std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));

  auto solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
  optimizer.setAlgorithm(solver);

  if (pbStopFlag) optimizer.setForceStopFlag(pbStopFlag);

  unsigned long maxKFid = 0;

  // Set Local KeyFrame vertices
  for (std::list<KeyFramePtr>::iterator lit = lLocalKeyFrames.begin(),
                                        lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    KeyFramePtr pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
    vSE3->setId(pKFi->id());
    vSE3->setFixed(pKFi->id() == 0);
    optimizer.addVertex(vSE3);
    if (pKFi->id() > maxKFid) maxKFid = pKFi->id();
  }

  // Set Fixed KeyFrame vertices
  for (std::list<KeyFramePtr>::iterator lit = lFixedCameras.begin(),
                                        lend = lFixedCameras.end();
       lit != lend; lit++) {
    KeyFramePtr pKFi = *lit;
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
    vSE3->setId(pKFi->id());
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (pKFi->id() > maxKFid) maxKFid = pKFi->id();
  }

  // Set MapPoint vertices
  const auto nExpectedSize =
      (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

  std::vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  std::vector<KeyFramePtr> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  std::vector<MapPointPtr> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  const auto thHuberMono = sqrt(5.991);

  for (std::list<MapPointPtr>::iterator lit = lLocalMapPoints.begin(),
                                        lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPointPtr pMP = *lit;
    g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
    vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
    int id = pMP->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    auto observations = pMP->GetObservations();

    // Set edges
    for (auto mit = observations.begin(), mend = observations.end();
         mit != mend; mit++) {
      KeyFramePtr pKFi = mit->first;

      if (!pKFi->isBad()) {
        const cv::Point2i& kpUn = mit->second;

        // Monocular observation
        Eigen::Matrix<double, 2, 1> obs;
        obs << kpUn.x, kpUn.y;

        g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                            optimizer.vertex(pKFi->id())));
        e->setMeasurement(obs);
        e->setInformation(Eigen::Matrix2d::Identity());

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuberMono);

        e->setFx(pKFi->fx());
        e->setFy(pKFi->fy());
        e->setCx(pKFi->cx());
        e->setCy(pKFi->cy());

        optimizer.addEdge(e);
        vpEdgesMono.push_back(e);
        vpEdgeKFMono.push_back(pKFi);
        vpMapPointEdgeMono.push_back(pMP);
      }
    }
  }

  if (pbStopFlag)
    if (*pbStopFlag) return;

  optimizer.initializeOptimization();
  optimizer.optimize(5);

  bool bDoMore = true;

  if (pbStopFlag)
    if (*pbStopFlag) bDoMore = false;

  if (bDoMore) {
    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
      g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
      MapPointPtr pMP = vpMapPointEdgeMono[i];

      if (pMP->isBad()) continue;

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        e->setLevel(1);
      }

      e->setRobustKernel(0);
    }

    // Optimize again without the outliers

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
  }

  std::vector<std::pair<KeyFramePtr, MapPointPtr>> vToErase;
  vToErase.reserve(vpEdgesMono.size());

  // Check inlier observations
  for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
    g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
    MapPointPtr pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) continue;

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFramePtr pKFi = vpEdgeKFMono[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  if (!vToErase.empty()) {
    for (size_t i = 0; i < vToErase.size(); i++) {
      KeyFramePtr pKFi = vToErase[i].first;
      MapPointPtr pMPi = vToErase[i].second;
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // Recover optimized data

  // Keyframes
  for (std::list<KeyFramePtr>::iterator lit = lLocalKeyFrames.begin(),
                                        lend = lLocalKeyFrames.end();
       lit != lend; lit++) {
    KeyFramePtr kf = *lit;
    g2o::VertexSE3Expmap* vSE3 =
        static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(kf->id()));
    g2o::SE3Quat SE3quat = vSE3->estimate();
    kf->SetPose(Converter::toCvMat(SE3quat));
  }

  // Points
  for (std::list<MapPointPtr>::iterator lit = lLocalMapPoints.begin(),
                                        lend = lLocalMapPoints.end();
       lit != lend; lit++) {
    MapPointPtr pMP = *lit;
    g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(
        optimizer.vertex(pMP->mnId + maxKFid + 1));
    pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
    pMP->UpdateNormalAndDepth();
  }
}

#ifdef _MSC_VER
#pragma optimize("", on)
#endif

}  // namespace SLAM_PIPELINE
