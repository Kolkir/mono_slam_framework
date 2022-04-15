#include "MapDrawer.h"

#ifdef _MSC_VER
#pragma warning(push, 0)
#else
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wall"
#pragma clang diagnostic ignored "-Wextra"
#endif

#include <pcl/point_cloud.h>

#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma clang diagnostic pop
#endif

#include "Map.h"

namespace SLAM_PIPELINE {

MapDrawer::MapDrawer(Map* pMap)
    : mMap(pMap),
      mPointCloud(new pcl::PointCloud<pcl::PointXYZ>),
      mPointCloudToDraw(new pcl::PointCloud<pcl::PointXYZ>) {}

void MapDrawer::Update() {
  std::unique_lock<std::mutex> lock(mGuard);
  mIsCloudUpdated = true;
  mPointCloud->clear();
  auto mapPoints = mMap->GetAllMapPoints();
  for (auto& mapPoint : mapPoints) {
    if (!mapPoint->isBad()) {
      auto pos = mapPoint->GetWorldPos();
      mPointCloud->emplace_back(pos.at<float>(0), pos.at<float>(1),
                                pos.at<float>(2));
    }
  }

  mKeyFramePositions.clear();
  auto frames = mMap->GetAllKeyFrames();
  for (auto& kf : frames) {
    auto pos = kf->GetTranslation();
    mKeyFramePositions.emplace_back(pos.at<float>(0), pos.at<float>(1),
                                    pos.at<float>(2));
  }
}

void MapDrawer::SetPos(float x, float y, float z) {
  std::unique_lock<std::mutex> lock(mGuard);
  mIsCloudUpdated = true;
  m_curX = x;
  m_curY = y;
  m_curZ = z;
}

void MapDrawer::Start() {
  mGuiThread = std::thread(&MapDrawer::ThreadFunc, this);
}

void MapDrawer::Stop() { mIsStopped = true; }

void MapDrawer::CreateViewer() {
  mViewer.reset(new pcl::visualization::PCLVisualizer("Map Viewer"));
  mViewer->setBackgroundColor(0, 0, 0);
  mViewer->addPointCloud<pcl::PointXYZ>(mPointCloudToDraw, "map");
  mViewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map");
  mViewer->addCoordinateSystem(1.0);
  mViewer->initCameraParameters();
  mViewer->getRenderWindow()->GlobalWarningDisplayOff();
}

void MapDrawer::ThreadFunc() {
  CreateViewer();
  if (mViewer) {
    while (!mIsStopped && !mViewer->wasStopped()) {
      {
        std::unique_lock<std::mutex> lock(mGuard);
        if (mIsCloudUpdated) {
          *mPointCloudToDraw = *mPointCloud;
          mViewer->removeAllShapes();
          mViewer->removePointCloud("map");
          mViewer->addPointCloud<pcl::PointXYZ>(mPointCloudToDraw, "map");
          mViewer->setPointCloudRenderingProperties(
              pcl::visualization::PCL_VISUALIZER_POINT_SIZE, /*size*/ 3, "map");
          mViewer->addSphere(pcl::PointXYZ(m_curX, m_curY, m_curZ),
                             /*radius*/ 0.02, "pos_sphere");
          int i = 0;
          for (auto& pos : mKeyFramePositions) {
            std::stringstream name;
            name << "kf" << i;
            mViewer->addSphere(pos, /*radius*/ 0.01, 1.0, 0.0, 0.0, name.str());
            ++i;
          }
          mIsCloudUpdated = false;
        }
      }
      mViewer->spinOnce(100);
    }
  }
}

}  // namespace SLAM_PIPELINE
