#include "MapDrawer.h"

#include <pcl/point_cloud.h>

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
    auto pos = mapPoint->GetWorldPos();
    mPointCloud->emplace_back(pos.at<float>(0), pos.at<float>(1),
                              pos.at<float>(2));
  }
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
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map");
  mViewer->addCoordinateSystem(1.0);
  mViewer->initCameraParameters();
}

void MapDrawer::ThreadFunc() {
  CreateViewer();
  if (mViewer) {
    while (!mIsStopped && !mViewer->wasStopped()) {
      {
        std::unique_lock<std::mutex> lock(mGuard);
        if (mIsCloudUpdated) {
          *mPointCloudToDraw = *mPointCloud;
          mViewer->removePointCloud("map");
          mViewer->addPointCloud<pcl::PointXYZ>(mPointCloudToDraw, "map");
          mIsCloudUpdated = false;
        }
      }
      mViewer->spinOnce(100);
    }
  }
}

}  // namespace SLAM_PIPELINE
