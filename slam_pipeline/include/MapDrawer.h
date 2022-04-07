
#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include <pcl/visualization/pcl_visualizer.h>
#include "slam_pipeline_export.h"

#include <mutex>
#include <thread>
#include <atomic>

namespace SLAM_PIPELINE {
class Map;

class SLAM_PIPELINE_EXPORT MapDrawer {
 public:
  MapDrawer(Map *pMap);
  MapDrawer(const MapDrawer &) = delete;
  MapDrawer &operator=(const MapDrawer &) = delete;

  // Update points from the last processed frame.
  void Update();

  void Start();
  void Stop();

 private:
  void ThreadFunc();
  void CreateViewer();
 private:
  Map *mMap{nullptr};
  pcl::visualization::PCLVisualizer::Ptr mViewer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mPointCloud;
  std::mutex mGuard;
  std::thread mGuiThread;
  std::atomic<bool> mIsStopped{false};
  bool mIsCloudUpdated{false};
};

}  // namespace SLAM_PIPELINE

#endif  // MAPDRAWER_H
