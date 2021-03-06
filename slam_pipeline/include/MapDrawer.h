
#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#ifdef _MSC_VER
#pragma warning(push, 0)
#else
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wall"
#pragma clang diagnostic ignored "-Wextra"
#endif

#include <pcl/visualization/pcl_visualizer.h>

#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma clang diagnostic pop
#endif

#include <atomic>
#include <mutex>
#include <thread>

#include "slam_pipeline_export.h"

namespace SLAM_PIPELINE {
class Map;

class SLAM_PIPELINE_EXPORT MapDrawer {
 public:
  MapDrawer(Map *pMap);
  MapDrawer(const MapDrawer &) = delete;
  MapDrawer &operator=(const MapDrawer &) = delete;
  ~MapDrawer();

  // Update points from the last processed frame.
  void Update();

  void SetPosDir(float x, float y, float z, float dx, float dy, float dz);

  void Start();
  void Stop();

 private:
  void ThreadFunc();
  void CreateViewer();

 private:
  Map *mMap{nullptr};
  pcl::visualization::PCLVisualizer::Ptr mViewer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mPointCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mPointCloudToDraw;
  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> mKeyFramePositions;
  std::mutex mGuard;
  std::thread mGuiThread;
  std::atomic<bool> mIsStopped{false};
  bool mIsCloudUpdated{false};
  pcl::PointXYZ m_curPos;
  pcl::PointXYZ m_curDir;
};

}  // namespace SLAM_PIPELINE

#endif  // MAPDRAWER_H
