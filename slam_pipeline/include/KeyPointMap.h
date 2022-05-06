#ifndef KEYPOINTMAP_H
#define KEYPOINTMAP_H

#include <opencv2/core/core.hpp>
#include <unordered_map>

#include "slam_pipeline_export.h"

namespace SLAM_PIPELINE {
class MapPoint;

struct SLAM_PIPELINE_EXPORT MapPointItem {
  MapPoint* mapPoint{nullptr};
  bool outlier{false};
};

// Class that stores correspondenses between 2d coordinates and 3d map points
class SLAM_PIPELINE_EXPORT KeyPointMap {
 public:
  typedef std::unordered_map<int, MapPointItem> PointsMap;

  KeyPointMap() = default;
  KeyPointMap(int cols, int rows);
  KeyPointMap(const KeyPointMap& keyPointMap);
  KeyPointMap& operator=(const KeyPointMap& keyPointMap);

  // If mapPoint is nullptr it will erase the existing one
  void SetMapPoint(const cv::Point2i& keyPoint, MapPoint* mapPoint);
  void SetMapPoint(int index, MapPoint* mapPoint);
  void SetOutlier(const cv::Point2i& keyPoint, bool isOutlier);
  void SetOutlier(int index, bool isOutlier);
  bool IsOutlier(const cv::Point2i& keyPoint);
  bool IsOutlier(int index);

  MapPoint* GetMapPoint(const cv::Point2i& keyPoint, int diameter = 5);

  cv::Point2i KeyPointFromIndex(int index) const;

  size_t GetSize() const;
  int GetCols() const;
  int GetRows() const;

  void Reserve(size_t size);
  void Clear();

  PointsMap::iterator Begin();
  PointsMap::iterator End();

 private:
  MapPointItem* GetMapItem(const cv::Point2i& keyPoint, int diameter);

 private:
  int mCols{0};
  int mRows{0};

  // Mask of 2d coordinates that corresponds to key points,
  // value = y * rows + x;
  cv::SparseMat mKeyPointMap;

  // MapPoints associated to 2d coordinates: key = y * rows + x.
  PointsMap mMapPoints;
};

}  // namespace SLAM_PIPELINE

#endif  // KEYPOINTMAP_H
