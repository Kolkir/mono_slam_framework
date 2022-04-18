#include "KeyPointMap.h"

#include "MapPoint.h"

namespace SLAM_PIPELINE {
KeyPointMap::KeyPointMap(int cols, int rows) : mCols(cols), mRows(rows) {
  Clear();
}

KeyPointMap::KeyPointMap(const KeyPointMap& keyPointMap)
    : mCols(keyPointMap.mCols),
      mRows(keyPointMap.mRows),
      mKeyPointMap(keyPointMap.mKeyPointMap.clone()),
      mMapPoints(keyPointMap.mMapPoints) {}

KeyPointMap& KeyPointMap::operator=(const KeyPointMap& keyPointMap) {
  mCols = keyPointMap.mCols;
  mRows = keyPointMap.mRows;
  mKeyPointMap = keyPointMap.mKeyPointMap.clone();
  mMapPoints = keyPointMap.mMapPoints;
  return *this;
}

void KeyPointMap::Clear() {
  const int dims = 2;
  int size[2] = {mCols, mRows};
  mKeyPointMap = cv::SparseMat(dims, size, CV_32S);
  mMapPoints.clear();
}

void KeyPointMap::SetMapPoint(int index, MapPoint* mapPoint) {
  SetMapPoint(KeyPointFromIndex(index), mapPoint);
}

void KeyPointMap::SetMapPoint(const cv::Point2i& keyPoint, MapPoint* mapPoint) {
  if (keyPoint.x >= 0 && keyPoint.x < mCols && keyPoint.y >= 0 &&
      keyPoint.y < mRows) {
    auto index = keyPoint.y * mCols + keyPoint.x;
    if (mapPoint) {
      // +1 is required to save the value, due to zero ones will be deleted
      mKeyPointMap.ref<int>(keyPoint.x, keyPoint.y) = index + 1;
    } else {
      mKeyPointMap.ref<int>(keyPoint.x, keyPoint.y) = 0;
    }
    if (mapPoint) {
      mMapPoints[index].mapPoint = mapPoint;
    } else {
      mMapPoints.erase(index);
    }
  }
}

MapPoint* KeyPointMap::GetMapPoint(const cv::Point2i& keyPoint, int diameter) {
  auto item = GetMapItem(keyPoint, diameter);
  if (item) {
    return item->mapPoint;
  }
  return nullptr;
}

MapPointItem* KeyPointMap::GetMapItem(const cv::Point2i& keyPoint,
                                      int diameter) {
  int index = -1;
  if (keyPoint.x >= 0 && keyPoint.x < mCols && keyPoint.y >= 0 &&
      keyPoint.y < mRows) {
    index = mKeyPointMap.value<int>(keyPoint.x, keyPoint.y) - 1;
  }
  if (index >= 0) {
    auto cx = keyPoint.x;
    auto cy = keyPoint.y;
    auto radius = diameter / 2;
    auto x = cx - radius;
    auto y = cy - radius;
    for (; y < (cy + radius); ++y) {
      for (; x < (cx + radius); ++x) {
        if (x >= 0 && x < mCols && y >= 0 && y < mRows) {
          index = mKeyPointMap.value<int>(keyPoint.x, keyPoint.y) - 1;
        }
      }
    }
  }

  if (index >= 0) {
    auto i = mMapPoints.find(index);
    if (i != mMapPoints.end()) return &mMapPoints.at(index);
  }

  return nullptr;
}

KeyPointMap::PointsMap::iterator KeyPointMap::Begin() {
  return mMapPoints.begin();
}

KeyPointMap::PointsMap::iterator KeyPointMap::End() { return mMapPoints.end(); }

cv::Point2i KeyPointMap::KeyPointFromIndex(int index) const {
  cv::Point2i key;
  key.y = index / mCols;
  key.x = index - key.y * mCols;
  return key;
}

size_t KeyPointMap::GetSize() const { return mMapPoints.size(); }

int KeyPointMap::GetCols() const { return mCols; }

int KeyPointMap::GetRows() const { return mRows; }

void KeyPointMap::Reserve(size_t size) { mMapPoints.reserve(size); }

void KeyPointMap::SetOutlier(const cv::Point2i& keyPoint, bool isOutlier) {
  auto item = GetMapItem(keyPoint, 1);
  if (item) {
    item->outlier = isOutlier;
  }
}

void KeyPointMap::SetOutlier(int index, bool isOutlier) {
  SetOutlier(KeyPointFromIndex(index), isOutlier);
}

bool KeyPointMap::IsOutlier(const cv::Point2i& keyPoint) {
  auto item = GetMapItem(keyPoint, 1);
  if (item) {
    return item->outlier;
  }
  return false;
}

bool KeyPointMap::IsOutlier(int index) {
  return IsOutlier(KeyPointFromIndex(index));
}

}  // namespace SLAM_PIPELINE