#ifndef SLAM_TYPES_H
#define SLAM_TYPES_H

#include <memory>

namespace SLAM_PIPELINE {
class MapPoint;
class Frame;
class KeyFrame;

typedef std::shared_ptr<MapPoint> MapPointPtr;
typedef std::shared_ptr<Frame> FramePtr;
typedef std::shared_ptr<KeyFrame> KeyFramePtr;

}  // namespace SLAM_PIPELINE

#endif  //  // SLAM_TYPES_H
