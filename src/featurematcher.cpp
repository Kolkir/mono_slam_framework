#include "featurematcher.h"

FeatureMatcher::FeatureMatcher(float threshold) : threshold_(threshold) {
  feature_extractor_ = cv::ORB::create();
  feature_matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

FeatureMatcher::~FeatureMatcher() {}

SLAM_PIPELINE::MatchFramesResult FeatureMatcher::MatchFrames(
    SLAM_PIPELINE::FrameBase* pF1, SLAM_PIPELINE::FrameBase* pF2) {
  std::unique_lock<std::mutex> lock(guard_);

  cv::Mat mask(255 * cv::Mat::ones(pF1->imGray.size(), CV_8U));
  cv::Mat first_desc, second_desc;
  std::vector<cv::KeyPoint> first_kp, second_kp;
  feature_extractor_->detectAndCompute(pF1->imGray, mask, first_kp, first_desc);
  feature_extractor_->detectAndCompute(pF2->imGray, mask, second_kp,
                                       second_desc);

  feature_matcher_->clear();
  std::vector<std::vector<cv::DMatch> > matches;
  feature_matcher_->knnMatch(first_desc, second_desc, matches, 2);

  SLAM_PIPELINE::MatchFramesResult matchResult;
  matchResult.pF1 = pF1;
  matchResult.keyPoints1.reserve(100);
  matchResult.pF2 = pF2;
  matchResult.keyPoints2.reserve(100);
  for (unsigned i = 0; i < matches.size(); i++) {
    if (matches[i][0].distance < threshold_ * matches[i][1].distance) {
      cv::Point2i pt1(static_cast<int>(first_kp[matches[i][0].queryIdx].pt.x),
                      static_cast<int>(first_kp[matches[i][0].queryIdx].pt.y));
      cv::Point2i pt2(static_cast<int>(second_kp[matches[i][0].trainIdx].pt.x),
                      static_cast<int>(second_kp[matches[i][0].trainIdx].pt.y));
      matchResult.keyPoints1.emplace_back(pt1);
      matchResult.keyPoints2.emplace_back(pt2);
    }
  }

  return matchResult;
}

void FeatureMatcher::SetThreshold(float value) { threshold_ = value; }