#pragma once

#include <onnxruntime_cxx_api.h>

#include <opencv2/core/core.hpp>

#include "slam_pipeline/include/FeatureMatcher.h"

class DNNFeatureMatcher : public SLAM_PIPELINE::FeatureMatcher {
 public:
  DNNFeatureMatcher(const std::wstring& model_file_path,
                    float threshold = 0.15f, int64_t image_width = 640,
                    int64_t image_height = 480, int model_resolution = 16);
  DNNFeatureMatcher(const DNNFeatureMatcher&) = delete;
  DNNFeatureMatcher& operator=(const DNNFeatureMatcher&) = delete;
  ~DNNFeatureMatcher() override;

  SLAM_PIPELINE::MatchFramesResult MatchFrames(
      SLAM_PIPELINE::FrameBase* pF1, SLAM_PIPELINE::FrameBase* pF2) override;

  void SetThreshold(float value);

 private:
  int64_t image_width_{640};
  int64_t image_height_{480};
  int model_resolution_{16};
  float threshold_ = 0.1f;
  Ort::Env env_;
  Ort::Session session_;
  Ort::MemoryInfo memory_info_;
  std::array<int64_t, 4> input_shape_{1, 1, image_height_, image_width_};

  std::vector<std::string> input_node_names_str_;
  std::vector<std::string> output_node_names_str_;
  std::vector<const char*> input_node_names_;
  std::vector<const char*> output_node_names_;
};