#include "dnnfeaturematcher.h"

#include <iostream>

static cv::Mat ConvertImageToFloat(const cv::Mat& image) {
  cv::Mat image_float;
  image.convertTo(image_float, CV_32F, 1.0 / 255.0);
  return image_float;
}

DNNFeatureMatcher::DNNFeatureMatcher(const std::wstring& model_file_path,
                                     float threshold, int64_t image_width,
                                     int64_t image_height, int model_resolution)
    : image_width_(image_width),
      image_height_(image_height),
      model_resolution_(model_resolution),
      threshold_(threshold),
      env_(ORT_LOGGING_LEVEL_WARNING),
      session_(env_, model_file_path.c_str(), Ort::SessionOptions{nullptr}),
      memory_info_(
          Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU)) {
  Ort::AllocatorWithDefaultOptions ort_alloc;
  for (size_t i = 0; i < session_.GetInputCount(); ++i) {
    char* name = session_.GetInputName(i, ort_alloc);
    input_node_names_str_.push_back(std::string(name));
    ort_alloc.Free(name);
  }
  for (size_t i = 0; i < session_.GetOutputCount(); ++i) {
    char* name = session_.GetOutputName(i, ort_alloc);
    output_node_names_str_.push_back(std::string(name));
    ort_alloc.Free(name);
  }

  for (size_t i = 0; i < session_.GetInputCount(); ++i) {
    input_node_names_.push_back(input_node_names_str_[i].data());
  }
  for (size_t i = 0; i < session_.GetOutputCount(); ++i) {
    output_node_names_.push_back(output_node_names_str_[i].data());
  }
}

DNNFeatureMatcher::~DNNFeatureMatcher() {}

SLAM_PIPELINE::MatchFramesResult DNNFeatureMatcher::MatchFrames(
    SLAM_PIPELINE::FrameBase* pF1, SLAM_PIPELINE::FrameBase* pF2) {
  std::unique_lock<std::mutex> lock(guard_);
  auto first_input_image = ConvertImageToFloat(pF1->imGray);
  auto second_input_image = ConvertImageToFloat(pF2->imGray);
  size_t input_size = first_input_image.total() * first_input_image.elemSize();

  std::vector<Ort::Value> input;
  input.emplace_back(Ort::Value::CreateTensor<float>(
      memory_info_, reinterpret_cast<float*>(first_input_image.data),
      input_size, input_shape_.data(), input_shape_.size()));
  assert(input[0].IsTensor());

  input.emplace_back(Ort::Value::CreateTensor<float>(
      memory_info_, reinterpret_cast<float*>(second_input_image.data),
      input_size, input_shape_.data(), input_shape_.size()));
  assert(input[1].IsTensor());

  cv::Mat confidence_map;
  {
    // std::unique_lock<std::mutex> lock(guard_);
    auto output_tensors = session_.Run(
        Ort::RunOptions{nullptr}, input_node_names_.data(), input.data(),
        input.size(), output_node_names_.data(), output_node_names_.size());

    auto output_tensor_info =
        output_tensors[0].GetTypeInfo().GetTensorTypeAndShapeInfo();
    auto output_shape = output_tensor_info.GetShape();
    assert(output_shape.size() == 3);

    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    confidence_map = cv::Mat(cv::Size(static_cast<int>(output_shape[2]),
                                      static_cast<int>(output_shape[1])),
                             CV_32F, output_data);
    confidence_map = confidence_map > threshold_;
    confidence_map = confidence_map.clone();  // copy data from onnx runtime
  }
  auto model_width = image_width_ / model_resolution_;
  cv::Mat feature_coordinates;
  cv::findNonZero(confidence_map, feature_coordinates);

  SLAM_PIPELINE::MatchFramesResult matchResult;
  matchResult.pF1 = pF1;
  matchResult.keyPoints1.reserve(100);
  matchResult.pF2 = pF2;
  matchResult.keyPoints2.reserve(100);

  for (size_t k = 0; k < feature_coordinates.total(); k++) {
    auto feature_point = feature_coordinates.at<cv::Point>(static_cast<int>(k));
    auto x0 =
        static_cast<int>(feature_point.y % model_width) * model_resolution_;
    auto y0 =
        static_cast<int>(feature_point.y / model_width) * model_resolution_;
    auto x1 =
        static_cast<int>(feature_point.x % model_width) * model_resolution_;
    auto y1 =
        static_cast<int>(feature_point.x / model_width) * model_resolution_;
    matchResult.keyPoints1.emplace_back(cv::Point2i{x0, y0});
    matchResult.keyPoints2.emplace_back(cv::Point2i{x1, y1});
  }
  return matchResult;
}

void DNNFeatureMatcher::SetThreshold(float value) { threshold_ = value; }