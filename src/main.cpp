#include <chrono>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include "dnnfeaturematcher.h"
#include "featurematcher.h"
#include "slam_pipeline/include/System.h"

using namespace webots;
int main(int /*argc*/, char** /*argv*/) {
  namespace slam = SLAM_PIPELINE;

  /*for (;;) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }*/

  int width = 640;
  int height = 480;
  // Instantite objects
  auto robot = std::make_shared<Robot>();
  auto keyboard = std::make_shared<Keyboard>();

  // Configure camera
  auto* camera = robot->getCamera("camera_top");
  int samplingPeriod = 32;
  camera->enable(samplingPeriod);

  // Configure SLAM objects
  // DNNFeatureMatcher featureMatcher(L"model/LoFTR_teacher.onnx");
  // featureMatcher.SetThreshold(0.1);

  FeatureMatcher featureMatcher;
  featureMatcher.SetThreshold(0.8);

  slam::FeatureParameters slam_parameters;
  slam_parameters.cx = camera->getWidth() / 2;
  slam_parameters.cy = camera->getHeight() / 2;
  auto hFov = camera->getFov();
  slam_parameters.fx = slam_parameters.cx / tan(hFov / 2);
  auto vFov =
      2 * atan(tan(hFov * 0.5) *
               (static_cast<double>(camera->getHeight()) / camera->getWidth()));
  slam_parameters.fy = slam_parameters.cy / tan(vFov / 2);
  slam_parameters.maxFrames = 10;
  slam::KeyFrameMatchDatabase keyFrameDatabase(&featureMatcher);
  slam::FrameFactory frameFactory;
  slam::KeyFrameFactory keyFrameFactory;
  slam::System slam_system(slam_parameters, &featureMatcher, &keyFrameDatabase,
                           &frameFactory, &keyFrameFactory);
  slam_system.StartGUI();

  // Configure display
  auto* display_features = robot->getDisplay("display_features");

  // configure motors
  std::vector<Motor*> wheels;
  std::vector<std::string> wheels_names = {"wheel1", "wheel2", "wheel3",
                                           "wheel4"};
  for (auto& wheel_name : wheels_names) {
    auto* motor = robot->getMotor(wheel_name);
    wheels.push_back(motor);
    motor->setPosition(std::numeric_limits<double>::infinity());
    motor->setVelocity(0.0);
  }

  int time_step = 64;  //(int)robot->getBasicTimeStep();

  keyboard->enable(time_step);

  cv::Mat img(cv::Size(width, height), CV_8UC4);
  cv::Mat out_img(cv::Size(width * 2, height), CV_8UC3);
  cv::Mat img_gray;
  size_t img_size = img.total() * img.elemSize();

  const double MAX_SPEED = 2.0;
  double left_speed = 0.0;
  double right_speed = 0.0;
  while (robot->step(time_step) != -1) {
    // process camera image
    auto* img_data = camera->getImage();
    if (img_data) {
      std::memcpy(img.data, img_data, img_size);
      cv::cvtColor(img, img_gray, cv::COLOR_BGRA2GRAY);

      auto time_stamp = std::chrono::high_resolution_clock::now();
      slam_system.TrackMonocular(img_gray.clone(),
                                 time_stamp.time_since_epoch().count());
      out_img = slam_system.GetIniMatchImage();

      // show
      if (!out_img.empty()) {
        auto* ir = display_features->imageNew(out_img.cols, out_img.rows,
                                              out_img.data, Display::RGB);
        display_features->imagePaste(ir, 0, 0, false);
        display_features->imageDelete(ir);
      }
    }

    // keyboard control
    auto key = keyboard->getKey();
    switch (key) {
      case Keyboard::UP:
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
        break;
      case Keyboard::DOWN:
        left_speed = -MAX_SPEED;
        right_speed = -MAX_SPEED;
        break;
      case Keyboard::LEFT:
        left_speed = -MAX_SPEED;
        right_speed = MAX_SPEED;
        break;
      case Keyboard::RIGHT:
        left_speed = MAX_SPEED;
        right_speed = -MAX_SPEED;
        break;
      default:
        left_speed = 0.0;
        right_speed = 0.0;
    }
    if (key == 'I') {
      slam_system.ToggleInitializationAllowed();
    }

    // update motors
    wheels[0]->setVelocity(left_speed);
    wheels[1]->setVelocity(right_speed);
    wheels[2]->setVelocity(left_speed);
    wheels[3]->setVelocity(right_speed);
  };

  slam_system.StopGUI();
  slam_system.Shutdown();

  return 0;
}
