#pragma once

#include "yolo_video_car_shared.hpp"

class MediaCapture
{
public:
  void configure(rclcpp::Node & node);
  std::optional<std::string> capturePhoto(const cv::Mat & frame);
  std::string toggleVideoRecord(const cv::Mat & frame);
  std::string stopVideoRecord();
  void update(const cv::Mat & frame);
  void shutdown();
  std::string recordState() const;

private:
  std::string makeTimestamp() const;

  std::filesystem::path photo_directory_;
  std::filesystem::path video_directory_;
  std::string photo_prefix_{"photo"};
  std::string video_prefix_{"video"};
  cv::VideoWriter writer_;
  std::string record_state_{"stop"};
  std::filesystem::path current_video_path_;
  double video_fps_{20.0};
};
