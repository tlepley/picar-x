#pragma once

#include "yolo_video_car_shared.hpp"

class DetectionPipeline
{
public:
  void configure(rclcpp::Node & node);
  void initialize(rclcpp::Node & node);
  void processFrame(rclcpp::Node & node, const cv::Mat & frame);
  void setDetectorEnabled(bool enabled);
  bool toggleDetectorEnabled();
  const std::vector<Detection> & displayDetections() const;
  std::string className(int class_id) const;
  DetectionPipelineStats getStats() const;

private:
  void clearTracks();
  void updateTracks(const std::vector<Detection> & detections, int image_width, int image_height);

  std::string engine_path_;
  std::string classes_path_;
  ModelTask model_task_{ModelTask::Detect};
  std::string yolo_variant_{"yolov8"};
  std::string decoder_name_{"ultralytics"};
  int input_width_{640};
  int input_height_{640};
  int detect_every_n_frames_{2};
  double confidence_threshold_{0.25};
  double score_threshold_{0.25};
  double nms_threshold_{0.45};
  double tracker_iou_threshold_{0.35};
  int tracker_confirm_frames_{2};
  int tracker_max_missed_frames_{3};
  double tracker_smoothing_alpha_{0.65};
  double tracker_confidence_alpha_{0.7};
  bool detector_ready_{false};
  bool detector_enabled_{false};
  std::size_t frame_counter_{0};
  double last_inference_ms_{0.0};
  std::string backend_in_use_{"TensorRT"};
  std::string model_name_;
  int next_track_id_{1};
  TensorRtYoloEngine detector_;
  std::vector<std::string> classes_;
  std::vector<Detection> last_raw_detections_;
  std::vector<Detection> last_display_detections_;
  std::vector<Track> tracks_;
};
