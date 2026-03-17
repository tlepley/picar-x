#include "detection_pipeline.hpp"

void DetectionPipeline::configure(rclcpp::Node & node)
{
  engine_path_ = node.declare_parameter<std::string>("engine_path", "");
  classes_path_ = node.declare_parameter<std::string>("classes_path", "");
  model_task_ = parseModelTask(node.declare_parameter<std::string>("model_task", "detect"));
  yolo_variant_ = detectDecoderName(
    parseDetectDecoder(node.declare_parameter<std::string>("yolo_variant", "yolov8")));
  input_width_ = node.declare_parameter<int>("input_width", 640);
  input_height_ = node.declare_parameter<int>("input_height", 640);
  detect_every_n_frames_ = std::max(
    1, static_cast<int>(node.declare_parameter<int>("detect_every_n_frames", 2)));
  confidence_threshold_ = node.declare_parameter<double>("confidence_threshold", 0.25);
  score_threshold_ = node.declare_parameter<double>("score_threshold", 0.25);
  nms_threshold_ = node.declare_parameter<double>("nms_threshold", 0.45);
  tracker_iou_threshold_ = node.declare_parameter<double>("tracker_iou_threshold", 0.35);
  tracker_confirm_frames_ = std::max(
    1, static_cast<int>(node.declare_parameter<int>("tracker_confirm_frames", 2)));
  tracker_max_missed_frames_ = std::max(
    0, static_cast<int>(node.declare_parameter<int>("tracker_max_missed_frames", 3)));
  tracker_smoothing_alpha_ = node.declare_parameter<double>("tracker_smoothing_alpha", 0.65);
  tracker_confidence_alpha_ = node.declare_parameter<double>("tracker_confidence_alpha", 0.7);
  classes_ = loadClasses(classes_path_);
}

void DetectionPipeline::initialize(rclcpp::Node & node)
{
  if (model_task_ != ModelTask::Detect) {
    detector_ready_ = false;
    detector_enabled_ = false;
    RCLCPP_ERROR(
      node.get_logger(),
      "model_task='%s' is not supported by this native detector. Supported task: detect.",
      modelTaskName(model_task_).c_str());
    return;
  }

  if (engine_path_.empty()) {
    detector_ready_ = false;
    detector_enabled_ = false;
    RCLCPP_WARN(node.get_logger(), "YOLO disabled: parameter 'engine_path' is empty.");
    return;
  }

  try {
    detector_.load(engine_path_, input_width_, input_height_);
    detector_ready_ = true;
    detector_enabled_ = true;
    backend_in_use_ = detector_.runtimeName();
    RCLCPP_INFO(
      node.get_logger(),
      "Loaded TensorRT engine: %s (task=%s, decoder=%s)",
      engine_path_.c_str(),
      modelTaskName(model_task_).c_str(),
      yolo_variant_.c_str());
  } catch (const std::exception & exc) {
    detector_ready_ = false;
    detector_enabled_ = false;
    RCLCPP_ERROR(node.get_logger(), "Failed to load TensorRT engine: %s", exc.what());
  }
}

void DetectionPipeline::processFrame(rclcpp::Node & node, const cv::Mat & frame)
{
  ++frame_counter_;
  if (detector_ready_ && detector_enabled_ && (frame_counter_ % detect_every_n_frames_ == 0)) {
    try {
      last_raw_detections_ = detector_.infer(
        frame, score_threshold_, confidence_threshold_, nms_threshold_, yolo_variant_);
      updateTracks(last_raw_detections_, frame.cols, frame.rows);
      backend_in_use_ = detector_.runtimeName();
      last_inference_ms_ = detector_.lastInferenceMs();
    } catch (const std::exception & exc) {
      detector_enabled_ = false;
      clearTracks();
      RCLCPP_ERROR_THROTTLE(
        node.get_logger(), *node.get_clock(), 2000,
        "TensorRT inference failed, detector disabled: %s", exc.what());
    }
  } else if (!detector_enabled_) {
    clearTracks();
  }
}

void DetectionPipeline::setDetectorEnabled(bool enabled)
{
  detector_enabled_ = detector_ready_ && enabled;
  if (!detector_enabled_) {
    clearTracks();
  }
}

bool DetectionPipeline::toggleDetectorEnabled()
{
  setDetectorEnabled(!detector_enabled_);
  return detector_enabled_;
}

const std::vector<Detection> & DetectionPipeline::displayDetections() const
{
  return last_display_detections_;
}

std::string DetectionPipeline::className(int class_id) const
{
  if (class_id >= 0 && static_cast<std::size_t>(class_id) < classes_.size()) {
    return classes_[static_cast<std::size_t>(class_id)];
  }
  std::ostringstream out;
  out << "cls_" << class_id;
  return out.str();
}

DetectionPipelineStats DetectionPipeline::getStats() const
{
  DetectionPipelineStats stats;
  stats.detector_ready = detector_ready_;
  stats.detector_enabled = detector_enabled_;
  stats.backend_in_use = backend_in_use_;
  stats.model_task = modelTaskName(model_task_);
  stats.decoder = yolo_variant_;
  stats.detect_every_n_frames = detect_every_n_frames_;
  stats.last_inference_ms = last_inference_ms_;
  stats.raw_detection_count = last_raw_detections_.size();
  stats.display_detection_count = last_display_detections_.size();
  stats.track_count = tracks_.size();
  return stats;
}

void DetectionPipeline::clearTracks()
{
  tracks_.clear();
  last_raw_detections_.clear();
  last_display_detections_.clear();
}

void DetectionPipeline::updateTracks(
  const std::vector<Detection> & detections, int image_width, int image_height)
{
  for (auto & track : tracks_) {
    track.matched_in_frame = false;
  }

  std::vector<bool> used_detections(detections.size(), false);
  for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
    const auto & detection = detections[detection_index];
    float best_iou = static_cast<float>(tracker_iou_threshold_);
    auto best_track = tracks_.end();
    for (auto it = tracks_.begin(); it != tracks_.end(); ++it) {
      if (it->matched_in_frame || it->class_id != detection.class_id) {
        continue;
      }
      const float iou = intersectionOverUnion(it->box, cv::Rect2f(detection.box));
      if (iou > best_iou) {
        best_iou = iou;
        best_track = it;
      }
    }
    if (best_track == tracks_.end()) {
      continue;
    }

    const float box_alpha = std::clamp(static_cast<float>(tracker_smoothing_alpha_), 0.0F, 0.98F);
    const float conf_alpha = std::clamp(static_cast<float>(tracker_confidence_alpha_), 0.0F, 0.98F);
    best_track->box = blendRect(best_track->box, detection.box, box_alpha);
    best_track->confidence =
      (conf_alpha * best_track->confidence) + ((1.0F - conf_alpha) * detection.confidence);
    best_track->hits += 1;
    best_track->misses = 0;
    best_track->matched_in_frame = true;
    if (!best_track->confirmed && best_track->hits >= tracker_confirm_frames_) {
      best_track->confirmed = true;
    }
    used_detections[detection_index] = true;
  }

  for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
    if (used_detections[detection_index]) {
      continue;
    }
    const auto & detection = detections[detection_index];
    tracks_.push_back(Track{
      next_track_id_++,
      detection.class_id,
      detection.confidence,
      cv::Rect2f(detection.box),
      1,
      0,
      tracker_confirm_frames_ <= 1,
      true});
  }

  for (auto & track : tracks_) {
    if (!track.matched_in_frame) {
      track.misses += 1;
    }
  }

  tracks_.erase(
    std::remove_if(
      tracks_.begin(),
      tracks_.end(),
      [this](const Track & track) {return track.misses > tracker_max_missed_frames_;}),
    tracks_.end());

  last_display_detections_.clear();
  for (const auto & track : tracks_) {
    if (!track.confirmed) {
      continue;
    }
    const cv::Rect box = roundedRect(track.box, image_width, image_height);
    if (box.area() <= 0) {
      continue;
    }
    last_display_detections_.push_back(Detection{track.class_id, track.confidence, box});
  }
}
