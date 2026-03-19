#include "media_capture.hpp"

void MediaCapture::configure(rclcpp::Node & node)
{
  photo_directory_ = node.declare_parameter<std::string>(
    "photo_directory",
    (std::filesystem::path(std::getenv("HOME") != nullptr ? std::getenv("HOME") : "/tmp") /
    "Pictures" / "picar-x").string());
  video_directory_ = node.declare_parameter<std::string>(
    "video_directory",
    (std::filesystem::path(std::getenv("HOME") != nullptr ? std::getenv("HOME") : "/tmp") /
    "Videos").string());
  video_fps_ = node.declare_parameter<double>("video_record_fps", 20.0);
}

std::optional<std::string> MediaCapture::capturePhoto(const cv::Mat & frame)
{
  if (frame.empty()) {
    return std::nullopt;
  }

  std::filesystem::create_directories(photo_directory_);
  const std::filesystem::path path = photo_directory_ / (photo_prefix_ + "_" + makeTimestamp() + ".jpg");
  if (!cv::imwrite(path.string(), frame)) {
    return std::nullopt;
  }
  return path.string();
}

std::string MediaCapture::toggleVideoRecord(const cv::Mat & frame)
{
  if (frame.empty()) {
    return "No frame available yet.";
  }

  if (record_state_ == "stop") {
    std::filesystem::create_directories(video_directory_);
    current_video_path_ = video_directory_ / (video_prefix_ + "_" + makeTimestamp() + ".avi");
    writer_ = cv::VideoWriter(
      current_video_path_.string(),
      cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
      video_fps_,
      cv::Size(frame.cols, frame.rows));
    if (!writer_.isOpened()) {
      record_state_ = "stop";
      current_video_path_.clear();
      return "Could not open video writer.";
    }
    record_state_ = "start";
    return "Recording started: " + current_video_path_.string();
  }

  if (record_state_ == "start") {
    record_state_ = "pause";
    return "Recording paused.";
  }

  record_state_ = "start";
  return "Recording resumed.";
}

std::string MediaCapture::stopVideoRecord()
{
  if (record_state_ == "stop") {
    return "Recording already stopped.";
  }

  record_state_ = "stop";
  if (writer_.isOpened()) {
    writer_.release();
  }
  return "Recording stopped.";
}

void MediaCapture::update(const cv::Mat & frame)
{
  if (record_state_ == "start" && writer_.isOpened() && !frame.empty()) {
    writer_.write(frame);
  }
}

void MediaCapture::shutdown()
{
  if (writer_.isOpened()) {
    writer_.release();
  }
  record_state_ = "stop";
}

std::string MediaCapture::recordState() const
{
  return record_state_;
}

std::string MediaCapture::makeTimestamp() const
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
  localtime_r(&now_time, &local_tm);
  std::ostringstream out;
  out << std::put_time(&local_tm, "%Y-%m-%d-%H-%M-%S");
  return out.str();
}
