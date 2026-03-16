#include <chrono>
#include <cctype>
#include <cstring>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
std::string trim(const std::string & value)
{
  const auto start = value.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(start, end - start + 1);
}

bool is_integer_string(const std::string & value)
{
  if (value.empty()) {
    return false;
  }
  std::size_t start = 0;
  if (value[0] == '-' || value[0] == '+') {
    start = 1;
  }
  if (start >= value.size()) {
    return false;
  }
  for (std::size_t i = start; i < value.size(); ++i) {
    if (!std::isdigit(static_cast<unsigned char>(value[i]))) {
      return false;
    }
  }
  return true;
}

std::string make_libcamera_pipeline(
  int width, int height, double fps, bool auto_exposure, const std::string & extra_controls)
{
  std::ostringstream stream;
  const auto trimmed_controls = trim(extra_controls);
  stream << "libcamerasrc";
  if (auto_exposure) {
    stream << " ae-enable=true";
  }
  if (!trimmed_controls.empty()) {
    stream << " " << trimmed_controls;
  }

  stream
    << " ! video/x-raw,width=" << width
    << ",height=" << height
    << ",framerate=" << static_cast<int>(fps) << "/1 ! "
    << "videoconvert ! video/x-raw,format=BGR ! "
    << "appsink drop=true max-buffers=1 sync=false";
  return stream.str();
}

struct CaptureOption
{
  std::string description;
  std::string source_string;
  int api_preference;
  bool use_string_source;
  int index_source;
};
}  // namespace

class PicarxCameraPublisherNode : public rclcpp::Node
{
public:
  PicarxCameraPublisherNode()
  : Node("picarx_camera_publisher_node")
  {
    declare_parameter<std::string>("camera_source", "");
    declare_parameter<std::string>("camera_backend", "auto");
    declare_parameter<std::string>("gstreamer_pipeline", "");
    declare_parameter<std::string>("image_topic", "/picarx/camera/image_raw");
    declare_parameter<std::string>("frame_id", "picarx_camera");
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<int>("width", 640);
    declare_parameter<int>("height", 480);
    declare_parameter<double>("fps", 30.0);
    declare_parameter<bool>("camera_auto_exposure", true);
    declare_parameter<std::string>("camera_controls", "");
    declare_parameter<bool>("start_stream_on_launch", false);

    camera_source_ = trim(get_parameter("camera_source").as_string());
    camera_backend_ = trim(get_parameter("camera_backend").as_string());
    gstreamer_pipeline_ = trim(get_parameter("gstreamer_pipeline").as_string());
    image_topic_ = get_parameter("image_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    publish_rate_hz_ = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    width_ = static_cast<int>(std::max<int64_t>(1, get_parameter("width").as_int()));
    height_ = static_cast<int>(std::max<int64_t>(1, get_parameter("height").as_int()));
    fps_ = std::max(1.0, get_parameter("fps").as_double());
    camera_auto_exposure_ = get_parameter("camera_auto_exposure").as_bool();
    camera_controls_ = trim(get_parameter("camera_controls").as_string());
    start_stream_on_launch_ = get_parameter("start_stream_on_launch").as_bool();

    publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      std::bind(&PicarxCameraPublisherNode::handle_start, this, std::placeholders::_1, std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      std::bind(&PicarxCameraPublisherNode::handle_stop, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
      get_logger(),
      "Created publisher for topic %s",
      publisher_->get_topic_name());
    if (start_stream_on_launch_) {
      RCLCPP_INFO(get_logger(), "Camera stream auto-start is enabled.");
      start_capture();
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Camera stream auto-start is disabled. Waiting for %s",
        start_service_->get_service_name());
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PicarxCameraPublisherNode::publish_frame, this));
    if (!streaming_enabled_) {
      timer_->cancel();
    }

    RCLCPP_INFO(
      get_logger(),
      "Publishing camera frames on %s using %s",
      image_topic_.c_str(),
      active_capture_description_.c_str());
  }

private:
  void start_capture()
  {
    if (streaming_enabled_) {
      return;
    }

    open_capture();
    streaming_enabled_ = true;
    if (timer_) {
      timer_->reset();
    }
    RCLCPP_INFO(
      get_logger(),
      "Camera stream started on %s using %s",
      image_topic_.c_str(),
      active_capture_description_.c_str());
  }

  void stop_capture()
  {
    if (!streaming_enabled_) {
      return;
    }

    if (timer_) {
      timer_->cancel();
    }
    if (capture_.isOpened()) {
      capture_.release();
    }
    streaming_enabled_ = false;
    RCLCPP_INFO(get_logger(), "Camera stream stopped on %s", image_topic_.c_str());
  }

  void open_capture()
  {
    const auto options = build_capture_options();
    std::vector<std::string> errors;

    for (const auto & option : options) {
      cv::VideoCapture candidate;
      if (option.use_string_source) {
        candidate.open(option.source_string, option.api_preference);
      } else {
        candidate.open(option.index_source, option.api_preference);
      }

      if (!candidate.isOpened()) {
        errors.push_back(option.description + ": open failed");
        continue;
      }

      if (!option.use_string_source) {
        candidate.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        candidate.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        candidate.set(cv::CAP_PROP_FPS, fps_);
      }

      capture_ = std::move(candidate);
      active_capture_description_ = option.description;
      return;
    }

    std::ostringstream message;
    message << "Failed to open any camera source";
    if (!errors.empty()) {
      message << " [";
      for (std::size_t i = 0; i < errors.size(); ++i) {
        if (i != 0) {
          message << "; ";
        }
        message << errors[i];
      }
      message << "]";
    }
    throw std::runtime_error(message.str());
  }

  void handle_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (streaming_enabled_) {
      response->success = true;
      response->message = "Camera stream already running.";
      return;
    }

    try {
      start_capture();
      response->success = true;
      response->message = "Camera stream started.";
    } catch (const std::exception & exc) {
      response->success = false;
      response->message = exc.what();
      RCLCPP_ERROR(get_logger(), "Failed to start camera stream: %s", exc.what());
    }
  }

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!streaming_enabled_) {
      response->success = true;
      response->message = "Camera stream already stopped.";
      return;
    }

    stop_capture();
    response->success = true;
    response->message = "Camera stream stopped.";
  }

  std::vector<CaptureOption> build_capture_options() const
  {
    std::vector<CaptureOption> options;

    const bool wants_gstreamer = camera_backend_ == "gstreamer";
    const bool wants_default = camera_backend_.empty() || camera_backend_ == "auto";
    const bool wants_opencv = camera_backend_ == "opencv";

    if (wants_gstreamer || wants_default) {
      const auto pipeline = !gstreamer_pipeline_.empty() ?
        gstreamer_pipeline_ :
        make_libcamera_pipeline(width_, height_, fps_, camera_auto_exposure_, camera_controls_);
      options.push_back({"gstreamer pipeline", pipeline, cv::CAP_GSTREAMER, true, -1});
    }

    if (!camera_source_.empty()) {
      if (is_integer_string(camera_source_)) {
        options.push_back({"OpenCV camera index " + camera_source_, "", cv::CAP_ANY, false, std::stoi(camera_source_)});
      } else if (wants_opencv || wants_default) {
        options.push_back({"OpenCV source " + camera_source_, camera_source_, cv::CAP_ANY, true, -1});
      }
    } else if (wants_opencv || wants_default) {
      options.push_back({"default OpenCV camera index 0", "", cv::CAP_ANY, false, 0});
    }

    return options;
  }

  void publish_frame()
  {
    if (!streaming_enabled_ || !capture_.isOpened()) {
      return;
    }

    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Failed to read camera frame.");
      return;
    }

    auto message = sensor_msgs::msg::Image();
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.height = static_cast<uint32_t>(frame.rows);
    message.width = static_cast<uint32_t>(frame.cols);
    message.encoding = "bgr8";
    message.is_bigendian = false;
    message.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    const auto size = frame.total() * frame.elemSize();
    message.data.resize(size);
    std::memcpy(message.data.data(), frame.data, size);
    publisher_->publish(std::move(message));
  }

  std::string camera_source_;
  std::string camera_backend_;
  std::string gstreamer_pipeline_;
  std::string image_topic_;
  std::string frame_id_;
  double publish_rate_hz_{10.0};
  int width_{640};
  int height_{480};
  double fps_{30.0};
  bool camera_auto_exposure_{true};
  std::string camera_controls_;
  bool start_stream_on_launch_{false};
  bool streaming_enabled_{false};
  std::string active_capture_description_{"camera stream not started"};
  cv::VideoCapture capture_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PicarxCameraPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
