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
    const auto backend = camera_backend_.empty() ? std::string("auto") : camera_backend_;
    const bool auto_backend = backend == "auto";

    auto make_device_option = [&](const std::string & desc, int index, int api) {
      return CaptureOption{desc, "", api, false, index};
    };
    auto make_string_option = [&](const std::string & desc, const std::string & source, int api) {
      return CaptureOption{desc, source, api, true, 0};
    };

    std::vector<CaptureOption> options;

    if (!camera_source_.empty()) {
      if (is_integer_string(camera_source_)) {
        const int index = std::stoi(camera_source_);
        if (backend == "v4l2" || auto_backend) {
          options.push_back(make_device_option("V4L2 device index " + std::to_string(index), index, cv::CAP_V4L2));
        }
        if (backend == "opencv" || auto_backend) {
          options.push_back(make_device_option("OpenCV camera index " + std::to_string(index), index, cv::CAP_ANY));
        }
      } else {
        const int api = backend == "gstreamer" || auto_backend ? cv::CAP_GSTREAMER : cv::CAP_ANY;
        options.push_back(make_string_option("Configured camera source", camera_source_, api));
      }
      return options;
    }

    if (!gstreamer_pipeline_.empty()) {
      options.push_back(make_string_option("Configured GStreamer pipeline", gstreamer_pipeline_, cv::CAP_GSTREAMER));
      return options;
    }

    if (backend == "gstreamer" || auto_backend) {
      options.push_back(make_string_option(
        "libcamerasrc GStreamer pipeline",
        make_libcamera_pipeline(width_, height_, fps_, camera_auto_exposure_, camera_controls_),
        cv::CAP_GSTREAMER));
    }
    if (backend == "v4l2" || auto_backend) {
      options.push_back(make_device_option("V4L2 device index 0", 0, cv::CAP_V4L2));
    }
    if (backend == "opencv" || auto_backend) {
      options.push_back(make_device_option("OpenCV camera index 0", 0, cv::CAP_ANY));
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
        get_logger(), *get_clock(), 5000, "Failed to read camera frame via %s",
        active_capture_description_.c_str());
      return;
    }

    if (frame.channels() == 4) {
      cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
    } else if (frame.channels() == 1) {
      cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<std::uint32_t>(frame.rows);
    msg.width = static_cast<std::uint32_t>(frame.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    const auto bytes = frame.total() * frame.elemSize();
    msg.data.resize(bytes);
    std::memcpy(msg.data.data(), frame.data, bytes);
    publisher_->publish(msg);
    RCLCPP_INFO_ONCE(
      get_logger(),
      "First frame published on %s (%ux%u)",
      publisher_->get_topic_name(),
      msg.width,
      msg.height);
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
  std::string active_capture_description_;
  bool streaming_enabled_{false};
  cv::VideoCapture capture_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PicarxCameraPublisherNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(rclcpp::get_logger("picarx_camera_publisher_node"), "%s", exc.what());
  }
  rclcpp::shutdown();
  return 0;
}
