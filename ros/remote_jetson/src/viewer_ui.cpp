#include "viewer_ui.hpp"

namespace
{
KeyAction decodeWindowKey(int key)
{
  const int normalized_ascii = key & 0xFF;
  const int normalized_x11_keysym = key & 0xFFFF;

  switch (key) {
    case kArrowUp:
    case 82:
    case 65362:
      return KeyAction::Forward;
    case kArrowDown:
    case 84:
    case 65364:
      return KeyAction::Backward;
    case kArrowLeft:
    case 81:
    case 65361:
      return KeyAction::Left;
    case kArrowRight:
    case 83:
    case 65363:
      return KeyAction::Right;
    default:
      break;
  }

  switch (normalized_x11_keysym) {
    case 0xFF52:
      return KeyAction::Forward;
    case 0xFF54:
      return KeyAction::Backward;
    case 0xFF51:
      return KeyAction::Left;
    case 0xFF53:
      return KeyAction::Right;
    case 0xFF1B:
      return KeyAction::Quit;
    case 0xFF0D:
    case 0xFF8D:
      return KeyAction::Stop;
    case 0xFFAB:
      return KeyAction::SpeedUp;
    case 0xFFAD:
      return KeyAction::SpeedDown;
    default:
      break;
  }

  switch (normalized_ascii) {
    case ' ':
    case '\r':
    case '\n':
      return KeyAction::Stop;
    case 't':
    case 'T':
      return KeyAction::CapturePhoto;
    case 'q':
    case 'Q':
      return KeyAction::ToggleVideoRecord;
    case 'e':
    case 'E':
      return KeyAction::StopVideoRecord;
    case '+':
    case '=':
      return KeyAction::SpeedUp;
    case '-':
    case '_':
      return KeyAction::SpeedDown;
    case 'f':
    case 'F':
      return KeyAction::ToggleDetector;
    case 'h':
    case 'H':
      return KeyAction::Help;
    case 'x':
    case 'X':
    case 27:
      return KeyAction::Quit;
    default:
      return KeyAction::None;
  }
}
}  // namespace

void ViewerUi::configure(rclcpp::Node & node)
{
  enabled_ = node.declare_parameter<bool>("enable_viewer", true);
  window_name_ = node.declare_parameter<std::string>("window_name", "PI-CAR-X Jetson Video Car");
  show_fps_ = node.declare_parameter<bool>("show_fps", true);
}

void ViewerUi::initialize(rclcpp::Node &)
{
  if (!enabled_) {
    return;
  }
  cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
  RCLCPP_INFO(rclcpp::get_logger("ViewerUi"), "OpenCV HighGUI backend: GTK2");
}

void ViewerUi::shutdown()
{
  if (!enabled_) {
    return;
  }
  try {
    cv::destroyWindow(window_name_);
  } catch (const cv::Exception &) {
  }
}

bool ViewerUi::enabled() const
{
  return enabled_;
}

void ViewerUi::printHelp(rclcpp::Node & node) const
{
  if (!enabled_) {
    RCLCPP_INFO(
      node.get_logger(),
      "Viewer disabled. Terminal keyboard teleoperation may still be available in the shell.");
    return;
  }
  RCLCPP_INFO(
    node.get_logger(),
    "Controls: arrows drive, space stop, +/- speed, t photo, q rec/pause, e rec stop, f detector, h help, x/esc quit.");
}

std::vector<KeyAction> ViewerUi::pollWindowActions(rclcpp::Node & node)
{
  std::vector<KeyAction> actions;
  if (!enabled_) {
    return actions;
  }

  const int key = cv::waitKeyEx(1);
  if (key < 0) {
    return actions;
  }

  const KeyAction action = decodeWindowKey(key);
  if (action == KeyAction::None) {
    RCLCPP_INFO_THROTTLE(
      node.get_logger(), *node.get_clock(), 2000,
      "Unhandled OpenCV key code: %d", key);
    return actions;
  }
  actions.push_back(action);
  return actions;
}

void ViewerUi::renderFrame(
  cv::Mat & frame,
  const VehicleController & controller,
  const DetectionPipeline & pipeline,
  const std::string & record_state)
{
  if (!enabled_) {
    return;
  }

  for (const auto & detection : pipeline.displayDetections()) {
    cv::rectangle(frame, detection.box, cv::Scalar(0, 220, 255), 2);
    std::ostringstream label;
    label << pipeline.className(detection.class_id) << ' '
          << static_cast<int>(std::round(detection.confidence * 100.0F)) << '%';
    cv::putText(
      frame, label.str(), {detection.box.x, std::max(20, detection.box.y - 8)},
      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 220, 255), 2);
  }

  drawStatusText(frame, controller, pipeline.getStats(), record_state);
  cv::imshow(window_name_, frame);
}

void ViewerUi::renderPlaceholder(
  const VehicleController & controller,
  const DetectionPipeline & pipeline,
  const std::string & record_state)
{
  if (!enabled_) {
    return;
  }

  cv::Mat placeholder(480, 640, CV_8UC3, cv::Scalar(16, 16, 16));
  cv::putText(
    placeholder, "Waiting for /picarx/camera/image_raw", {24, 80},
    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(240, 240, 240), 2);
  cv::putText(
    placeholder, "Launch the local hardware stack on the car first.", {24, 120},
    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(180, 180, 180), 1);
  drawStatusText(placeholder, controller, pipeline.getStats(), record_state);
  cv::imshow(window_name_, placeholder);
}

void ViewerUi::drawStatusText(
  cv::Mat & frame,
  const VehicleController & controller,
  const DetectionPipelineStats & stats,
  const std::string & record_state)
{
  std::ostringstream line1;
  line1 << "speed=" << controller.currentSpeed()
        << " steering=" << controller.currentSteering();
  cv::putText(
    frame, line1.str(), {16, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
    cv::Scalar(255, 255, 255), 2);

  std::ostringstream line2;
  line2 << "detector=" << (stats.detector_enabled ? "on" : "off");
  if (stats.detector_ready) {
    line2 << " backend=" << stats.backend_in_use;
  }
  line2 << " task=" << stats.model_task;
  line2 << " decoder=" << stats.decoder;
  line2 << " boxes=" << stats.display_detection_count
        << " tracks=" << stats.track_count
        << " rec=" << record_state;
  cv::putText(
    frame, line2.str(), {16, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
    cv::Scalar(200, 200, 200), 2);

  if (show_fps_) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - fps_window_start_).count();
    if (elapsed >= 1.0) {
      last_fps_ = static_cast<double>(fps_frame_counter_) / elapsed;
      fps_window_start_ = now;
      fps_frame_counter_ = 0;
    }
    ++fps_frame_counter_;

    const double yolo_potential_fps =
      stats.last_inference_ms > 0.0 ? 1000.0 / stats.last_inference_ms : 0.0;
    const double scheduled_detect_fps =
      stats.detector_enabled && stats.detect_every_n_frames > 0 ?
      last_fps_ / static_cast<double>(stats.detect_every_n_frames) : 0.0;

    std::ostringstream line3;
    line3 << "ui_fps=" << static_cast<int>(std::round(last_fps_))
          << " yolo=" << static_cast<int>(std::round(stats.last_inference_ms)) << "ms";
    cv::putText(
      frame, line3.str(), {16, 84}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(200, 200, 200), 2);

    std::ostringstream line4;
    line4 << "yolo_potential=" << std::fixed << std::setprecision(1) << yolo_potential_fps
          << "fps detect_sched=" << scheduled_detect_fps << "fps";
    cv::putText(
      frame, line4.str(), {16, 112}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(200, 200, 200), 2);
  }

  cv::putText(
    frame, "arrows drive | space stop | +/- speed | t photo | q/e video | f detector | x quit",
    {16, frame.rows - 18}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
}
