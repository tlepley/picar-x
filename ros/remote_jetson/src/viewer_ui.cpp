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
    case 'p':
    case 'P':
      return KeyAction::CapturePhoto;
    case 'v':
    case 'V':
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
    case 'q':
    case 'Q':
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
    "Controls: arrows drive, space stop, +/- speed, p photo, v rec/pause, e rec stop, f detector, h help, q/esc quit.");
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

  if (record_state == "start") {
    const auto now = std::chrono::steady_clock::now();
    const double blink_phase = std::chrono::duration<double>(now.time_since_epoch()).count();
    if (std::fmod(blink_phase, 1.0) < 0.5) {
      const cv::Point center(frame.cols - 24, 24);
      cv::circle(frame, center, 10, cv::Scalar(0, 0, 255), cv::FILLED);
      cv::circle(frame, center, 12, cv::Scalar(220, 220, 220), 1);
    }
  }

  const cv::Mat status_panel = buildStatusPanel(
    frame.cols, frame.rows, controller, pipeline.getStats());
  cv::Mat composed(frame.rows + status_panel.rows, frame.cols, CV_8UC3, cv::Scalar(18, 18, 18));
  frame.copyTo(composed(cv::Rect(0, 0, frame.cols, frame.rows)));
  status_panel.copyTo(composed(cv::Rect(0, frame.rows, status_panel.cols, status_panel.rows)));
  cv::imshow(window_name_, composed);
}

void ViewerUi::renderPlaceholder(
  const VehicleController & controller,
  const DetectionPipeline & pipeline)
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
  const cv::Mat status_panel = buildStatusPanel(
    placeholder.cols, placeholder.rows, controller, pipeline.getStats());
  cv::Mat composed(
    placeholder.rows + status_panel.rows, placeholder.cols, CV_8UC3, cv::Scalar(18, 18, 18));
  placeholder.copyTo(composed(cv::Rect(0, 0, placeholder.cols, placeholder.rows)));
  status_panel.copyTo(
    composed(cv::Rect(0, placeholder.rows, status_panel.cols, status_panel.rows)));
  cv::imshow(window_name_, composed);
}

cv::Mat ViewerUi::buildStatusPanel(
  int width,
  int height,
  const VehicleController & controller,
  const DetectionPipelineStats & stats)
{
  constexpr int kPanelHeight = 216;
  cv::Mat panel(kPanelHeight, width, CV_8UC3, cv::Scalar(24, 24, 24));
  cv::line(panel, {0, 0}, {width, 0}, cv::Scalar(60, 60, 60), 1);

  std::ostringstream line1;
  line1 << "speed=" << controller.currentSpeed()
        << " steering=" << controller.currentSteering();
  cv::putText(
    panel, line1.str(), {16, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
    cv::Scalar(255, 255, 255), 1);

  std::ostringstream line2;
  line2 << "stream=" << width << "x" << height;
  if (stats.detector_ready) {
    line2 << " " << stats.backend_in_use;
  }
  cv::putText(
    panel, line2.str(), {16, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
    cv::Scalar(200, 200, 200), 1);

  std::ostringstream lineModel;
  if (!stats.model_name.empty()) {
    lineModel << "model=" << stats.model_name;
    if (!stats.decoder.empty()) {
      lineModel << " (" << stats.decoder << ")";
    }
  }
  cv::putText(
    panel, lineModel.str(), {16, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
    cv::Scalar(200, 200, 200), 1);

  std::ostringstream lineDetection;
  lineDetection << "detector=" << (stats.detector_enabled ? "on" : "off")
                << " tracked_objects=" << stats.display_detection_count;
  cv::putText(
    panel, lineDetection.str(), {16, 104}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
    cv::Scalar(200, 200, 200), 1);

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
          << " detect_sched=" << std::fixed << std::setprecision(1) << scheduled_detect_fps
          << "fps";
    cv::putText(
      panel, line3.str(), {16, 132}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(200, 200, 200), 1);

    std::ostringstream line4;
    line4 << "yolo=" << static_cast<int>(std::round(stats.last_inference_ms))
          << "ms yolo_potential=" << std::fixed << std::setprecision(1) << yolo_potential_fps
          << "fps";
    cv::putText(
      panel, line4.str(), {16, 156}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(200, 200, 200), 1);
  }

  cv::putText(
    panel, "arrows drive | space stop | +/- speed | p photo | v/e video | f detector | q quit",
    {16, panel.rows - 14}, cv::FONT_HERSHEY_SIMPLEX, 0.47, cv::Scalar(0, 255, 255), 1);

  return panel;
}
