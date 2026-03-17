#include "viewer_ui.hpp"

void ViewerUi::configure(rclcpp::Node & node)
{
  enabled_ = node.declare_parameter<bool>("enable_viewer", true);
  window_name_ = node.declare_parameter<std::string>("window_name", "PI-CAR-X Jetson Video Car");
  show_fps_ = node.declare_parameter<bool>("show_fps", true);
}

void ViewerUi::initialize(rclcpp::Node & node)
{
  if (!enabled_) {
    return;
  }
  terminal_keyboard_available_ = terminal_keyboard_.open();
  if (terminal_keyboard_available_) {
    RCLCPP_INFO(
      node.get_logger(),
      "Terminal keyboard capture enabled. Keep the launch terminal focused for reliable arrow keys.");
  } else {
    RCLCPP_WARN(
      node.get_logger(),
      "Terminal keyboard capture unavailable; falling back to OpenCV window key handling.");
  }
  cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
}

void ViewerUi::shutdown()
{
  if (!enabled_) {
    return;
  }
  terminal_keyboard_.close();
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
    RCLCPP_INFO(node.get_logger(), "Viewer disabled. Manual keyboard teleoperation is unavailable.");
    return;
  }
  RCLCPP_INFO(
    node.get_logger(),
    "Controls: arrows in terminal or OpenCV window, space stop, +/- speed, f toggle detector, h help, x/esc quit.");
}

std::vector<KeyAction> ViewerUi::pollActions(rclcpp::Node & node)
{
  std::vector<KeyAction> actions;
  if (!enabled_) {
    return actions;
  }

  actions = terminal_keyboard_.pollActions();
  const int key = cv::waitKeyEx(1);
  if (key < 0) {
    return actions;
  }

  switch (key) {
    case kArrowUp:
      actions.push_back(KeyAction::Forward);
      return actions;
    case kArrowDown:
      actions.push_back(KeyAction::Backward);
      return actions;
    case kArrowLeft:
      actions.push_back(KeyAction::Left);
      return actions;
    case kArrowRight:
      actions.push_back(KeyAction::Right);
      return actions;
    case ' ':
      actions.push_back(KeyAction::Stop);
      return actions;
    case '+':
    case '=':
      actions.push_back(KeyAction::SpeedUp);
      return actions;
    case '-':
    case '_':
      actions.push_back(KeyAction::SpeedDown);
      return actions;
    case 'f':
    case 'F':
      actions.push_back(KeyAction::ToggleDetector);
      return actions;
    case 'h':
    case 'H':
      actions.push_back(KeyAction::Help);
      return actions;
    case 'x':
    case 'X':
    case 27:
      actions.push_back(KeyAction::Quit);
      return actions;
    default:
      RCLCPP_INFO_THROTTLE(
        node.get_logger(), *node.get_clock(), 2000,
        "Unhandled OpenCV key code: %d", key);
      return actions;
  }
}

void ViewerUi::renderFrame(
  cv::Mat & frame,
  const VehicleController & controller,
  const DetectionPipeline & pipeline)
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

  drawStatusText(frame, controller, pipeline.getStats());
  cv::imshow(window_name_, frame);
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
  drawStatusText(placeholder, controller, pipeline.getStats());
  cv::imshow(window_name_, placeholder);
}

void ViewerUi::drawStatusText(
  cv::Mat & frame,
  const VehicleController & controller,
  const DetectionPipelineStats & stats)
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
        << " tracks=" << stats.track_count;
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
    frame, "terminal arrows drive | space stop | +/- speed | f detector | x quit",
    {16, frame.rows - 18}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
}
