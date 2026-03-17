#pragma once

#include "detection_pipeline.hpp"
#include "vehicle_controller.hpp"

class ViewerUi
{
public:
  void configure(rclcpp::Node & node);
  void initialize(rclcpp::Node & node);
  void shutdown();
  bool enabled() const;
  void printHelp(rclcpp::Node & node) const;
  std::vector<KeyAction> pollActions(rclcpp::Node & node);
  void renderFrame(
    cv::Mat & frame,
    const VehicleController & controller,
    const DetectionPipeline & pipeline);
  void renderPlaceholder(
    const VehicleController & controller,
    const DetectionPipeline & pipeline);

private:
  void drawStatusText(
    cv::Mat & frame,
    const VehicleController & controller,
    const DetectionPipelineStats & stats);

  bool enabled_{true};
  bool show_fps_{true};
  bool terminal_keyboard_available_{false};
  double last_fps_{0.0};
  int fps_frame_counter_{0};
  std::chrono::steady_clock::time_point fps_window_start_{std::chrono::steady_clock::now()};
  std::string window_name_{"PI-CAR-X Jetson Video Car"};
  TerminalKeyboard terminal_keyboard_;
};
