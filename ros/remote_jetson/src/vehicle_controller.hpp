#pragma once

#include "yolo_video_car_shared.hpp"

class VehicleController
{
public:
  void configure(rclcpp::Node & node);
  void initialize(rclcpp::Node & node);
  void centerCamera();
  void maybeRequestStreamOnStart(rclcpp::Node & node);
  void shutdown(rclcpp::Node & node);
  void applyAction(KeyAction action);
  float currentSpeed() const;
  float currentSteering() const;

private:
  void publishScalar(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher,
    float value);
  void publishDrive(float speed, float steering);
  void requestStream(rclcpp::Node & node, bool start);

  std::string speed_topic_;
  std::string steering_topic_;
  std::string camera_pan_topic_;
  std::string camera_tilt_topic_;
  std::string camera_start_service_;
  std::string camera_stop_service_;
  double base_speed_{20.0};
  double max_speed_{60.0};
  double turn_angle_deg_{28.0};
  bool auto_request_stream_{true};
  float current_speed_{0.0F};
  float current_steering_{0.0F};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr camera_pan_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr camera_tilt_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
};
