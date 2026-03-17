#include "vehicle_controller.hpp"

void VehicleController::configure(rclcpp::Node & node)
{
  speed_topic_ = node.declare_parameter<std::string>("speed_topic", "/picarx/speed");
  steering_topic_ = node.declare_parameter<std::string>("steering_topic", "/picarx/steering");
  camera_pan_topic_ = node.declare_parameter<std::string>("camera_pan_topic", "/picarx/camera_pan");
  camera_tilt_topic_ = node.declare_parameter<std::string>("camera_tilt_topic", "/picarx/camera_tilt");
  camera_start_service_ = node.declare_parameter<std::string>(
    "camera_start_service", "/picarx_camera_publisher_node/start");
  camera_stop_service_ = node.declare_parameter<std::string>(
    "camera_stop_service", "/picarx_camera_publisher_node/stop");
  base_speed_ = node.declare_parameter<double>("base_speed", 20.0);
  max_speed_ = node.declare_parameter<double>("max_speed", 60.0);
  turn_angle_deg_ = node.declare_parameter<double>("turn_angle_deg", 28.0);
  auto_request_stream_ = node.declare_parameter<bool>("auto_request_stream", true);
}

void VehicleController::initialize(rclcpp::Node & node)
{
  speed_pub_ = node.create_publisher<std_msgs::msg::Float32>(speed_topic_, 10);
  steering_pub_ = node.create_publisher<std_msgs::msg::Float32>(steering_topic_, 10);
  camera_pan_pub_ = node.create_publisher<std_msgs::msg::Float32>(camera_pan_topic_, 10);
  camera_tilt_pub_ = node.create_publisher<std_msgs::msg::Float32>(camera_tilt_topic_, 10);
  start_client_ = node.create_client<std_srvs::srv::Trigger>(camera_start_service_);
  stop_client_ = node.create_client<std_srvs::srv::Trigger>(camera_stop_service_);
}

void VehicleController::centerCamera()
{
  publishScalar(camera_pan_pub_, 0.0F);
  publishScalar(camera_tilt_pub_, 0.0F);
}

void VehicleController::maybeRequestStreamOnStart(rclcpp::Node & node)
{
  if (auto_request_stream_) {
    requestStream(node, true);
  }
}

void VehicleController::shutdown(rclcpp::Node & node)
{
  publishDrive(0.0F, 0.0F);
  requestStream(node, false);
}

void VehicleController::applyAction(KeyAction action)
{
  switch (action) {
    case KeyAction::Forward:
      publishDrive(current_speed_ <= 0.0F ? static_cast<float>(base_speed_) : current_speed_, 0.0F);
      break;
    case KeyAction::Backward:
      publishDrive(current_speed_ >= 0.0F ? -static_cast<float>(base_speed_) : current_speed_, 0.0F);
      break;
    case KeyAction::Left:
      publishDrive(static_cast<float>(base_speed_), -static_cast<float>(turn_angle_deg_));
      break;
    case KeyAction::Right:
      publishDrive(static_cast<float>(base_speed_), static_cast<float>(turn_angle_deg_));
      break;
    case KeyAction::Stop:
      publishDrive(0.0F, 0.0F);
      break;
    case KeyAction::SpeedUp:
      current_speed_ = std::min(current_speed_ + 5.0F, static_cast<float>(max_speed_));
      publishDrive(current_speed_, current_steering_);
      break;
    case KeyAction::SpeedDown:
      current_speed_ = std::max(current_speed_ - 5.0F, -static_cast<float>(max_speed_));
      publishDrive(current_speed_, current_steering_);
      break;
    case KeyAction::None:
    case KeyAction::ToggleDetector:
    case KeyAction::Help:
    case KeyAction::Quit:
      break;
  }
}

float VehicleController::currentSpeed() const
{
  return current_speed_;
}

float VehicleController::currentSteering() const
{
  return current_steering_;
}

void VehicleController::publishScalar(
  const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher,
  float value)
{
  std_msgs::msg::Float32 msg;
  msg.data = value;
  publisher->publish(msg);
}

void VehicleController::publishDrive(float speed, float steering)
{
  speed = std::clamp(speed, -static_cast<float>(max_speed_), static_cast<float>(max_speed_));
  steering = std::clamp(steering, -30.0F, 30.0F);
  current_speed_ = speed;
  current_steering_ = steering;
  publishScalar(speed_pub_, current_speed_);
  publishScalar(steering_pub_, current_steering_);
}

void VehicleController::requestStream(rclcpp::Node & node, bool start)
{
  auto & client = start ? start_client_ : stop_client_;
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(
      node.get_logger(), "Camera %s service unavailable: %s",
      start ? "start" : "stop", client->get_service_name());
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  const auto result = rclcpp::spin_until_future_complete(
    node.get_node_base_interface(), future, std::chrono::seconds(5));
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(node.get_logger(), "Camera %s request timed out.", start ? "start" : "stop");
    return;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_WARN(
      node.get_logger(), "Camera %s request failed: %s",
      start ? "start" : "stop", response->message.c_str());
  }
}
